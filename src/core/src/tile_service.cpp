#include <basestation/core/tile_service.h>

#include <basestation/core/http_client.h>

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>

namespace basestation::tiles {

namespace fs = std::filesystem;

namespace {

bool read_file(const std::string& path, std::vector<uint8_t>& out) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return false;
    out.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
    return !in.bad();
}

// Temp file + rename, so a crash or a concurrent reader never sees half a
// tile.
bool write_file_atomic(const std::string& path, const std::vector<uint8_t>& data, std::string* error) {
    static std::atomic<unsigned> counter{0};
    std::error_code ec;
    fs::create_directories(fs::path(path).parent_path(), ec);
    if (ec) {
        *error = "cannot create directory for " + path + ": " + ec.message();
        return false;
    }
    const std::string tmp = path + ".tmp" + std::to_string(counter++);
    {
        std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
        out.write(reinterpret_cast<const char*>(data.data()), static_cast<std::streamsize>(data.size()));
        out.flush();
        if (!out) {
            *error = "cannot write " + tmp;
            out.close();
            std::remove(tmp.c_str());
            return false;
        }
    }
    if (std::rename(tmp.c_str(), path.c_str()) != 0) {
        *error = "cannot rename " + tmp;
        std::remove(tmp.c_str());
        return false;
    }
    return true;
}

bool file_exists(const std::string& path) {
    std::error_code ec;
    return fs::is_regular_file(path, ec);
}

// Cache directory names this service may scan and delete: built-in source
// ids and custom-<8 hex>. Anything else in cache_dir is left alone, in case
// the operator points the cache at a directory with other files in it.
bool is_source_dir_name(const std::string& name) {
    if (find_builtin_source(name)) return true;
    const std::string prefix = std::string(CUSTOM_SOURCE_ID) + "-";
    if (name.size() != prefix.size() + 8 || name.compare(0, prefix.size(), prefix) != 0) return false;
    return std::all_of(name.begin() + static_cast<std::ptrdiff_t>(prefix.size()), name.end(),
                       [](char c) { return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'); });
}

}  // namespace

std::string user_agent(const std::string& version) {
    return "ASEP-base-station/" + version + " (+https://github.com/spseng/ASEP-boat-base-station)";
}

TileService::TileService(Config cfg, Decoder decoder)
    : cfg_(std::move(cfg)), decoder_(std::move(decoder)), http_ok_(http::available()) {
    http::global_init();  // before any downloader thread exists
    scan_requested_ = true;
    cache_.scanning = true;
    loader_ = std::thread([this] { loader_main(); });
    if (http_ok_) {
        for (int i = 0; i < std::max(1, cfg_.download_threads); ++i)
            downloaders_.emplace_back([this] { downloader_main(); });
    }
}

TileService::~TileService() {
    {
        std::lock_guard<std::mutex> lk(mu_);
        stop_ = true;
        abort_net_ = true;
    }
    cv_load_.notify_all();
    cv_net_.notify_all();
    loader_.join();
    for (std::thread& t : downloaders_) t.join();
}

// ---------------------------------------------------------------------------
// UI-thread API
// ---------------------------------------------------------------------------

void TileService::request(const TileSource& src, TileId t, uint64_t priority, bool cache_only) {
    TileKey key{src.id, t};
    std::lock_guard<std::mutex> lk(mu_);
    latest_priority_ = std::max(latest_priority_, priority);
    // Already waiting: refresh its priority (and upgrade a cache-only
    // request to a full one).
    for (auto* q : {&load_q_, &net_q_}) {
        auto it = q->find(key);
        if (it != q->end()) {
            it->second.priority = std::max(it->second.priority, priority);
            it->second.cache_only = it->second.cache_only && cache_only;
            return;
        }
    }
    if (busy_.count(key)) return;
    auto f = failed_.find(key);
    if (f != failed_.end()) {
        // A tile that was only missing from the cache may still be
        // downloaded when it is wanted for real (not as a fallback).
        const bool may_download = f->second.cache_miss && !cache_only && downloads_allowed();
        if (Clock::now() < f->second.until && !may_download) return;
        failed_.erase(f);
    }
    load_q_[key] = Job{std::make_shared<const TileSource>(src), priority, cache_only};
    cv_load_.notify_one();
}

std::vector<TileImage> TileService::take_ready(size_t max) {
    std::lock_guard<std::mutex> lk(mu_);
    std::vector<TileImage> out;
    if (ready_.size() <= max) {
        out.swap(ready_);
    } else {
        out.assign(std::make_move_iterator(ready_.begin()),
                   std::make_move_iterator(ready_.begin() + static_cast<std::ptrdiff_t>(max)));
        ready_.erase(ready_.begin(), ready_.begin() + static_cast<std::ptrdiff_t>(max));
    }
    for (const TileImage& img : out) busy_.erase(img.key);
    return out;
}

void TileService::set_offline(bool offline) {
    {
        std::lock_guard<std::mutex> lk(mu_);
        if (offline == offline_) return;
        offline_ = offline;
        abort_net_ = offline;
        if (offline) {
            net_q_.clear();
            if (pf_.active) {
                pf_q_.clear();
                pf_.active = false;
                pf_.message = "Stopped: offline mode";
            }
        } else {
            // Tiles that were missing only because we were offline may be
            // downloadable now.
            for (auto it = failed_.begin(); it != failed_.end();)
                it = it->second.cache_miss ? failed_.erase(it) : std::next(it);
            consecutive_failures_ = 0;
            paused_until_ = {};
        }
    }
    cv_net_.notify_all();
}

bool TileService::offline() const {
    std::lock_guard<std::mutex> lk(mu_);
    return offline_;
}

bool TileService::start_prefetch(const TileSource& src, std::vector<TileId> tiles, std::string* error) {
    std::lock_guard<std::mutex> lk(mu_);
    auto fail = [&](const char* why) {
        if (error) *error = why;
        return false;
    };
    if (!http_ok_) return fail("This build has no libcurl, so it cannot download tiles");
    if (offline_) return fail("Offline mode is on");
    if (pf_.active) return fail("A download is already running");
    if (tiles.empty()) return fail("Nothing to download");
    pf_src_ = std::make_shared<const TileSource>(src);
    pf_q_.assign(tiles.begin(), tiles.end());
    pf_ = PrefetchStatus{};
    ++pf_gen_;
    pf_.active = true;
    pf_.source_name = src.name;
    pf_.total = tiles.size();
    cv_net_.notify_all();
    return true;
}

void TileService::cancel_prefetch() {
    std::lock_guard<std::mutex> lk(mu_);
    if (!pf_.active) return;
    pf_q_.clear();
    pf_.active = false;
    pf_.message = "Cancelled";
}

TileService::PrefetchStatus TileService::prefetch_status() const {
    std::lock_guard<std::mutex> lk(mu_);
    return pf_;
}

void TileService::rescan_cache() {
    std::lock_guard<std::mutex> lk(mu_);
    scan_requested_ = true;
    cache_.scanning = true;
    cv_load_.notify_one();
}

void TileService::clear_cache() {
    std::lock_guard<std::mutex> lk(mu_);
    clear_requested_ = true;
    cache_.scanning = true;
    cv_load_.notify_one();
}

TileService::CacheStats TileService::cache_stats() const {
    std::lock_guard<std::mutex> lk(mu_);
    return cache_;
}

TileService::NetStatus TileService::net_status() const {
    std::lock_guard<std::mutex> lk(mu_);
    NetStatus s;
    s.downloads_available = http_ok_;
    s.offline = offline_;
    s.downloaded = downloaded_;
    s.failed = failed_count_;
    s.queued = net_q_.size();
    const Clock::time_point now = Clock::now();
    if (paused_until_ > now) s.paused_s = std::chrono::duration<double>(paused_until_ - now).count();
    s.last_error = last_error_;
    return s;
}

// ---------------------------------------------------------------------------
// Worker threads
// ---------------------------------------------------------------------------

bool TileService::pop_best(std::map<TileKey, Job>& q, TileKey& key, Job& job) {
    while (!q.empty()) {
        auto best = q.begin();
        for (auto it = q.begin(); it != q.end(); ++it)
            if (it->second.priority > best->second.priority) best = it;
        const bool stale = best->second.priority + cfg_.stale_after < latest_priority_;
        if (stale) {
            q.erase(best);  // off screen by now; requested again if it comes back
            continue;
        }
        key = best->first;
        job = std::move(best->second);
        q.erase(best);
        return true;
    }
    return false;
}

void TileService::loader_main() {
    std::unique_lock<std::mutex> lk(mu_);
    for (;;) {
        cv_load_.wait(lk, [&] { return stop_ || clear_requested_ || scan_requested_ || !load_q_.empty(); });
        if (stop_) return;

        if (clear_requested_) {
            clear_requested_ = false;
            scan_requested_ = false;
            lk.unlock();
            do_clear();
            lk.lock();
            continue;
        }
        if (scan_requested_) {
            scan_requested_ = false;
            lk.unlock();
            do_scan();
            lk.lock();
            continue;
        }

        TileKey key;
        Job job;
        if (!pop_best(load_q_, key, job)) continue;
        busy_.insert(key);
        lk.unlock();

        const std::string path = cache_path(cfg_.cache_dir, key.source_id, key.tile, job.src->ext);
        std::vector<uint8_t> bytes;
        TileImage img;
        img.key = key;
        const bool on_disk = read_file(path, bytes) && !bytes.empty();
        bool decoded = on_disk && decoder_ && decoder_(bytes, img) && img.width > 0 && img.height > 0 &&
                       img.rgba.size() == size_t(img.width) * size_t(img.height) * 4;
        if (on_disk && !decoded) std::remove(path.c_str());  // corrupt: fetch it again

        lk.lock();
        if (decoded) {
            ready_.push_back(std::move(img));  // stays busy_ until taken, so it is not loaded twice
            continue;
        }
        busy_.erase(key);
        if (!job.cache_only && downloads_allowed()) {
            net_q_[key] = std::move(job);
            cv_net_.notify_one();
        } else {
            failed_[key] = Failure{Clock::now() + cfg_.retry_after_cache_miss, true};
        }
    }
}

void TileService::downloader_main() {
    http::Options opts;
    opts.user_agent = cfg_.user_agent;
    opts.connect_timeout_s = cfg_.connect_timeout_s;
    opts.timeout_s = cfg_.timeout_s;
    const auto interval = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(1.0 / std::max(0.1, cfg_.max_requests_per_s)));

    std::unique_lock<std::mutex> lk(mu_);
    for (;;) {
        if (stop_) return;
        if (!downloads_allowed() || (net_q_.empty() && pf_q_.empty())) {
            cv_net_.wait(lk);
            continue;
        }
        // Politeness: one request start per interval across all threads,
        // and nothing at all while paused after repeated failures.
        const Clock::time_point slot = std::max(next_request_at_, paused_until_);
        if (Clock::now() < slot) {
            cv_net_.wait_until(lk, slot);
            continue;
        }

        TileKey key;
        Job job;
        bool prefetch = false;
        const uint64_t gen = pf_gen_;  // results of a cancelled job must not count toward the next
        if (!pop_best(net_q_, key, job)) {
            if (pf_q_.empty()) continue;
            prefetch = true;
            key = TileKey{pf_src_->id, pf_q_.front()};
            job = Job{pf_src_, 0, false};
            pf_q_.pop_front();
            // Never download what is cached (or being fetched right now).
            if (busy_.count(key) || file_exists(cache_path(cfg_.cache_dir, key.source_id, key.tile, job.src->ext))) {
                ++pf_.cached;
                finish_prefetch_if_done();
                continue;
            }
            auto f = failed_.find(key);
            if (f != failed_.end() && Clock::now() < f->second.until && !f->second.cache_miss) {
                ++pf_.failed;
                finish_prefetch_if_done();
                continue;
            }
            ++pf_in_flight_;
        }
        next_request_at_ = Clock::now() + interval;
        busy_.insert(key);
        lk.unlock();

        const std::string url = tile_url(job.src->url_template, key.tile);
        const std::string path = cache_path(cfg_.cache_dir, key.source_id, key.tile, job.src->ext);
        http::Response resp = http::get(url, opts, &abort_net_);
        std::string error = resp.error;
        bool saved = false;
        if (resp.ok) {
            if (!looks_like_image(resp.body.data(), resp.body.size())) {
                error = "not a PNG/JPEG image" + (resp.content_type.empty() ? "" : " (" + resp.content_type + ")");
            } else {
                saved = write_file_atomic(path, resp.body, &error);
            }
        }

        lk.lock();
        busy_.erase(key);
        if (prefetch) --pf_in_flight_;
        const bool counts = prefetch && gen == pf_gen_ && pf_.active;
        const Clock::time_point now = Clock::now();
        if (saved) {
            consecutive_failures_ = 0;
            ++downloaded_;
            failed_.erase(key);
            if (cache_.valid) {
                ++cache_.files;
                cache_.bytes += resp.body.size();
            }
            if (counts) {
                ++pf_.downloaded;
            } else if (!prefetch) {
                load_q_[key] = std::move(job);  // decode it from disk
                cv_load_.notify_one();
            }
        } else if (error == "cancelled") {
            // Offline mode or shutdown; not the tile's fault.
            if (counts) ++pf_.failed;
        } else {
            ++failed_count_;
            last_error_ = url + ": " + error;
            // 4xx: the server has no such tile, ask again much later.
            // Anything else may be transient.
            const bool not_found = resp.status >= 400 && resp.status < 500 && resp.status != 429;
            failed_[key] = Failure{now + (not_found ? cfg_.retry_after_not_found : cfg_.retry_after_error), false};
            if (resp.transport_error || resp.status == 429 || resp.status >= 500) ++consecutive_failures_;
            if (consecutive_failures_ >= cfg_.pause_after_failures || resp.status == 429) {
                // No retry storms when the network is down or the server
                // asks us to slow down.
                paused_until_ = now + cfg_.pause;
                consecutive_failures_ = 0;
                last_error_ += " (pausing downloads)";
            }
            if (counts) ++pf_.failed;
        }
        if (prefetch) finish_prefetch_if_done();
    }
}

void TileService::finish_prefetch_if_done() {
    if (!pf_.active || !pf_q_.empty() || pf_in_flight_ > 0) return;
    pf_.active = false;
    pf_.message = "Done: " + std::to_string(pf_.downloaded) + " downloaded, " + std::to_string(pf_.cached) +
                  " already cached, " + std::to_string(pf_.failed) + " failed";
}

// ---------------------------------------------------------------------------
// Cache maintenance (loader thread)
// ---------------------------------------------------------------------------

std::vector<std::string> TileService::source_dirs() const {
    std::vector<std::string> dirs;
    std::error_code ec;
    for (fs::directory_iterator it(cfg_.cache_dir, ec), end; !ec && it != end; it.increment(ec)) {
        std::error_code ec2;
        if (it->is_directory(ec2) && is_source_dir_name(it->path().filename().string()))
            dirs.push_back(it->path().string());
    }
    return dirs;
}

void TileService::do_scan() {
    CacheStats s;
    for (const std::string& dir : source_dirs()) {
        std::error_code ec;
        for (fs::recursive_directory_iterator it(dir, fs::directory_options::skip_permission_denied, ec), end;
             !ec && it != end; it.increment(ec)) {
            std::error_code ec2;
            if (!it->is_regular_file(ec2)) continue;
            const auto size = it->file_size(ec2);
            if (ec2) continue;
            ++s.files;
            s.bytes += size;
        }
    }
    s.valid = true;
    std::lock_guard<std::mutex> lk(mu_);
    if (!scan_requested_ && !clear_requested_) {
        cache_ = s;
    }
}

void TileService::do_clear() {
    for (const std::string& dir : source_dirs()) {
        std::error_code ec;
        fs::remove_all(dir, ec);
    }
    {
        std::lock_guard<std::mutex> lk(mu_);
        // Previously cached tiles are now misses; forget the "not cached"
        // memory too so they can be fetched again.
        for (auto it = failed_.begin(); it != failed_.end();)
            it = it->second.cache_miss ? failed_.erase(it) : std::next(it);
    }
    do_scan();
}

}  // namespace basestation::tiles
