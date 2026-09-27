// TileService against a tiny HTTP server on 127.0.0.1 (in this process), so
// the download path is tested without the internet. Download tests are
// skipped when the build has no libcurl; the disk-cache tests always run.

#include <catch2/catch_test_macros.hpp>

#include <basestation/core/http_client.h>
#include <basestation/core/tile_service.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

using namespace basestation;
using namespace basestation::tiles;
using namespace std::chrono_literals;
namespace fs = std::filesystem;

namespace {

const uint8_t PNG_MAGIC[8] = {0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A};

// Serves /t/{z}/{x}/{y}.png (y == 99: 404) and /html/... (200, text/html).
// One thread per connection, so concurrency can be measured.
class TestServer {
public:
    TestServer() {
        fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
        int one = 1;
        ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
        sockaddr_in a{};
        a.sin_family = AF_INET;
        a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        a.sin_port = 0;
        if (::bind(fd_, reinterpret_cast<sockaddr*>(&a), sizeof a) != 0 || ::listen(fd_, 16) != 0) return;
        socklen_t len = sizeof a;
        ::getsockname(fd_, reinterpret_cast<sockaddr*>(&a), &len);
        port_ = ntohs(a.sin_port);
        thread_ = std::thread([this] { run(); });
    }
    ~TestServer() {
        stop_ = true;
        if (thread_.joinable()) thread_.join();
        for (std::thread& t : conns_) t.join();
        if (fd_ >= 0) ::close(fd_);
    }

    std::string url(const std::string& prefix = "t") const {
        return "http://127.0.0.1:" + std::to_string(port_) + "/" + prefix + "/{z}/{x}/{y}.png";
    }
    bool ok() const { return port_ != 0; }
    int count(const std::string& path) {
        std::lock_guard<std::mutex> lk(mu_);
        return hits_[path];
    }
    int total() {
        std::lock_guard<std::mutex> lk(mu_);
        int n = 0;
        for (auto& [p, c] : hits_) n += c;
        return n;
    }
    std::string last_user_agent() {
        std::lock_guard<std::mutex> lk(mu_);
        return user_agent_;
    }
    int max_concurrent() const { return max_concurrent_; }
    std::atomic<int> delay_ms{0};

private:
    void run() {
        while (!stop_) {
            pollfd p{fd_, POLLIN, 0};
            if (::poll(&p, 1, 20) <= 0) continue;
            const int c = ::accept(fd_, nullptr, nullptr);
            if (c < 0) continue;
            conns_.emplace_back([this, c] { serve(c); });
        }
    }

    void serve(int c) {
#ifdef SO_NOSIGPIPE
        int one = 1;
        ::setsockopt(c, SOL_SOCKET, SO_NOSIGPIPE, &one, sizeof one);
#endif
        const int now = ++concurrent_;
        int prev = max_concurrent_;
        while (now > prev && !max_concurrent_.compare_exchange_weak(prev, now)) {}

        std::string req;
        char buf[1024];
        while (req.find("\r\n\r\n") == std::string::npos) {
            pollfd p{c, POLLIN, 0};
            if (::poll(&p, 1, 2000) <= 0) break;
            const ssize_t n = ::recv(c, buf, sizeof buf, 0);
            if (n <= 0) break;
            req.append(buf, static_cast<size_t>(n));
        }
        const size_t sp1 = req.find(' ');
        const size_t sp2 = req.find(' ', sp1 + 1);
        const std::string path = sp1 == std::string::npos ? "" : req.substr(sp1 + 1, sp2 - sp1 - 1);
        {
            std::lock_guard<std::mutex> lk(mu_);
            ++hits_[path];
            const size_t ua = req.find("User-Agent: ");
            if (ua != std::string::npos) user_agent_ = req.substr(ua + 12, req.find("\r\n", ua) - ua - 12);
        }
        if (delay_ms > 0) std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms.load()));

        std::string status = "200 OK", type = "image/png", body;
        if (path.rfind("/html/", 0) == 0) {
            type = "text/html";
            body = "<html>captive portal</html>";
        } else if (path.find("/99.png") != std::string::npos) {
            status = "404 Not Found";
            type = "text/plain";
            body = "no such tile";
        } else {
            body.assign(reinterpret_cast<const char*>(PNG_MAGIC), sizeof PNG_MAGIC);
            body += path;  // unique content per tile
        }
        // Done "being in progress" before the client can see the response,
        // so its next request can never overlap this one in the count.
        --concurrent_;
        const std::string resp = "HTTP/1.1 " + status + "\r\nContent-Type: " + type +
                                 "\r\nContent-Length: " + std::to_string(body.size()) +
                                 "\r\nConnection: close\r\n\r\n" + body;
#ifdef MSG_NOSIGNAL
        ::send(c, resp.data(), resp.size(), MSG_NOSIGNAL);
#else
        ::send(c, resp.data(), resp.size(), 0);
#endif
        ::close(c);
    }

    int fd_ = -1;
    uint16_t port_ = 0;
    std::atomic<bool> stop_{false};
    std::thread thread_;
    std::vector<std::thread> conns_;  // only touched by run(), joined after it ends
    std::mutex mu_;
    std::map<std::string, int> hits_;
    std::string user_agent_;
    std::atomic<int> concurrent_{0};
    std::atomic<int> max_concurrent_{0};
};

// Temporary cache directory, removed afterwards.
struct TempDir {
    fs::path path;
    TempDir() {
        std::random_device rd;
        path = fs::temp_directory_path() / ("asep-tiles-test-" + std::to_string(rd()) + std::to_string(rd()));
        fs::create_directories(path);
    }
    ~TempDir() {
        std::error_code ec;
        fs::remove_all(path, ec);
    }
};

// Accepts anything with a PNG signature; the "image" is 1x1.
bool fake_decode(const std::vector<uint8_t>& bytes, TileImage& out) {
    if (bytes.size() < 8 || std::memcmp(bytes.data(), PNG_MAGIC, 8) != 0) return false;
    out.width = out.height = 1;
    out.rgba = {1, 2, 3, 255};
    return true;
}

TileService::Config fast_config(const std::string& dir) {
    TileService::Config c;
    c.cache_dir = dir;
    c.user_agent = user_agent("test");
    c.max_requests_per_s = 200;  // keep the tests quick
    return c;
}

// Keeps requesting (like the UI does every frame) until an image arrives.
std::vector<TileImage> wait_for(TileService& s, const TileSource& src, std::vector<TileId> tiles,
                                std::chrono::milliseconds timeout = 5000ms) {
    std::vector<TileImage> got;
    const auto deadline = Clock::now() + timeout;
    uint64_t frame = 0;
    while (Clock::now() < deadline && got.size() < tiles.size()) {
        ++frame;
        for (const TileId& t : tiles) {
            const bool have = std::any_of(got.begin(), got.end(), [&](const TileImage& i) { return i.key.tile == t; });
            if (!have) s.request(src, t, frame);
        }
        for (TileImage& i : s.take_ready()) got.push_back(std::move(i));
        std::this_thread::sleep_for(5ms);
    }
    return got;
}

template <class F>
bool eventually(F done, std::chrono::milliseconds timeout = 5000ms) {
    const auto deadline = Clock::now() + timeout;
    while (Clock::now() < deadline) {
        if (done()) return true;
        std::this_thread::sleep_for(5ms);
    }
    return done();
}

void write_tile(const std::string& dir, const TileSource& src, TileId t) {
    const std::string p = cache_path(dir, src.id, t, src.ext);
    fs::create_directories(fs::path(p).parent_path());
    std::ofstream(p, std::ios::binary).write(reinterpret_cast<const char*>(PNG_MAGIC), 8);
}

}  // namespace

TEST_CASE("TileService: user agent", "[tiles][service]") {
    CHECK(user_agent("1.2.3") == "ASEP-base-station/1.2.3 (+https://github.com/spseng/ASEP-boat-base-station)");
}

TEST_CASE("TileService: cached tiles load offline, missing ones are remembered", "[tiles][service]") {
    TempDir dir;
    const TileSource src = custom_source("http://127.0.0.1:1/{z}/{x}/{y}.png");
    write_tile(dir.path.string(), src, {5, 1, 2});
    // A corrupt cache file is removed rather than shown.
    {
        const std::string bad = cache_path(dir.path.string(), src.id, {5, 1, 3}, src.ext);
        fs::create_directories(fs::path(bad).parent_path());
        std::ofstream(bad) << "garbage";
    }

    auto cfg = fast_config(dir.path.string());
    cfg.retry_after_cache_miss = 60000ms;
    TileService s(cfg, fake_decode);
    s.set_offline(true);
    const auto got = wait_for(s, src, {{5, 1, 2}});
    REQUIRE(got.size() == 1);
    CHECK(got[0].key.source_id == src.id);
    CHECK(got[0].key.tile == TileId{5, 1, 2});
    CHECK(got[0].width == 1);
    CHECK(got[0].rgba.size() == 4);

    CHECK(wait_for(s, src, {{5, 1, 3}}, 300ms).empty());
    CHECK_FALSE(fs::exists(cache_path(dir.path.string(), src.id, {5, 1, 3}, src.ext)));
    CHECK(s.net_status().queued == 0);

    s.rescan_cache();
    CHECK(eventually([&] { return s.cache_stats().valid && !s.cache_stats().scanning; }));
    CHECK(s.cache_stats().files == 1);
    CHECK(s.cache_stats().bytes == 8);

    std::string err;
    CHECK_FALSE(s.start_prefetch(src, {{5, 1, 4}}, &err));
    CHECK_FALSE(err.empty());
}

TEST_CASE("TileService: downloads once, caches, and serves from disk", "[tiles][service][net]") {
    if (!http::available()) SKIP("built without libcurl");
    TestServer server;
    REQUIRE(server.ok());
    TempDir dir;
    const TileSource src = custom_source(server.url());

    {
        TileService s(fast_config(dir.path.string()), fake_decode);
        const auto got = wait_for(s, src, {{16, 19820, 24250}, {16, 19821, 24250}});
        CHECK(got.size() == 2);
        CHECK(server.count("/t/16/19820/24250.png") == 1);
        CHECK(server.count("/t/16/19821/24250.png") == 1);
        CHECK(server.last_user_agent() == user_agent("test"));
        CHECK(fs::exists(cache_path(dir.path.string(), src.id, {16, 19820, 24250}, "png")));
        CHECK(s.net_status().downloaded == 2);
        // Asking again (the texture was evicted, say) reads the disk.
        CHECK(wait_for(s, src, {{16, 19820, 24250}}).size() == 1);
        CHECK(server.total() == 2);
    }
    // A new session, online: still no second download.
    TileService s(fast_config(dir.path.string()), fake_decode);
    CHECK(wait_for(s, src, {{16, 19820, 24250}}).size() == 1);
    CHECK(server.total() == 2);
}

TEST_CASE("TileService: a cache-only miss does not block a real request", "[tiles][service][net]") {
    if (!http::available()) SKIP("built without libcurl");
    TestServer server;
    REQUIRE(server.ok());
    TempDir dir;
    const TileSource src = custom_source(server.url());
    auto cfg = fast_config(dir.path.string());
    cfg.retry_after_cache_miss = 60000ms;
    TileService s(cfg, fake_decode);
    // As a lower-zoom fallback: never downloads.
    for (uint64_t frame = 1; frame < 40; ++frame) {
        s.request(src, {15, 3, 4}, frame, true);
        std::this_thread::sleep_for(5ms);
    }
    CHECK(s.take_ready().empty());
    CHECK(server.total() == 0);
    // Then wanted for real (the view zoomed out to it): downloaded at once.
    CHECK(wait_for(s, src, {{15, 3, 4}}, 2000ms).size() == 1);
    CHECK(server.total() == 1);
}

TEST_CASE("TileService: failures are remembered, not retried every frame", "[tiles][service][net]") {
    if (!http::available()) SKIP("built without libcurl");
    TestServer server;
    REQUIRE(server.ok());
    TempDir dir;
    TileService s(fast_config(dir.path.string()), fake_decode);

    SECTION("404") {
        const TileSource src = custom_source(server.url());
        CHECK(wait_for(s, src, {{10, 5, 99}}, 800ms).empty());
        CHECK(server.count("/t/10/5/99.png") == 1);
        CHECK(s.net_status().failed == 1);
        CHECK(s.net_status().last_error.find("HTTP 404") != std::string::npos);
    }
    SECTION("200 that is not an image is not cached") {
        const TileSource src = custom_source(server.url("html"));
        CHECK(wait_for(s, src, {{10, 5, 6}}, 800ms).empty());
        CHECK(server.count("/html/10/5/6.png") == 1);
        CHECK_FALSE(fs::exists(cache_path(dir.path.string(), src.id, {10, 5, 6}, src.ext)));
        CHECK(s.net_status().last_error.find("not a PNG/JPEG") != std::string::npos);
    }
}

TEST_CASE("TileService: repeated network failures pause downloads", "[tiles][service][net]") {
    if (!http::available()) SKIP("built without libcurl");
    TempDir dir;
    auto cfg = fast_config(dir.path.string());
    cfg.pause_after_failures = 2;
    cfg.pause = 10000ms;
    cfg.connect_timeout_s = 2;
    cfg.download_threads = 1;
    TileService s(cfg, fake_decode);
    // Nothing listens on port 1: connection refused, a transport error.
    const TileSource src = custom_source("http://127.0.0.1:1/{z}/{x}/{y}.png");
    wait_for(s, src, {{3, 1, 1}, {3, 1, 2}, {3, 1, 3}, {3, 1, 4}}, 1000ms);
    const auto st = s.net_status();
    CHECK(st.failed == 2);
    CHECK(st.paused_s > 5.0);
    CHECK(st.last_error.find("pausing") != std::string::npos);
}

TEST_CASE("TileService: offline mode never touches the network", "[tiles][service][net]") {
    if (!http::available()) SKIP("built without libcurl");
    TestServer server;
    REQUIRE(server.ok());
    TempDir dir;
    const TileSource src = custom_source(server.url());
    TileService s(fast_config(dir.path.string()), fake_decode);
    s.set_offline(true);
    CHECK(s.offline());
    CHECK(wait_for(s, src, {{12, 1, 1}}, 500ms).empty());
    std::string err;
    CHECK_FALSE(s.start_prefetch(src, {{12, 1, 2}}, &err));
    CHECK(err == "Offline mode is on");
    CHECK(server.total() == 0);

    // Back online, the same tile is fetched right away (the "not cached"
    // memory from offline mode is dropped).
    s.set_offline(false);
    CHECK(wait_for(s, src, {{12, 1, 1}}).size() == 1);
    CHECK(server.total() == 1);
}

TEST_CASE("TileService: area prefetch skips cached tiles, limits concurrency", "[tiles][service][net]") {
    if (!http::available()) SKIP("built without libcurl");
    TestServer server;
    REQUIRE(server.ok());
    server.delay_ms = 30;
    TempDir dir;
    const TileSource src = custom_source(server.url());
    const PrefetchPlan plan = plan_prefetch(box_around({42.316, -71.1205}, 400), 14, 16);
    REQUIRE(plan.ok);
    REQUIRE(plan.tiles.size() >= 6);
    write_tile(dir.path.string(), src, plan.tiles[0]);
    write_tile(dir.path.string(), src, plan.tiles[1]);

    auto cfg = fast_config(dir.path.string());
    cfg.download_threads = 2;
    TileService s(cfg, fake_decode);
    std::string err;
    REQUIRE(s.start_prefetch(src, plan.tiles, &err));
    CHECK_FALSE(s.start_prefetch(src, plan.tiles, &err));  // one job at a time
    CHECK(s.prefetch_status().active);
    CHECK(s.prefetch_status().total == plan.tiles.size());

    REQUIRE(eventually([&] { return !s.prefetch_status().active; }, 10000ms));
    const auto st = s.prefetch_status();
    CHECK(st.cached == 2);
    CHECK(st.downloaded == plan.tiles.size() - 2);
    CHECK(st.failed == 0);
    CHECK(st.processed() == st.total);
    CHECK(st.message.rfind("Done", 0) == 0);
    CHECK(server.total() == static_cast<int>(plan.tiles.size()) - 2);
    CHECK(server.max_concurrent() <= 2);
    for (const TileId& t : plan.tiles) CHECK(fs::exists(cache_path(dir.path.string(), src.id, t, src.ext)));

    // Cache accounting and clearing (only the source directories).
    fs::create_directories(dir.path / "keepme");
    std::ofstream(dir.path / "keepme" / "notes.txt") << "not a tile";
    s.rescan_cache();
    REQUIRE(eventually([&] { return !s.cache_stats().scanning; }));
    CHECK(s.cache_stats().files == plan.tiles.size());
    s.clear_cache();
    REQUIRE(eventually([&] { return !s.cache_stats().scanning && s.cache_stats().files == 0; }));
    CHECK_FALSE(fs::exists(dir.path / src.id));
    CHECK(fs::exists(dir.path / "keepme" / "notes.txt"));
}

TEST_CASE("TileService: prefetch can be cancelled", "[tiles][service][net]") {
    if (!http::available()) SKIP("built without libcurl");
    TestServer server;
    REQUIRE(server.ok());
    server.delay_ms = 100;
    TempDir dir;
    const TileSource src = custom_source(server.url());
    const PrefetchPlan plan = plan_prefetch(box_around({42.316, -71.1205}, 2000), 15, 16);
    REQUIRE(plan.ok);
    REQUIRE(plan.tiles.size() > 20);

    auto cfg = fast_config(dir.path.string());
    cfg.max_requests_per_s = 20;
    TileService s(cfg, fake_decode);
    std::string err;
    REQUIRE(s.start_prefetch(src, plan.tiles, &err));
    REQUIRE(eventually([&] { return s.prefetch_status().processed() >= 2; }));
    s.cancel_prefetch();
    CHECK_FALSE(s.prefetch_status().active);
    CHECK(s.prefetch_status().message == "Cancelled");
    std::this_thread::sleep_for(400ms);  // in-flight requests finish, nothing new starts
    const int after = server.total();
    std::this_thread::sleep_for(300ms);
    CHECK(server.total() == after);
    CHECK(after < static_cast<int>(plan.tiles.size()));

    // A new job can start after a cancel.
    REQUIRE(s.start_prefetch(src, {plan.tiles.back()}, &err));
    REQUIRE(eventually([&] { return !s.prefetch_status().active; }));
    CHECK(s.prefetch_status().processed() == 1);
}
