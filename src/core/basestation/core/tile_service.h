#pragma once

// Background loading and downloading of map tiles.
//
// One loader thread reads cached tiles from disk and decodes them; up to two
// downloader threads fetch missing tiles over HTTP(S), politely: at most
// Config::max_requests_per_s across both, a pause after repeated network
// failures, failed tiles remembered for a while, and nothing re-downloaded
// that is already cached. The UI thread only queues requests and collects
// decoded images, so it never waits on the network or the disk.
//
// Offline mode never touches the network: only cached tiles are shown.

#include <basestation/core/common.h>
#include <basestation/core/map_tiles.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

namespace basestation::tiles {

// A decoded tile, RGBA8, row-major, width * height * 4 bytes.
struct TileImage {
    TileKey key;
    int width = 0;
    int height = 0;
    std::vector<uint8_t> rgba;
};

// Turns encoded image bytes (PNG / JPEG) into RGBA. Runs on the loader
// thread. Injected so core does not depend on an image library.
using Decoder = std::function<bool(const std::vector<uint8_t>& encoded, TileImage& out)>;

// "ASEP-base-station/<version> (+https://github.com/spseng/ASEP-boat-base-station)"
std::string user_agent(const std::string& version);

class TileService {
public:
    struct Config {
        std::string cache_dir;
        std::string user_agent;
        int download_threads = 2;          // concurrent downloads; 2 is the polite maximum
        double max_requests_per_s = 8.0;   // across all download threads
        std::chrono::milliseconds retry_after_error{120000};      // network error, 5xx
        std::chrono::milliseconds retry_after_not_found{1800000}; // 4xx: the server has no such tile
        std::chrono::milliseconds retry_after_cache_miss{10000};  // offline / cache-only miss
        int pause_after_failures = 4;      // consecutive network failures ...
        std::chrono::milliseconds pause{30000};  // ... pause all downloads this long
        // A queued on-screen request is dropped if it was not repeated within
        // this many priority steps (frames): the view has moved on.
        uint64_t stale_after = 30;
        long connect_timeout_s = 10;
        long timeout_s = 30;
    };

    struct PrefetchStatus {
        bool active = false;
        std::string source_name;
        size_t total = 0;
        size_t downloaded = 0;
        size_t cached = 0;   // already on disk, skipped
        size_t failed = 0;
        std::string message; // result of the last job ("Done", "Cancelled", ...)
        size_t processed() const { return downloaded + cached + failed; }
    };

    struct CacheStats {
        bool valid = false;     // a scan has completed
        bool scanning = false;
        uint64_t files = 0;
        uint64_t bytes = 0;
    };

    struct NetStatus {
        bool downloads_available = false;  // built with libcurl
        bool offline = false;
        uint64_t downloaded = 0;
        uint64_t failed = 0;
        size_t queued = 0;                 // on-screen tiles waiting to download
        double paused_s = 0;               // remaining pause after repeated failures
        std::string last_error;
    };

    TileService(Config cfg, Decoder decoder);
    ~TileService();
    TileService(const TileService&) = delete;
    TileService& operator=(const TileService&) = delete;

    // ---- UI thread -----------------------------------------------------------

    // Asks for a tile. Cheap and idempotent: call it every frame for every
    // tile on screen, with a priority that grows every frame (the frame
    // counter); the newest requests are served first. Tiles already queued,
    // in progress or recently failed are ignored. cache_only never
    // downloads (used for lower-zoom fallbacks).
    void request(const TileSource& src, TileId t, uint64_t priority, bool cache_only = false);
    // Decoded tiles since the last call (at most `max`).
    std::vector<TileImage> take_ready(size_t max = SIZE_MAX);

    void set_offline(bool offline);
    bool offline() const;

    // Downloads every tile in `tiles` that is not cached yet, in the
    // background, after on-screen tiles. Fails if offline, without libcurl,
    // or while another job runs.
    bool start_prefetch(const TileSource& src, std::vector<TileId> tiles, std::string* error);
    void cancel_prefetch();
    PrefetchStatus prefetch_status() const;

    // Cache size is computed in the background (rescan_cache) and kept up
    // to date as tiles are downloaded. clear_cache deletes only the
    // directories of known sources inside cache_dir.
    void rescan_cache();
    void clear_cache();
    CacheStats cache_stats() const;
    NetStatus net_status() const;
    const std::string& cache_dir() const { return cfg_.cache_dir; }

private:
    struct Job {
        std::shared_ptr<const TileSource> src;
        uint64_t priority = 0;
        bool cache_only = false;
    };
    struct Failure {
        Clock::time_point until;
        bool cache_miss = false;  // forgotten when going back online
    };

    void loader_main();
    void downloader_main();
    // Highest-priority job, dropping stale ones. Caller holds mu_.
    bool pop_best(std::map<TileKey, Job>& q, TileKey& key, Job& job);
    bool downloads_allowed() const { return !offline_ && !stop_ && http_ok_; }
    void finish_prefetch_if_done();
    void do_scan();
    void do_clear();
    std::vector<std::string> source_dirs() const;

    const Config cfg_;
    const Decoder decoder_;
    const bool http_ok_;

    mutable std::mutex mu_;
    std::condition_variable cv_load_;
    std::condition_variable cv_net_;
    bool stop_ = false;
    bool offline_ = false;
    std::atomic<bool> abort_net_{false};  // cancels transfers in flight

    std::map<TileKey, Job> load_q_;
    std::map<TileKey, Job> net_q_;
    std::set<TileKey> busy_;
    std::map<TileKey, Failure> failed_;
    std::vector<TileImage> ready_;
    uint64_t latest_priority_ = 0;

    Clock::time_point next_request_at_{};
    Clock::time_point paused_until_{};
    int consecutive_failures_ = 0;
    uint64_t downloaded_ = 0;
    uint64_t failed_count_ = 0;
    std::string last_error_;

    std::shared_ptr<const TileSource> pf_src_;
    std::deque<TileId> pf_q_;
    size_t pf_in_flight_ = 0;
    uint64_t pf_gen_ = 0;
    PrefetchStatus pf_;

    bool scan_requested_ = false;
    bool clear_requested_ = false;
    CacheStats cache_;

    std::thread loader_;
    std::vector<std::thread> downloaders_;
};

}  // namespace basestation::tiles
