#include <basestation/core/link_session.h>

#include <utility>

namespace basestation {

namespace {

constexpr int READ_TIMEOUT_MS = 50;    // bounds how long close() waits for the reader
constexpr int WRITE_TIMEOUT_MS = 200;  // a frame is tiny; longer means the port is stuck
constexpr size_t READ_CHUNK = 4096;

}  // namespace

LinkSession::LinkSession(EventLog* events, FrameLogWriter* log) : events_(events), log_(log) {}

LinkSession::~LinkSession() { close(); }

bool LinkSession::open(const std::string& path, uint32_t baud, std::string* error) {
    close();

    std::lock_guard<std::mutex> lock(port_mutex_);
    std::string err;
    if (!port_.open(path, baud, &err)) {
        if (error) *error = err;
        return false;
    }
    path_ = path;
    baud_ = baud;
    {
        std::lock_guard<std::mutex> stats_lock(stats_mutex_);
        stats_ = Stats{};
        last_error_.clear();
    }
    // tx_seq_ is deliberately not reset: receivers that track seq per type
    // would otherwise see it jump backwards after a reconnect.

    running_ = true;
    reader_ = std::thread(&LinkSession::reader_loop, this);
    if (events_) events_->info("Connected to " + path + " at " + std::to_string(baud) + " baud");
    return true;
}

void LinkSession::close() {
    running_ = false;

    std::thread reader;
    {
        std::lock_guard<std::mutex> lock(port_mutex_);
        reader = std::move(reader_);
    }
    // Join without holding port_mutex_: the reader takes it when it shuts
    // itself down after a port error. The reader never calls close(), but if
    // it ever did it must not join itself.
    if (reader.joinable()) {
        if (reader.get_id() == std::this_thread::get_id()) {
            reader.detach();
        } else {
            reader.join();
        }
    }

    bool was_open = false;
    std::string path;
    {
        std::lock_guard<std::mutex> lock(port_mutex_);
        was_open = port_.is_open();
        port_.close();
        path = path_;
    }
    if (was_open && events_) events_->info("Disconnected from " + path);
}

bool LinkSession::is_open() const {
    std::lock_guard<std::mutex> lock(port_mutex_);
    return port_.is_open();
}

std::string LinkSession::path() const {
    std::lock_guard<std::mutex> lock(port_mutex_);
    return path_;
}

uint32_t LinkSession::baud() const {
    std::lock_guard<std::mutex> lock(port_mutex_);
    return baud_;
}

std::string LinkSession::last_error() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    return last_error_;
}

size_t LinkSession::poll(std::vector<RxFrame>& out) {
    std::lock_guard<std::mutex> lock(rx_mutex_);
    const size_t n = rx_queue_.size();
    out.insert(out.end(), rx_queue_.begin(), rx_queue_.end());
    rx_queue_.clear();
    return n;
}

bool LinkSession::send(const wirelink::Frame& frame) {
    std::lock_guard<std::mutex> lock(port_mutex_);
    if (!port_.is_open()) return false;

    const uint8_t seq = tx_seq_[frame.type]++;
    const auto packet = wirelink::framing::wrap(frame, seq);
    std::string err;
    const bool ok = packet && port_.write_all(packet->data.data(), packet->len, WRITE_TIMEOUT_MS, &err);
    {
        std::lock_guard<std::mutex> stats_lock(stats_mutex_);
        if (ok) {
            ++stats_.tx_frames;
            stats_.tx_bytes += packet->len;
        } else {
            ++stats_.tx_errors;
        }
    }
    if (ok && log_) {
        wirelink::Frame sent = frame;
        sent.seq = seq;
        log_->write(Direction::Tx, sent);
    }
    return ok;
}

LinkSession::Stats LinkSession::stats() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    return stats_;
}

void LinkSession::reader_loop() {
    // One splitter per connection: a new connection starts unsynchronised.
    FrameSplitter splitter;
    FrameSplitter::Stats seen{};
    std::vector<uint8_t> buf(READ_CHUNK);
    std::vector<wirelink::Frame> frames;

    // Only this thread and (after joining it) close() touch the fd, so
    // reading without port_mutex_ is safe and does not block send().
    while (running_) {
        std::string err;
        const long n = port_.read(buf.data(), buf.size(), READ_TIMEOUT_MS, &err);
        if (n < 0) {
            {
                std::lock_guard<std::mutex> lock(port_mutex_);
                port_.close();
            }
            // A failure racing a user close() is not worth reporting.
            if (running_.exchange(false)) {
                {
                    std::lock_guard<std::mutex> lock(stats_mutex_);
                    last_error_ = err;
                }
                if (events_) events_->error("Serial link lost: " + err);
            }
            return;
        }
        if (n == 0) continue;

        frames.clear();
        splitter.feed(buf.data(), static_cast<size_t>(n), frames);
        const Clock::time_point now = Clock::now();

        if (log_) {
            for (const auto& f : frames) log_->write(Direction::Rx, f);
        }
        if (!frames.empty()) {
            std::lock_guard<std::mutex> lock(rx_mutex_);
            for (const auto& f : frames) rx_queue_.push_back(RxFrame{now, f});
        }

        const FrameSplitter::Stats& s = splitter.stats();
        std::lock_guard<std::mutex> lock(stats_mutex_);
        stats_.rx_bytes += static_cast<uint64_t>(n);
        stats_.rx_frames += s.frames_ok - seen.frames_ok;
        stats_.rx_bad += s.frames_bad - seen.frames_bad;
        stats_.rx_overflows += s.overflows - seen.overflows;
        seen = s;
    }
}

}  // namespace basestation
