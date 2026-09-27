#pragma once

// Owns the serial connection to the base-station ESP32.
//
// A background thread reads the port, splits frames and queues them with an
// arrival timestamp; the UI thread drains them with poll(). send() may be
// called from any thread. If the port fails (USB unplugged) the session
// closes itself, records the error and logs an event.

#include <basestation/core/common.h>
#include <basestation/core/event_log.h>
#include <basestation/core/frame_log.h>
#include <basestation/core/frame_splitter.h>
#include <basestation/core/serial_port.h>
#include <basestation/proto/codec.h>

#include <array>
#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace basestation {

class LinkSession {
public:
    struct Stats {
        uint64_t rx_bytes = 0;
        uint64_t rx_frames = 0;
        uint64_t rx_bad = 0;        // corrupt frames (COBS / length / CRC)
        uint64_t rx_overflows = 0;  // runs too long to be a frame
        uint64_t tx_frames = 0;
        uint64_t tx_bytes = 0;
        uint64_t tx_errors = 0;
    };

    // `events` and `log` may be null; they must outlive the session.
    explicit LinkSession(EventLog* events = nullptr, FrameLogWriter* log = nullptr);
    ~LinkSession();
    LinkSession(const LinkSession&) = delete;
    LinkSession& operator=(const LinkSession&) = delete;

    bool open(const std::string& path, uint32_t baud, std::string* error);
    void close();
    bool is_open() const;
    std::string path() const;
    uint32_t baud() const;
    // Why the session last closed on its own ("" if it has not).
    std::string last_error() const;

    // Moves every frame received since the last call into `out` (appends).
    // Returns the number of frames appended.
    size_t poll(std::vector<RxFrame>& out);

    // Sends a frame, assigning the next sequence number for its type.
    // Thread-safe. False if closed or the write failed.
    bool send(const wirelink::Frame& frame);

    template <class T>
    bool send_msg(const T& msg) {
        wirelink::Frame f{};
        if (!codec::pack(msg, f)) return false;
        return send(f);
    }

    Stats stats() const;

private:
    void reader_loop();

    EventLog* events_;
    FrameLogWriter* log_;

    SerialPort port_;
    mutable std::mutex port_mutex_;   // guards open/close and writes
    mutable std::mutex rx_mutex_;     // guards rx_queue_
    mutable std::mutex stats_mutex_;  // guards stats_ and last_error_
    std::vector<RxFrame> rx_queue_;
    std::array<uint8_t, 256> tx_seq_{};
    Stats stats_{};
    std::string last_error_;
    std::string path_;
    uint32_t baud_ = 0;

    std::thread reader_;
    std::atomic<bool> running_{false};
};

}  // namespace basestation
