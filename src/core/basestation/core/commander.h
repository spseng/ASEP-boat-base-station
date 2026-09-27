#pragma once

// One-shot operator commands (Disable, Reenable, SetMode).
//
// Disable is safety-critical and LoRa is lossy, so it is sent immediately
// and then repeated `disable_repeats - 1` more times, `repeat_interval`
// apart, from a background thread (Disable is idempotent on the boat).
// Reenable and SetMode are sent once: repeating a mode change could
// re-apply it after the operator has already issued another.

#include <basestation/core/link_session.h>

#include <boat_defs/mode.h>

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>

namespace basestation {

class Commander {
public:
    struct Config {
        int disable_repeats = 3;
        std::chrono::milliseconds repeat_interval{100};
    };

    explicit Commander(LinkSession& link, EventLog* events = nullptr);
    ~Commander();
    Commander(const Commander&) = delete;
    Commander& operator=(const Commander&) = delete;

    void set_config(const Config& cfg);
    Config config() const;

    // Each returns false if the first transmission failed (link closed).
    bool disable(uint8_t rx_id);
    bool reenable(uint8_t rx_id);
    bool set_mode(uint8_t rx_id, boat::mode::Mode mode, boat::mode::ArmedState armed);

    // "boat 3" or "ALL".
    static std::string target_name(uint8_t rx_id);

private:
    struct Pending {
        Clock::time_point due;
        uint8_t rx_id;
        int remaining;
    };
    void loop();

    LinkSession& link_;
    EventLog* events_;
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    Config cfg_{};
    std::vector<Pending> pending_;
    bool quit_ = false;
    std::thread thread_;
};

}  // namespace basestation
