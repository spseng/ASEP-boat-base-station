#pragma once

// Offers teleop Command frames to the Uplink at a fixed rate from its own
// thread, so a stalled or minimised UI cannot freeze the last command in
// place: the UI thread only pushes gamepad snapshots, and a snapshot older
// than TeleopConfig::input_stale_ms produces a zero command.
//
// The base-station ESP32 keeps only the newest Command and transmits it in
// the next land slot, so rate_hz sets how fresh that command is, not how
// many go on air (one per TDMA cycle at most). While Disable / Reenable /
// SetMode are queued the Uplink holds Commands back (counted as `held`).
//
// While enabled: one Command per 1/rate_hz to the current target, zero
// velocity unless compute_teleop() says driving.
// On disable, or on a target change: zero commands to the previous target
// for TeleopConfig::stop_burst TDMA cycles before stopping / switching.

#include <basestation/core/teleop.h>
#include <basestation/core/uplink.h>

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>

namespace basestation {

class TeleopSender {
public:
    struct Status {
        bool enabled = false;
        uint8_t target = 0;
        TeleopOutput last{};
        uint64_t sent = 0;
        uint64_t held = 0;  // not offered: a one-shot command was queued
        uint64_t send_failures = 0;
        Clock::time_point last_sent{};
    };

    explicit TeleopSender(Uplink& uplink, EventLog* events = nullptr);
    ~TeleopSender();  // stops the thread (leaving a zero command if enabled)
    TeleopSender(const TeleopSender&) = delete;
    TeleopSender& operator=(const TeleopSender&) = delete;

    void set_enabled(bool enabled);
    void set_target(uint8_t rx_id);
    void set_config(const TeleopConfig& cfg);
    TeleopConfig config() const;

    // Called from the UI thread every frame.
    void update_input(const GamepadSnapshot& pad);

    Status status() const;

private:
    void loop();

    Uplink& uplink_;
    EventLog* events_;

    mutable std::mutex mutex_;
    std::condition_variable cv_;
    TeleopConfig cfg_{};
    GamepadSnapshot pad_{};
    Status status_{};
    Clock::time_point stop_until_{};  // zero commands to stop_target_ until then
    uint8_t stop_target_ = 0;
    bool quit_ = false;
    std::thread thread_;
};

}  // namespace basestation
