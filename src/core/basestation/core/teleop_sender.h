#pragma once

// Sends teleop Command frames at a fixed rate from its own thread, so a
// stalled or minimised UI cannot freeze the last command in place: the UI
// thread only pushes gamepad snapshots, and a snapshot older than
// TeleopConfig::input_stale_ms produces a zero command.
//
// While enabled: one Command per 1/rate_hz to the current target, zero
// velocity unless compute_teleop() says driving.
// On disable, or on a target change: TeleopConfig::stop_burst zero commands
// to the previous target before stopping / switching.

#include <basestation/core/link_session.h>
#include <basestation/core/teleop.h>

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
        uint64_t send_failures = 0;
        Clock::time_point last_sent{};
    };

    explicit TeleopSender(LinkSession& link, EventLog* events = nullptr);
    ~TeleopSender();  // stops the thread (sending the stop burst if enabled)
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

    LinkSession& link_;
    EventLog* events_;

    mutable std::mutex mutex_;
    std::condition_variable cv_;
    TeleopConfig cfg_{};
    GamepadSnapshot pad_{};
    Status status_{};
    int pending_stop_ = 0;         // zero commands still owed
    uint8_t pending_stop_target_ = 0;
    bool quit_ = false;
    std::thread thread_;
};

}  // namespace basestation
