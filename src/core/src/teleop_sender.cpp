#include <basestation/core/teleop_sender.h>

#include <basestation/core/commander.h>
#include <basestation/proto/msgs.h>

#include <boat_defs/ids.h>
#include <boat_defs/lora.h>
#include <boat_defs/units.h>

#include <algorithm>
#include <cmath>
#include <string>

namespace basestation {

namespace {

// Keeps a mistyped rate from spinning the thread or starving the boat.
constexpr float MIN_RATE_HZ = 1.0f;
constexpr float MAX_RATE_HZ = 50.0f;

// One TDMA cycle: every id (boats and land) transmits once.
constexpr std::chrono::microseconds CYCLE{boat::ids::NUM_IDS * boat::lora::config::TRANSMIT_INTERVAL};

Clock::time_point stop_deadline(const TeleopConfig& cfg, Clock::time_point now) {
    return now + std::max(0, cfg.stop_burst) * CYCLE;
}

Clock::duration period_of(const TeleopConfig& cfg) {
    const float hz = std::isfinite(cfg.rate_hz) ? std::clamp(cfg.rate_hz, MIN_RATE_HZ, MAX_RATE_HZ)
                                                : TeleopConfig{}.rate_hz;
    return std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1.0 / hz));
}

}  // namespace

TeleopSender::TeleopSender(Uplink& uplink, EventLog* events) : uplink_(uplink), events_(events) {
    thread_ = std::thread([this] { loop(); });
}

TeleopSender::~TeleopSender() {
    bool stop = false;
    uint8_t target = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (status_.enabled) {
            stop = true;
            target = status_.target;
        } else if (Clock::now() < stop_until_) {
            stop = true;
            target = stop_target_;
        }
        status_.enabled = false;
        quit_ = true;
    }
    cv_.notify_all();
    if (thread_.joinable()) thread_.join();
    // The app is exiting: no time for a paced stop. The ESP32 keeps this
    // zero command until the next land slot.
    if (stop) uplink_.offer(lora::Command{target, 0, 0});
}

void TeleopSender::set_enabled(bool enabled) {
    uint8_t target = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (status_.enabled == enabled) return;
        status_.enabled = enabled;
        target = status_.target;
        if (!enabled) {
            stop_until_ = stop_deadline(cfg_, Clock::now());
            stop_target_ = target;
        }
        status_.last = TeleopOutput{};
    }
    cv_.notify_all();
    if (events_) {
        if (enabled)
            events_->info("Teleop enabled, target " + Commander::target_name(target));
        else
            events_->info("Teleop disabled, stopping " + Commander::target_name(target));
    }
}

void TeleopSender::set_target(uint8_t rx_id) {
    bool changed = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (status_.target != rx_id) {
            changed = true;
            // The old target must not keep its last non-zero command. A
            // stop still owed to an even older target is superseded; that
            // boat's own command timeout covers it.
            if (status_.enabled) {
                stop_until_ = stop_deadline(cfg_, Clock::now());
                stop_target_ = status_.target;
            }
            status_.target = rx_id;
        }
    }
    if (!changed) return;
    cv_.notify_all();
    if (events_) events_->info("Teleop target: " + Commander::target_name(rx_id));
}

void TeleopSender::set_config(const TeleopConfig& cfg) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        cfg_ = cfg;
    }
    cv_.notify_all();
}

TeleopConfig TeleopSender::config() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return cfg_;
}

void TeleopSender::update_input(const GamepadSnapshot& pad) {
    // No notify: the send rate is fixed, the next tick picks this up.
    std::lock_guard<std::mutex> lock(mutex_);
    pad_ = pad;
}

TeleopSender::Status TeleopSender::status() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return status_;
}

void TeleopSender::loop() {
    std::unique_lock<std::mutex> lock(mutex_);
    auto next = Clock::now();
    bool warned = false;  // one send-failure warning per enable period
    bool was_enabled = false;

    while (!quit_) {
        // Sleep until the next tick, but wake early when the operator
        // changes something so a stop burst starts without delay.
        const bool en0 = status_.enabled;
        const uint8_t target0 = status_.target;
        const auto stop0 = stop_until_;
        cv_.wait_until(lock, next, [&] {
            return quit_ || status_.enabled != en0 || status_.target != target0 || stop_until_ != stop0;
        });
        if (quit_) break;

        // Fixed schedule rather than now + period: wake-up latency (large
        // with macOS timer coalescing) must not stretch every period. After
        // a stall, skip missed ticks instead of sending a burst.
        const auto now = Clock::now();
        const auto period = period_of(cfg_);
        if (now < next || now >= next + period)
            next = now + period;  // woken early by a change, or stalled
        else
            next += period;
        if (status_.enabled && !was_enabled) warned = false;
        was_enabled = status_.enabled;

        lora::Command cmd{};
        bool is_stop = false;
        if (now < stop_until_) {
            cmd = lora::Command{stop_target_, 0, 0};
            is_stop = true;
        } else if (status_.enabled) {
            const TeleopOutput out = compute_teleop(pad_, cfg_, now);
            status_.last = out;
            cmd = lora::Command{status_.target, boat::units::mps_to_mm(out.linear_mps),
                                boat::units::radps_to_mrad(out.angular_radps)};
        } else {
            continue;
        }

        // Never hold the mutex across a serial write.
        lock.unlock();
        const Uplink::Offer r = uplink_.offer(cmd);
        lock.lock();

        if (r == Uplink::Offer::Sent) {
            ++status_.sent;
            status_.last_sent = now;
        } else if (r == Uplink::Offer::Held) {
            ++status_.held;
        } else {
            ++status_.send_failures;
            if (!warned && events_) {
                warned = true;
                events_->warn(std::string("Teleop ") + (is_stop ? "stop" : "command") +
                              " to " + Commander::target_name(cmd.rx_id) + " not sent (link closed?)");
            }
        }
    }
}

}  // namespace basestation
