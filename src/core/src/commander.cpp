#include <basestation/core/commander.h>

#include <basestation/proto/lora.h>

#include <boat_defs/ids.h>

#include <algorithm>

namespace basestation {

namespace {

const char* mode_name(boat::mode::Mode m) {
    using boat::mode::Mode;
    switch (m) {
    case Mode::MANUAL: return "MANUAL";
    case Mode::AUTONOMOUS: return "AUTONOMOUS";
    case Mode::RETURN_TO_HOME: return "RETURN_TO_HOME";
    case Mode::EMERGENCY_STOP: return "EMERGENCY_STOP";
    }
    return "MODE(?)";
}

}  // namespace

Commander::Commander(LinkSession& link, EventLog* events) : link_(link), events_(events) {
    thread_ = std::thread([this] { loop(); });
}

Commander::~Commander() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        quit_ = true;
        pending_.clear();
    }
    cv_.notify_all();
    if (thread_.joinable()) thread_.join();
}

void Commander::set_config(const Config& cfg) {
    std::lock_guard<std::mutex> lock(mutex_);
    cfg_ = cfg;
}

Commander::Config Commander::config() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return cfg_;
}

std::string Commander::target_name(uint8_t rx_id) {
    if (rx_id == boat::ids::BROADCAST_ID) return "ALL";
    return "boat " + std::to_string(rx_id);
}

bool Commander::disable(uint8_t rx_id) {
    // First copy goes out on the caller's thread: no scheduling latency.
    const bool ok = link_.send_msg(lora::Disable{rx_id});
    if (events_) {
        if (ok)
            events_->warn("Sent DISABLE to " + target_name(rx_id));
        else
            events_->error("Failed to send DISABLE to " + target_name(rx_id));
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const int remaining = cfg_.disable_repeats - 1;
        // Repeats are scheduled even if the first send failed: the link may
        // come back within the repeat window, and Disable is idempotent.
        if (remaining > 0 && !quit_) {
            const Pending p{Clock::now() + cfg_.repeat_interval, rx_id, remaining};
            // A second press for the same target restarts its schedule
            // instead of doubling the traffic.
            auto it = std::find_if(pending_.begin(), pending_.end(),
                                   [&](const Pending& q) { return q.rx_id == rx_id; });
            if (it != pending_.end())
                *it = p;
            else
                pending_.push_back(p);
        }
    }
    cv_.notify_all();
    return ok;
}

bool Commander::reenable(uint8_t rx_id) {
    bool ok = false;
    {
        // Queued Disable repeats aimed at this target must not land after
        // the Reenable and trip the gate again. Reenable(ALL) drops every
        // repeat; reenabling one boat leaves a repeating Disable(ALL) alone,
        // because the operator still wants every other boat stopped.
        // Sending under the lock orders this against the repeat thread,
        // which also sends under it.
        std::lock_guard<std::mutex> lock(mutex_);
        pending_.erase(std::remove_if(pending_.begin(), pending_.end(),
                                      [&](const Pending& p) {
                                          return rx_id == boat::ids::BROADCAST_ID || p.rx_id == rx_id;
                                      }),
                       pending_.end());
        ok = link_.send_msg(lora::Reenable{rx_id});
    }
    if (events_) {
        if (ok)
            events_->info("Sent REENABLE to " + target_name(rx_id));
        else
            events_->error("Failed to send REENABLE to " + target_name(rx_id));
    }
    return ok;
}

bool Commander::set_mode(uint8_t rx_id, boat::mode::Mode mode, boat::mode::ArmedState armed) {
    const lora::SetMode msg{rx_id, static_cast<uint8_t>(mode), static_cast<uint8_t>(armed)};
    const bool ok = link_.send_msg(msg);
    if (events_) {
        const std::string what = std::string("SET_MODE ") + mode_name(mode) +
                                 (armed == boat::mode::ArmedState::ARMED ? " ARMED" : " DISARMED");
        if (ok)
            events_->info("Sent " + what + " to " + target_name(rx_id));
        else
            events_->error("Failed to send " + what + " to " + target_name(rx_id));
    }
    return ok;
}

void Commander::loop() {
    std::unique_lock<std::mutex> lock(mutex_);
    while (!quit_) {
        if (pending_.empty()) {
            cv_.wait(lock, [this] { return quit_ || !pending_.empty(); });
            continue;
        }
        auto next = std::min_element(pending_.begin(), pending_.end(),
                                     [](const Pending& a, const Pending& b) { return a.due < b.due; });
        const auto due = next->due;
        if (Clock::now() < due) {
            // Woken early by a new request or quit; re-evaluate either way.
            cv_.wait_until(lock, due);
            continue;
        }
        const uint8_t rx_id = next->rx_id;
        if (--next->remaining <= 0)
            pending_.erase(next);
        else
            next->due = due + cfg_.repeat_interval;

        // Sent under the lock so reenable() can cancel repeats without one
        // already in flight slipping out after its Reenable. A frame write
        // is short; disable() itself sends its first copy unlocked.
        link_.send_msg(lora::Disable{rx_id});
    }
}

}  // namespace basestation
