#include <basestation/core/commander.h>

#include <basestation/proto/msgs.h>
#include <basestation/proto/names.h>

#include <boat_defs/ids.h>

#include <algorithm>

namespace basestation {

Commander::Commander(Uplink& uplink, EventLog* events) : uplink_(uplink), events_(events) {}

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
    const std::string what = "DISABLE to " + target_name(rx_id);
    const bool ok = uplink_.push_msg(lora::Disable{rx_id}, std::max(1, config().disable_repeats), what);
    if (events_) {
        if (ok)
            events_->warn("Sent " + what);
        else
            events_->error("Failed to send " + what + " (link closed?)");
    }
    return ok;
}

bool Commander::reenable(uint8_t rx_id) {
    // Disable copies still owed to this target must not go out after the
    // Reenable and trip the gate again. Reenable(ALL) drops every Disable;
    // reenabling one boat leaves a Disable(ALL) alone, because the operator
    // still wants every other boat stopped.
    uplink_.drop_if([&](const wirelink::Frame& f) {
        lora::Disable d{};
        if (f.type != static_cast<uint8_t>(lora::MsgType::Disable) || !wirelink::codec::unpack(f, d)) return false;
        return rx_id == boat::ids::BROADCAST_ID || d.rx_id == rx_id;
    });
    const std::string what = "REENABLE to " + target_name(rx_id);
    const bool ok = uplink_.push_msg(lora::Reenable{rx_id}, 1, what);
    if (events_) {
        if (ok)
            events_->info("Sent " + what);
        else
            events_->error("Failed to send " + what + " (link closed?)");
    }
    return ok;
}

bool Commander::set_mode(uint8_t rx_id, boat::mode::Mode mode, boat::mode::ArmedState armed) {
    const lora::SetMode msg{rx_id, static_cast<uint8_t>(mode), static_cast<uint8_t>(armed)};
    const std::string what = "SET_MODE " + names::mode_name(static_cast<uint8_t>(mode)) + " " +
                             names::armed_name(static_cast<uint8_t>(armed)) + " to " + target_name(rx_id);
    const bool ok = uplink_.push_msg(msg, 1, what);
    if (events_) {
        if (ok)
            events_->info("Sent " + what);
        else
            events_->error("Failed to send " + what + " (link closed?)");
    }
    return ok;
}

}  // namespace basestation
