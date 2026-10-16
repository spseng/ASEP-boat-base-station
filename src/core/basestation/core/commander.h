#pragma once

// One-shot operator commands (Disable, Reenable, SetMode), queued on the
// Uplink, which hands them to the base-station ESP32 one land slot at a time.
//
// Disable is safety-critical and LoRa is lossy, so it goes out in
// `disable_repeats` land slots (Disable is idempotent on the boat).
// Reenable and SetMode go out once: repeating a mode change could re-apply
// it after the operator has already issued another.

#include <basestation/core/uplink.h>

#include <boat_defs/mode.h>

#include <mutex>
#include <string>

namespace basestation {

class Commander {
public:
    struct Config {
        int disable_repeats = 3;  // land slots carrying each Disable
    };

    explicit Commander(Uplink& uplink, EventLog* events = nullptr);
    Commander(const Commander&) = delete;
    Commander& operator=(const Commander&) = delete;

    void set_config(const Config& cfg);
    Config config() const;

    // Each returns false if the link is closed (the command stays queued
    // briefly in case it comes back, see Uplink::Config::max_age).
    bool disable(uint8_t rx_id);
    bool reenable(uint8_t rx_id);
    bool set_mode(uint8_t rx_id, boat::mode::Mode mode, boat::mode::ArmedState armed);

    // "boat 3" or "ALL".
    static std::string target_name(uint8_t rx_id);

private:
    Uplink& uplink_;
    EventLog* events_;
    mutable std::mutex mutex_;
    Config cfg_{};
};

}  // namespace basestation
