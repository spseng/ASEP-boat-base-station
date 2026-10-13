#pragma once

// Helpers for the upstream LoRa message set (wirelink/msg/lora.h).
//
// The LoRa messages themselves are upstream's; `basestation::lora` is only a
// short alias for wirelink::msg::lora. What this file adds is the packing of
// SelfStatus::state, one byte that carries three boat_defs/mode.h enums:
//
//   bits 0-3  mode   boat::mode::Mode (raw value, 0-15)
//   bit  4    armed  boat::mode::ArmedState (1 = ARMED)
//   bit  5    gate   boat::mode::GateState  (1 = ENABLED)
//   bits 6-7  reserved, sent as 0
//
// decode() keeps the raw values, so a mode this build does not know still
// round-trips and is shown by names::mode_name()'s fallback.

#include <boat_defs/mode.h>

#include <wirelink/msg/lora.h>

#include <cstdint>

namespace basestation {

namespace lora = wirelink::msg::lora;

namespace state {

constexpr uint8_t MODE_MASK = 0x0F;
constexpr uint8_t ARMED_BIT = 1u << 4;
constexpr uint8_t GATE_BIT = 1u << 5;
constexpr uint8_t RESERVED_MASK = 0xC0;

// Highest mode value the state byte can carry.
constexpr uint8_t MAX_MODE = MODE_MASK;

struct State {
    uint8_t mode = 0;      // boat::mode::Mode raw value
    uint8_t armed = 0;     // boat::mode::ArmedState raw value
    uint8_t gate = 0;      // boat::mode::GateState raw value
    uint8_t reserved = 0;  // bits 6-7 as received (non-zero = newer firmware?)

    constexpr bool is_armed() const { return armed == static_cast<uint8_t>(boat::mode::ArmedState::ARMED); }
    constexpr bool is_tripped() const { return gate == static_cast<uint8_t>(boat::mode::GateState::TRIPPED); }
};

constexpr State decode(uint8_t s) {
    State out{};
    out.mode = static_cast<uint8_t>(s & MODE_MASK);
    out.armed = (s & ARMED_BIT) ? static_cast<uint8_t>(boat::mode::ArmedState::ARMED)
                                : static_cast<uint8_t>(boat::mode::ArmedState::DISARMED);
    out.gate = (s & GATE_BIT) ? static_cast<uint8_t>(boat::mode::GateState::ENABLED)
                              : static_cast<uint8_t>(boat::mode::GateState::TRIPPED);
    out.reserved = static_cast<uint8_t>(s & RESERVED_MASK);
    return out;
}

// Mode values above MAX_MODE do not fit and are truncated to 4 bits.
constexpr uint8_t encode(uint8_t mode, boat::mode::ArmedState armed, boat::mode::GateState gate) {
    uint8_t s = static_cast<uint8_t>(mode & MODE_MASK);
    if (armed == boat::mode::ArmedState::ARMED) s = static_cast<uint8_t>(s | ARMED_BIT);
    if (gate == boat::mode::GateState::ENABLED) s = static_cast<uint8_t>(s | GATE_BIT);
    return s;
}

constexpr uint8_t encode(boat::mode::Mode mode, boat::mode::ArmedState armed, boat::mode::GateState gate) {
    return encode(static_cast<uint8_t>(mode), armed, gate);
}

}  // namespace state
}  // namespace basestation
