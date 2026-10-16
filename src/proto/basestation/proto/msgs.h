#pragma once

// The message definitions this app uses, all from upstream (ASEP-boat
// shared/wirelink): LoRa messages (wirelink/msg/lora.h) and the messages
// local to the PC <-> base-station ESP32 link (wirelink/msg/base.h).
// Nothing here defines a message; this header only adds short aliases and a
// typed view of SelfStatus::state on top of upstream's get_*/make_state().

#include <boat_defs/mode.h>

#include <wirelink/msg/base.h>
#include <wirelink/msg/lora.h>

#include <cstdint>

namespace basestation {

namespace lora = wirelink::msg::lora;
namespace base = wirelink::msg::base;

namespace state {

// Highest mode value the state byte can carry (4 bits).
constexpr uint8_t MAX_MODE = 0x0F;
// Bits 6-7 are reserved upstream; non-zero means newer firmware.
constexpr uint8_t RESERVED_MASK = 0xC0;

struct State {
    uint8_t mode = 0;      // boat::mode::Mode raw value
    uint8_t armed = 0;     // boat::mode::ArmedState raw value
    uint8_t gate = 0;      // boat::mode::GateState raw value
    uint8_t reserved = 0;  // bits 6-7 as received

    constexpr bool is_armed() const { return armed == static_cast<uint8_t>(boat::mode::ArmedState::ARMED); }
    constexpr bool is_tripped() const { return gate == static_cast<uint8_t>(boat::mode::GateState::TRIPPED); }
};

// Raw values are kept, so a mode this build does not know still round-trips
// and is shown by names::mode_name()'s fallback.
constexpr State decode(uint8_t s) {
    State out{};
    out.mode = lora::get_mode(s);
    out.armed = lora::get_armed(s);
    out.gate = lora::get_gate(s);
    out.reserved = static_cast<uint8_t>(s & RESERVED_MASK);
    return out;
}

// Mode values above MAX_MODE do not fit and are truncated to 4 bits.
constexpr uint8_t encode(uint8_t mode, boat::mode::ArmedState armed, boat::mode::GateState gate) {
    return lora::make_state(mode, static_cast<uint8_t>(armed), static_cast<uint8_t>(gate));
}

constexpr uint8_t encode(boat::mode::Mode mode, boat::mode::ArmedState armed, boat::mode::GateState gate) {
    return encode(static_cast<uint8_t>(mode), armed, gate);
}

}  // namespace state
}  // namespace basestation
