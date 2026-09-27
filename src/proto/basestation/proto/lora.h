#pragma once

// PROPOSED replacement for ASEP-boat shared/wirelink/include/wirelink/msg/lora.h
//
// This file is the base station's proposal for the over-the-air message set.
// It lives here only because this repository does not modify ASEP-boat; see
// WIRELINK_CHANGES.md. To adopt it upstream: copy this file over
// wirelink/msg/lora.h, change the namespace to wirelink::msg::lora, and once
// wire.h can encode int16_t and nested structs, drop basestation::codec.
//
// Every message travels inside a wirelink frame (COBS + type|seq|len|payload|
// CRC16), the same container used on every other link. All fields use the
// scales defined in boat_defs/units.h. Encoded sizes are well under
// MAX_LORA_PAYLOAD (checked in tests/test_codec.cpp).

#include <wirelink/framing.h>
#include <wirelink/msg/common.h>

#include <cstddef>
#include <cstdint>

namespace basestation::lora {

    constexpr size_t MAX_LORA_PAYLOAD = 64;

    enum class MsgType : uint8_t {
        Disable    = 0,  // land -> boat   trips the output gate (latched)
        SelfStatus = 1,  // boat -> all    position + scalar, overheard by land
        Command    = 2,  // land -> boat   teleop velocity
        Status     = 3,  // boat -> land   mode / gate / faults / uplink quality
        Reenable   = 4,  // land -> boat   request to clear a tripped gate
        SetMode    = 5,  // land -> boat   mode and armed state
    };

    // Trip the output gate of `rx_id` (boat::ids::BROADCAST_ID = every boat).
    // Parsed by the boat's ESP32 itself, so it works with a hung Pi.
    struct Disable {
        constexpr static MsgType TYPE = MsgType::Disable;
        uint8_t rx_id;

        template <class F>
        void fields(F& f) { f(rx_id); }
    };

    // A boat's own position and authoritative scalar, broadcast every slot.
    // Unchanged from the draft. `self.age_ms` is the age of the position
    // the boat is broadcasting (0 when fresh).
    struct SelfStatus {
        constexpr static MsgType TYPE = MsgType::SelfStatus;
        wirelink::msg::common::PeerEntry self;

        template <class F>
        void fields(F& f) { f(self); }
    };

    // Teleop velocity for `rx_id`, in the (v, omega) form mode_manager muxes.
    // lin_vel: mm/s, +forward. ang_vel: mrad/s, +counter-clockwise (ROS REP 103).
    // The draft's age_ms is dropped: the frame seq already exposes stale or
    // missing commands, and the boat's own command timeout handles silence.
    struct Command {
        constexpr static MsgType TYPE = MsgType::Command;
        uint8_t rx_id;
        int16_t lin_vel;  // mm/s   (boat::units::mps_to_mm)
        int16_t ang_vel;  // mrad/s (boat::units::radps_to_mrad)

        template <class F>
        void fields(F& f) { f(rx_id); f(lin_vel); f(ang_vel); }
    };

    // What the ground station needs to show about a boat. Sent by each boat
    // at a low rate (e.g. 1 Hz).
    struct Status {
        constexpr static MsgType TYPE = MsgType::Status;
        uint8_t tx_id;
        uint8_t mode;          // boat::mode::Mode
        uint8_t armed;         // boat::mode::ArmedState
        uint8_t gate_state;    // boat::mode::GateState
        uint16_t fault_flags;  // boat::mode::fault bits
        float heading_deg;     // degrees true, 0-360; NaN if unknown
        float gs_rssi;         // dBm, last packet this boat heard from land; NaN if none
        float gs_snr;          // dB,  same packet

        template <class F>
        void fields(F& f) {
            f(tx_id); f(mode); f(armed); f(gate_state); f(fault_flags);
            f(heading_deg); f(gs_rssi); f(gs_snr);
        }
    };

    // Ask `rx_id` to clear a tripped gate. The ESP32 passes it up to the Pi,
    // which decides and tells its ESP32 to re-enable.
    struct Reenable {
        constexpr static MsgType TYPE = MsgType::Reenable;
        uint8_t rx_id;

        template <class F>
        void fields(F& f) { f(rx_id); }
    };

    // Request a mode / armed-state change on `rx_id`. Handled by the Pi.
    struct SetMode {
        constexpr static MsgType TYPE = MsgType::SetMode;
        uint8_t rx_id;
        uint8_t mode;   // boat::mode::Mode
        uint8_t armed;  // boat::mode::ArmedState

        template <class F>
        void fields(F& f) { f(rx_id); f(mode); f(armed); }
    };
}  // namespace basestation::lora
