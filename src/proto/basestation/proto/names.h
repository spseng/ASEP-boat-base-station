#pragma once

// Display names for the enums in boat_defs/mode.h, in ONE place.
//
// Modes are plain uint8 values on the wire, so a boat running newer firmware
// may report a mode this table does not know yet. Every lookup therefore
// takes the raw value and falls back to "MODE <n>" instead of failing, and
// the UI can send any raw value (see SetMode). To add a mode: add it to
// boat_defs/mode.h upstream, then add one line to MODES below.

#include <boat_defs/mode.h>

#include <cstdint>
#include <string>
#include <vector>

namespace basestation::names {

struct ModeInfo {
    boat::mode::Mode mode;
    const char* name;         // short, upper case, as shown in tables
    const char* description;  // tooltip
};

inline constexpr ModeInfo MODES[] = {
    {boat::mode::Mode::MANUAL, "MANUAL", "Teleop: the boat follows Command frames from the base station"},
    {boat::mode::Mode::AUTONOMOUS, "AUTONOMOUS", "Swarm algorithm drives the boat"},
    {boat::mode::Mode::RETURN_TO_HOME, "RETURN_TO_HOME", "Boat navigates back to its home position"},
    {boat::mode::Mode::EMERGENCY_STOP, "EMERGENCY_STOP", "Boat holds thrusters at neutral"},
};

inline const ModeInfo* find_mode(uint8_t raw) {
    for (const auto& m : MODES)
        if (static_cast<uint8_t>(m.mode) == raw) return &m;
    return nullptr;
}

inline std::string mode_name(uint8_t raw) {
    if (const ModeInfo* m = find_mode(raw)) return m->name;
    return "MODE " + std::to_string(raw);
}

inline std::string armed_name(uint8_t raw) {
    switch (static_cast<boat::mode::ArmedState>(raw)) {
    case boat::mode::ArmedState::DISARMED: return "DISARMED";
    case boat::mode::ArmedState::ARMED: return "ARMED";
    }
    return "ARMED? " + std::to_string(raw);
}

inline std::string gate_name(uint8_t raw) {
    switch (static_cast<boat::mode::GateState>(raw)) {
    case boat::mode::GateState::TRIPPED: return "TRIPPED";
    case boat::mode::GateState::ENABLED: return "ENABLED";
    }
    return "GATE? " + std::to_string(raw);
}

struct FaultInfo {
    uint16_t bit;
    const char* name;
};

inline constexpr FaultInfo FAULTS[] = {
    {boat::mode::fault::GPS_FAILURE, "GPS_FAILURE"},
    {boat::mode::fault::COMMUNICATION_FAILURE, "COMMUNICATION_FAILURE"},
    {boat::mode::fault::MOTOR_FAILURE, "MOTOR_FAILURE"},
    {boat::mode::fault::BATTERY_LOW, "BATTERY_LOW"},
    {boat::mode::fault::SENSOR_FAILURE, "SENSOR_FAILURE"},
};

// Names of the set bits; unknown bits appear as "BIT<n>".
inline std::vector<std::string> fault_names(uint16_t flags) {
    std::vector<std::string> out;
    for (int bit = 0; bit < 16; ++bit) {
        const uint16_t mask = static_cast<uint16_t>(1u << bit);
        if (!(flags & mask)) continue;
        const char* name = nullptr;
        for (const auto& f : FAULTS)
            if (f.bit == mask) name = f.name;
        out.push_back(name ? name : "BIT" + std::to_string(bit));
    }
    return out;
}

// "GPS_FAILURE, MOTOR_FAILURE" or "" when no bits are set.
inline std::string fault_list(uint16_t flags) {
    std::string s;
    for (const auto& n : fault_names(flags)) {
        if (!s.empty()) s += ", ";
        s += n;
    }
    return s;
}

}  // namespace basestation::names
