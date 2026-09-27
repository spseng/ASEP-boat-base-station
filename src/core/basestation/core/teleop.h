#pragma once

// Gamepad -> (v, omega) mapping. Pure functions, no SDL, so it is unit
// tested. The SDL layer fills a GamepadSnapshot every UI frame.

#include <basestation/core/common.h>

#include <array>
#include <cstdint>
#include <string>

namespace basestation {

enum class GamepadAxis : uint8_t { LeftX, LeftY, RightX, RightY, LeftTrigger, RightTrigger, Count };

enum class GamepadButton : uint8_t {
    South, East, West, North,  // Xbox: A, B, X, Y
    LeftShoulder, RightShoulder,
    Back, Start, Guide,
    LeftStick, RightStick,
    DpadUp, DpadDown, DpadLeft, DpadRight,
    Count
};

const char* to_string(GamepadAxis a);
const char* to_string(GamepadButton b);  // Xbox names: "A", "B", "RB", ...

struct GamepadSnapshot {
    bool connected = false;
    std::string name;
    // Sticks in [-1, 1], normalised so that stick UP and stick RIGHT are
    // positive. Triggers in [0, 1].
    std::array<float, static_cast<size_t>(GamepadAxis::Count)> axes{};
    std::array<bool, static_cast<size_t>(GamepadButton::Count)> buttons{};
    Clock::time_point t{};  // when this snapshot was taken

    float axis(GamepadAxis a) const { return axes[static_cast<size_t>(a)]; }
    bool button(GamepadButton b) const { return buttons[static_cast<size_t>(b)]; }
};

struct TeleopConfig {
    GamepadAxis linear_axis = GamepadAxis::LeftY;    // up = forward
    GamepadAxis angular_axis = GamepadAxis::RightX;  // right = turn right (negative omega)
    bool invert_linear = false;
    bool invert_angular = false;

    float deadzone = 0.12f;       // fraction of full stick travel
    float expo = 0.3f;            // 0 = linear, 1 = cubic
    float max_linear_mps = 1.0f;
    float max_angular_radps = 1.0f;

    bool require_deadman = true;
    GamepadButton deadman = GamepadButton::RightShoulder;  // RB
    GamepadButton disable_button = GamepadButton::East;    // B: Disable current target

    float rate_hz = 10.0f;     // command send rate
    int input_stale_ms = 250;  // snapshot older than this -> zero command
    int stop_burst = 3;        // zero commands sent when teleop is switched off
};

struct TeleopOutput {
    float linear_mps = 0;
    float angular_radps = 0;
    bool driving = false;        // true only when a non-neutral command is allowed
    const char* reason = "";     // why driving is false: "no gamepad", "deadman released", "input stale"
};

// Deadzone with rescaling (no jump at the edge) followed by expo:
//   out = sign(x) * ((1 - expo) * u + expo * u^3), u = rescaled |x| after deadzone.
float shape_axis(float x, float deadzone, float expo);

// Maps a snapshot to a velocity command. Zero with a reason when the gamepad
// is disconnected, the snapshot is stale, or the deadman is not held.
TeleopOutput compute_teleop(const GamepadSnapshot& pad, const TeleopConfig& cfg, Clock::time_point now);

}  // namespace basestation
