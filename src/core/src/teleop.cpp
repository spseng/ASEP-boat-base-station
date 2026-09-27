#include <basestation/core/teleop.h>

#include <algorithm>
#include <cmath>

namespace basestation {

namespace {
// Config comes from a settings file; never index past the arrays.
float safe_axis(const GamepadSnapshot& pad, GamepadAxis a) {
    return a < GamepadAxis::Count ? pad.axis(a) : 0.0f;
}
bool safe_button(const GamepadSnapshot& pad, GamepadButton b) {
    return b < GamepadButton::Count && pad.button(b);
}
}  // namespace

const char* to_string(GamepadAxis a) {
    switch (a) {
    case GamepadAxis::LeftX: return "Left stick X";
    case GamepadAxis::LeftY: return "Left stick Y";
    case GamepadAxis::RightX: return "Right stick X";
    case GamepadAxis::RightY: return "Right stick Y";
    case GamepadAxis::LeftTrigger: return "Left trigger";
    case GamepadAxis::RightTrigger: return "Right trigger";
    case GamepadAxis::Count: break;
    }
    return "?";
}

const char* to_string(GamepadButton b) {
    switch (b) {
    case GamepadButton::South: return "A";
    case GamepadButton::East: return "B";
    case GamepadButton::West: return "X";
    case GamepadButton::North: return "Y";
    case GamepadButton::LeftShoulder: return "LB";
    case GamepadButton::RightShoulder: return "RB";
    case GamepadButton::Back: return "View";
    case GamepadButton::Start: return "Menu";
    case GamepadButton::Guide: return "Xbox";
    case GamepadButton::LeftStick: return "LS";
    case GamepadButton::RightStick: return "RS";
    case GamepadButton::DpadUp: return "D-pad up";
    case GamepadButton::DpadDown: return "D-pad down";
    case GamepadButton::DpadLeft: return "D-pad left";
    case GamepadButton::DpadRight: return "D-pad right";
    case GamepadButton::Count: break;
    }
    return "?";
}

float shape_axis(float x, float deadzone, float expo) {
    if (!std::isfinite(x)) return 0.0f;
    x = std::clamp(x, -1.0f, 1.0f);
    const float dz = std::isfinite(deadzone) ? std::clamp(deadzone, 0.0f, 0.99f) : 0.0f;
    const float e = std::isfinite(expo) ? std::clamp(expo, 0.0f, 1.0f) : 0.0f;
    const float a = std::fabs(x);
    if (a <= dz) return 0.0f;
    // Rescale so the output starts at 0 at the deadzone edge and still
    // reaches 1 at full travel.
    const float u = (a - dz) / (1.0f - dz);
    const float out = (1.0f - e) * u + e * u * u * u;
    return x < 0 ? -out : out;
}

TeleopOutput compute_teleop(const GamepadSnapshot& pad, const TeleopConfig& cfg, Clock::time_point now) {
    TeleopOutput out;
    if (!pad.connected) {
        out.reason = "no gamepad";
        return out;
    }
    if (now - pad.t > std::chrono::milliseconds(cfg.input_stale_ms)) {
        out.reason = "input stale";
        return out;
    }
    if (cfg.require_deadman && !safe_button(pad, cfg.deadman)) {
        out.reason = "deadman released";
        return out;
    }
    out.driving = true;
    float lin = shape_axis(safe_axis(pad, cfg.linear_axis), cfg.deadzone, cfg.expo) * cfg.max_linear_mps;
    // Stick right is positive, but turning right is negative omega (REP 103).
    float ang = -shape_axis(safe_axis(pad, cfg.angular_axis), cfg.deadzone, cfg.expo) * cfg.max_angular_radps;
    if (cfg.invert_linear) lin = -lin;
    if (cfg.invert_angular) ang = -ang;
    // Avoid -0 showing up in the UI.
    out.linear_mps = lin + 0.0f;
    out.angular_radps = ang + 0.0f;
    return out;
}

}  // namespace basestation
