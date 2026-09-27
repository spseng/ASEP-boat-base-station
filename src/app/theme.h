#pragma once

// Visual style: dark theme, fonts, and the colour vocabulary shared by every
// panel (red = danger / tripped / disabled, green = ok / enabled,
// amber = warning / stale).

#include <imgui.h>

#include <cstdint>

namespace basestation::app {

namespace colors {
inline const ImVec4 danger{0.94f, 0.33f, 0.31f, 1.0f};
inline const ImVec4 danger_bg{0.70f, 0.13f, 0.13f, 1.0f};
inline const ImVec4 ok{0.40f, 0.82f, 0.45f, 1.0f};
inline const ImVec4 warn{1.00f, 0.72f, 0.20f, 1.0f};
inline const ImVec4 muted{0.55f, 0.58f, 0.62f, 1.0f};
inline const ImVec4 accent{0.35f, 0.62f, 0.95f, 1.0f};
}  // namespace colors

struct Fonts {
    ImFont* regular = nullptr;
    ImFont* bold = nullptr;  // same as regular when no bold face was found
    ImFont* mono = nullptr;  // same as regular when no monospace face was found
};

// Loads a system UI font when one is available (falls back to Dear ImGui's
// embedded scalable font) and sets the base size.
Fonts load_fonts(float base_size_px);

// Dark theme with rounded corners; sizes scaled by `ui_scale`.
void apply_theme(float ui_scale);

// Stable, distinct colour per boat id (avoids pure red / green, which carry
// meaning elsewhere).
ImVec4 boat_color(uint8_t id);
ImU32 boat_color_u32(uint8_t id, float alpha = 1.0f);

}  // namespace basestation::app
