// Teleop panel: gamepad state, enable switch, target, live output and the
// mapping configuration.

#include "app.h"

#include <boat_defs/ids.h>
#include <boat_defs/mode.h>

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace basestation::app {

namespace {

// iOS-style switch; returns true when clicked.
bool toggle_switch(const char* id, bool on, float height) {
    const float width = height * 1.9f;
    const ImVec2 p = ImGui::GetCursorScreenPos();
    const bool clicked = ImGui::InvisibleButton(id, ImVec2(width, height));
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImU32 bg = ImGui::GetColorU32(on ? ImVec4(0.20f, 0.62f, 0.30f, 1.0f)
                                           : ImGui::IsItemHovered() ? ImVec4(0.32f, 0.35f, 0.40f, 1.0f)
                                                                    : ImVec4(0.26f, 0.28f, 0.32f, 1.0f));
    dl->AddRectFilled(p, ImVec2(p.x + width, p.y + height), bg, height * 0.5f);
    const float r = height * 0.5f - 3.0f;
    const float cx = on ? p.x + width - height * 0.5f : p.x + height * 0.5f;
    dl->AddCircleFilled(ImVec2(cx, p.y + height * 0.5f), r, IM_COL32(240, 240, 240, 255));
    return clicked;
}

// Stick: circle, deadzone ring, guide lines for the axes teleop uses, dot.
void draw_stick(const char* label, float x, float y, float deadzone, bool uses_x, bool uses_y, float radius) {
    const ImVec2 p = ImGui::GetCursorScreenPos();
    const ImVec2 c(p.x + radius, p.y + radius);
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddCircleFilled(c, radius, IM_COL32(30, 33, 38, 255));
    dl->AddCircle(c, radius, IM_COL32(90, 96, 105, 255), 0, 1.5f);
    ImVec4 gc = colors::accent;
    gc.w = 0.55f;
    const ImU32 guide = ImGui::GetColorU32(gc);
    if (uses_x) dl->AddLine(ImVec2(c.x - radius, c.y), ImVec2(c.x + radius, c.y), guide, 2.0f);
    if (uses_y) dl->AddLine(ImVec2(c.x, c.y - radius), ImVec2(c.x, c.y + radius), guide, 2.0f);
    if (deadzone > 0) dl->AddCircle(c, radius * deadzone, IM_COL32(255, 184, 51, 150), 0, 1.0f);
    // Snapshot axes are UP-positive; screen y grows down.
    const ImVec2 dot(c.x + std::clamp(x, -1.0f, 1.0f) * radius, c.y - std::clamp(y, -1.0f, 1.0f) * radius);
    dl->AddLine(c, dot, IM_COL32(230, 230, 230, 120), 1.0f);
    dl->AddCircleFilled(dot, radius * 0.14f, IM_COL32(235, 238, 242, 255));
    const ImVec2 ts = ImGui::CalcTextSize(label);
    dl->AddText(ImVec2(c.x - ts.x / 2, p.y + radius * 2 + 2), ImGui::GetColorU32(ImGuiCol_TextDisabled), label);
    ImGui::Dummy(ImVec2(radius * 2, radius * 2 + ts.y + 4));
}

void draw_trigger(const char* label, float v, float w, float h) {
    const ImVec2 p = ImGui::GetCursorScreenPos();
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddRectFilled(p, ImVec2(p.x + w, p.y + h), IM_COL32(30, 33, 38, 255), 3.0f);
    const float fill = std::clamp(v, 0.0f, 1.0f) * h;
    dl->AddRectFilled(ImVec2(p.x, p.y + h - fill), ImVec2(p.x + w, p.y + h), ImGui::GetColorU32(colors::accent), 3.0f);
    dl->AddRect(p, ImVec2(p.x + w, p.y + h), IM_COL32(90, 96, 105, 255), 3.0f);
    const ImVec2 ts = ImGui::CalcTextSize(label);
    dl->AddText(ImVec2(p.x + (w - ts.x) / 2, p.y + h + 2), ImGui::GetColorU32(ImGuiCol_TextDisabled), label);
    ImGui::Dummy(ImVec2(std::max(w, ts.x), h + ts.y + 4));
}

// Centred horizontal bar for a signed value in [-1, 1].
void signed_bar(float frac, const ImVec4& color, float width, float height) {
    const ImVec2 p = ImGui::GetCursorScreenPos();
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddRectFilled(p, ImVec2(p.x + width, p.y + height), IM_COL32(30, 33, 38, 255), 3.0f);
    const float mid = p.x + width / 2;
    const float end = mid + std::clamp(frac, -1.0f, 1.0f) * width / 2;
    dl->AddRectFilled(ImVec2(std::min(mid, end), p.y), ImVec2(std::max(mid, end), p.y + height),
                      ImGui::GetColorU32(color), 2.0f);
    dl->AddLine(ImVec2(mid, p.y), ImVec2(mid, p.y + height), IM_COL32(120, 125, 135, 255), 1.0f);
    ImGui::Dummy(ImVec2(width, height));
}

template <class E>
bool enum_combo(const char* label, E& value) {
    bool changed = false;
    if (ImGui::BeginCombo(label, to_string(value))) {
        for (int i = 0; i < static_cast<int>(E::Count); ++i) {
            const E e = static_cast<E>(i);
            if (ImGui::Selectable(to_string(e), e == value)) {
                value = e;
                changed = true;
            }
        }
        ImGui::EndCombo();
    }
    return changed;
}

bool is_axis_x(GamepadAxis a) { return a == GamepadAxis::LeftX || a == GamepadAxis::RightX; }
bool is_left(GamepadAxis a) { return a == GamepadAxis::LeftX || a == GamepadAxis::LeftY; }
bool is_right(GamepadAxis a) { return a == GamepadAxis::RightX || a == GamepadAxis::RightY; }

}  // namespace

void App::draw_teleop() {
    if (!settings_.show_teleop) return;
    if (!ImGui::Begin("Teleop", &settings_.show_teleop)) {
        ImGui::End();
        return;
    }
    const float em = ImGui::GetFontSize();
    const TeleopConfig& cfg = settings_.teleop;
    const TeleopSender::Status& st = teleop_status_;

    // ---- enable switch + target ----
    if (toggle_switch("##teleop_on", st.enabled, em * 1.5f)) set_teleop_enabled(!st.enabled);
    ImGui::SameLine();
    ImGui::PushFont(fonts_.bold, 0.0f);
    ImGui::AlignTextToFramePadding();
    ImGui::TextColored(st.enabled ? colors::ok : ImGui::GetStyleColorVec4(ImGuiCol_Text),
                       st.enabled ? "Teleop ON" : "Teleop off");
    ImGui::PopFont();
    ImGui::SameLine();
    ImGui::SetNextItemWidth(-FLT_MIN);
    uint8_t target = teleop_target_;
    if (target_combo("##teleoptarget", target, true)) set_teleop_target(target);
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_ForTooltip))
        ImGui::SetTooltip("Boat to drive. Follows the selected boat while teleop is off.");
    if (teleop_target_ == 0) ImGui::TextColored(colors::warn, "Select a boat or pick a target to enable teleop.");

    // ---- will the boat listen? ----
    auto check_boat = [&](const BoatState& b, std::string& why) {
        if (!b.status) {
            why = "no Status received yet";
            return;
        }
        const auto& s = *b.status;
        std::string w;
        if (s.mode != static_cast<uint8_t>(boat::mode::Mode::MANUAL)) w += names::mode_name(s.mode) + " (needs MANUAL)";
        if (s.armed != static_cast<uint8_t>(boat::mode::ArmedState::ARMED)) w += (w.empty() ? "" : ", ") + names::armed_name(s.armed);
        if (s.gate_state == static_cast<uint8_t>(boat::mode::GateState::TRIPPED)) w += (w.empty() ? "" : ", ") + std::string("gate TRIPPED");
        why = w;
    };
    if (teleop_target_ != 0) {
        std::string warn;
        if (teleop_target_ == boat::ids::BROADCAST_ID) {
            for (const auto& [id, b] : fleet_.boats()) {
                std::string why;
                check_boat(b, why);
                if (!why.empty()) warn += (warn.empty() ? "" : "; ") + boat_label(id) + ": " + why;
            }
            if (!warn.empty()) warn = "Some boats will ignore commands - " + warn;
        } else {
            auto it = fleet_.boats().find(teleop_target_);
            if (it == fleet_.boats().end()) {
                warn = boat_label(teleop_target_) + " has not been heard";
            } else {
                std::string why;
                check_boat(it->second, why);
                if (!why.empty()) warn = boat_label(teleop_target_) + " will ignore commands: " + why;
            }
        }
        if (!warn.empty()) {
            ImGui::PushTextWrapPos(0.0f);
            ImGui::TextColored(colors::warn, "%s", warn.c_str());
            ImGui::PopTextWrapPos();
        }
    }
    if (st.enabled && !link_.is_open()) ImGui::TextColored(colors::danger, "Not connected: nothing is being sent.");

    // ---- live output ----
    // While off, preview what would be sent so the controller can be checked.
    const TeleopOutput out = st.enabled ? st.last : compute_teleop(pad_, cfg, Clock::now());
    ImGui::Separator();
    if (out.driving) status_chip("DRIVING", colors::ok, em * 6);
    else status_chip(out.reason[0] ? out.reason : "idle", st.enabled ? colors::warn : colors::muted, em * 6);
    ImGui::SameLine();
    ImGui::TextDisabled(st.enabled ? "sending" : "preview (not sending)");
    const float bar_w = std::max(em * 6, ImGui::GetContentRegionAvail().x - em * 9.5f);
    const ImVec4 bar_c = out.driving ? colors::ok : colors::muted;
    ImGui::Text("Linear  %+5.2f m/s  ", static_cast<double>(out.linear_mps));
    ImGui::SameLine(em * 9.5f);
    signed_bar(cfg.max_linear_mps > 0 ? out.linear_mps / cfg.max_linear_mps : 0.0f, bar_c, bar_w, em * 0.8f);
    ImGui::Text("Angular %+5.2f rad/s", static_cast<double>(out.angular_radps));
    ImGui::SameLine(em * 9.5f);
    signed_bar(cfg.max_angular_radps > 0 ? out.angular_radps / cfg.max_angular_radps : 0.0f, bar_c, bar_w,
               em * 0.8f);
    ImGui::TextDisabled("Sent %llu", static_cast<unsigned long long>(st.sent));
    ImGui::SameLine();
    if (st.send_failures) ImGui::TextColored(colors::danger, "failures %llu", static_cast<unsigned long long>(st.send_failures));
    else ImGui::TextDisabled("failures 0");

    // ---- gamepad ----
    ImGui::Separator();
    if (pad_.connected) {
        ImGui::TextColored(colors::ok, "Gamepad:");
        ImGui::SameLine();
        ImGui::TextUnformatted(pad_.name.c_str());
    } else {
        ImGui::TextColored(colors::warn, "No gamepad connected");
    }
    const float r = em * 2.4f;
    const bool lin_x = is_axis_x(cfg.linear_axis), ang_x = is_axis_x(cfg.angular_axis);
    draw_stick("Left", pad_.axis(GamepadAxis::LeftX), pad_.axis(GamepadAxis::LeftY), cfg.deadzone,
               (is_left(cfg.linear_axis) && lin_x) || (is_left(cfg.angular_axis) && ang_x),
               (is_left(cfg.linear_axis) && !lin_x) || (is_left(cfg.angular_axis) && !ang_x), r);
    ImGui::SameLine(0, em);
    draw_stick("Right", pad_.axis(GamepadAxis::RightX), pad_.axis(GamepadAxis::RightY), cfg.deadzone,
               (is_right(cfg.linear_axis) && lin_x) || (is_right(cfg.angular_axis) && ang_x),
               (is_right(cfg.linear_axis) && !lin_x) || (is_right(cfg.angular_axis) && !ang_x), r);
    ImGui::SameLine(0, em);
    draw_trigger("LT", pad_.axis(GamepadAxis::LeftTrigger), em * 0.9f, r * 2);
    ImGui::SameLine(0, em * 0.5f);
    draw_trigger("RT", pad_.axis(GamepadAxis::RightTrigger), em * 0.9f, r * 2);
    ImGui::SameLine(0, em);
    ImGui::BeginGroup();
    const bool deadman = pad_.connected && pad_.button(cfg.deadman);
    char dm[48];
    std::snprintf(dm, sizeof dm, "%s %s", to_string(cfg.deadman), deadman ? "held" : "released");
    if (cfg.require_deadman) status_chip(dm, deadman ? colors::ok : colors::muted, em * 6);
    else status_chip("no deadman", colors::warn, em * 6);
    ImGui::TextDisabled("deadman");
    ImGui::EndGroup();

    // Help line built from the live mapping.
    auto stick_name = [](GamepadAxis a) -> const char* {
        switch (a) {
        case GamepadAxis::LeftX: case GamepadAxis::LeftY: return "Left stick";
        case GamepadAxis::RightX: case GamepadAxis::RightY: return "Right stick";
        default: return to_string(a);
        }
    };
    ImGui::PushTextWrapPos(0.0f);
    if (cfg.require_deadman)
        ImGui::TextDisabled("Hold %s to drive · %s: throttle · %s: turn · %s: Disable target", to_string(cfg.deadman),
                            stick_name(cfg.linear_axis), stick_name(cfg.angular_axis), to_string(cfg.disable_button));
    else
        ImGui::TextDisabled("%s: throttle · %s: turn · %s: Disable target", stick_name(cfg.linear_axis),
                            stick_name(cfg.angular_axis), to_string(cfg.disable_button));
    ImGui::PopTextWrapPos();

    ImGui::Spacing();
    draw_teleop_config();
    ImGui::End();
}

void App::draw_teleop_config() {
    if (!ImGui::CollapsingHeader("Configuration")) return;
    const float em = ImGui::GetFontSize();
    TeleopConfig c = settings_.teleop;
    bool changed = false;
    ImGui::PushItemWidth(em * 9);

    ImGui::SeparatorText("Mapping");
    changed |= enum_combo("Throttle axis", c.linear_axis);
    ImGui::SameLine();
    changed |= ImGui::Checkbox("Invert##lin", &c.invert_linear);
    changed |= enum_combo("Turn axis", c.angular_axis);
    ImGui::SameLine();
    changed |= ImGui::Checkbox("Invert##ang", &c.invert_angular);
    changed |= ImGui::SliderFloat("Deadzone", &c.deadzone, 0.0f, 0.5f, "%.2f");
    changed |= ImGui::SliderFloat("Expo", &c.expo, 0.0f, 1.0f, "%.2f");
    help_marker("0 = linear response, 1 = cubic (fine control near centre)");
    changed |= ImGui::SliderFloat("Max speed", &c.max_linear_mps, 0.1f, 5.0f, "%.2f m/s");
    changed |= ImGui::SliderFloat("Max turn rate", &c.max_angular_radps, 0.1f, 5.0f, "%.2f rad/s");

    ImGui::SeparatorText("Safety");
    changed |= ImGui::Checkbox("Require deadman", &c.require_deadman);
    if (!c.require_deadman) {
        ImGui::SameLine();
        ImGui::TextColored(colors::warn, "not recommended");
    }
    changed |= enum_combo("Deadman button", c.deadman);
    changed |= enum_combo("Disable button", c.disable_button);
    help_marker("Pressing it sends DISABLE to the teleop target, whether or not teleop is on");

    ImGui::SeparatorText("Timing");
    changed |= ImGui::SliderFloat("Send rate", &c.rate_hz, 1.0f, 30.0f, "%.0f Hz");
    changed |= ImGui::SliderInt("Input stale after", &c.input_stale_ms, 50, 2000, "%d ms");
    changed |= ImGui::SliderInt("Stop burst", &c.stop_burst, 0, 10, "%d cmds");
    help_marker("Zero commands sent to the old target when teleop is switched off or the target changes");

    ImGui::SeparatorText("Disable command");
    bool cmd_changed = ImGui::SliderInt("Disable repeats", &settings_.disable_repeats, 1, 10);
    cmd_changed |= ImGui::SliderInt("Repeat interval", &settings_.repeat_interval_ms, 20, 1000, "%d ms");
    if (cmd_changed) apply_commander_config();
    ImGui::PopItemWidth();

    if (ImGui::Button("Reset to defaults")) {
        c = TeleopConfig{};
        settings_.disable_repeats = 3;
        settings_.repeat_interval_ms = 100;
        apply_commander_config();
        changed = true;
    }
    if (changed) apply_teleop_config(c);
}

}  // namespace basestation::app
