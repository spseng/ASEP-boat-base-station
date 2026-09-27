// Commands panel: Disable / Re-enable / Set mode for one boat or all, and
// the selected boat's details.

#include "app.h"

#include <boat_defs/ids.h>
#include <boat_defs/mode.h>

#include <imgui_internal.h>  // BringWindowToDisplayFront

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace basestation::app {

namespace {

constexpr int MODE_COUNT = static_cast<int>(sizeof(names::MODES) / sizeof(names::MODES[0]));

bool red_button(const char* label, const ImVec2& size = ImVec2(0, 0)) {
    ImGui::PushStyleColor(ImGuiCol_Button, colors::danger_bg);
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.85f, 0.18f, 0.18f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(1.0f, 0.25f, 0.25f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1, 1, 1, 1));
    const bool pressed = ImGui::Button(label, size);
    ImGui::PopStyleColor(4);
    return pressed;
}

std::string target_text(uint8_t id) {
    if (id == 0) return "(none)";
    if (id == boat::ids::BROADCAST_ID) return "ALL (broadcast)";
    return boat_label(id);
}

}  // namespace

bool App::target_combo(const char* label, uint8_t& target, bool allow_all) {
    bool changed = false;
    const std::string preview = target_text(target);
    if (ImGui::BeginCombo(label, preview.c_str())) {
        for (const auto& [id, b] : fleet_.boats()) {
            std::string item = boat_label(id);
            if (b.status) item += "   " + names::mode_name(b.status->mode);
            ImGui::PushStyleColor(ImGuiCol_Text, boat_color(id));
            if (ImGui::Selectable(item.c_str(), target == id)) {
                target = id;
                changed = true;
            }
            ImGui::PopStyleColor();
        }
        if (allow_all) {
            if (!fleet_.boats().empty()) ImGui::Separator();
            if (ImGui::Selectable("ALL (broadcast)", target == boat::ids::BROADCAST_ID)) {
                target = boat::ids::BROADCAST_ID;
                changed = true;
            }
        }
        ImGui::EndCombo();
    }
    return changed;
}

void App::draw_commands() {
    if (!settings_.show_commands) return;
    if (!ImGui::Begin("Commands", &settings_.show_commands)) {
        ImGui::End();
        return;
    }
    const float em = ImGui::GetFontSize();
    const bool connected = link_.is_open();

    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted("Target");
    ImGui::SameLine(em * 4.5f);
    ImGui::SetNextItemWidth(-FLT_MIN);
    target_combo("##cmdtarget", cmd_target_, true);

    const bool have_target = cmd_target_ != 0;
    ImGui::BeginDisabled(!connected || !have_target);

    // DISABLE: immediate, no confirmation. It is the safe direction.
    const float half = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) / 2;
    ImGui::PushFont(fonts_.bold, 0.0f);
    if (red_button("DISABLE", ImVec2(half, em * 2.0f))) commander_.disable(cmd_target_);
    ImGui::PopFont();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_ForTooltip))
        ImGui::SetTooltip("Trip the output gate of %s now (sent %d times)", target_text(cmd_target_).c_str(),
                          settings_.disable_repeats);
    ImGui::SameLine();
    if (ImGui::Button("Re-enable...", ImVec2(half, em * 2.0f))) {
        confirm_reenable_open_ = true;
        confirm_arm_open_ = false;
        confirm_appearing_ = true;
        popup_target_ = cmd_target_;
    }
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_ForTooltip))
        ImGui::SetTooltip("Ask %s to clear a tripped gate (asks for confirmation)", target_text(cmd_target_).c_str());

    // Set mode: known modes from names::MODES, plus a raw value so modes
    // added to newer firmware can be sent before this app learns them.
    ImGui::Spacing();
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted("Mode");
    ImGui::SameLine(em * 4.5f);
    // Combo takes whatever is left after the Armed checkbox and Send button.
    const ImGuiStyle& style = ImGui::GetStyle();
    const float armed_w = ImGui::GetFrameHeight() + style.ItemInnerSpacing.x + ImGui::CalcTextSize("Armed").x;
    const float send_w = ImGui::CalcTextSize("Send").x + style.FramePadding.x * 2 + em;
    ImGui::SetNextItemWidth(std::max(em * 6, ImGui::GetContentRegionAvail().x - armed_w - send_w - style.ItemSpacing.x * 2));
    const char* preview = cmd_mode_idx_ < MODE_COUNT ? names::MODES[cmd_mode_idx_].name : "Other (raw)";
    if (ImGui::BeginCombo("##mode", preview)) {
        for (int i = 0; i < MODE_COUNT; ++i) {
            if (ImGui::Selectable(names::MODES[i].name, cmd_mode_idx_ == i)) cmd_mode_idx_ = i;
            if (ImGui::IsItemHovered(ImGuiHoveredFlags_ForTooltip))
                ImGui::SetTooltip("%s", names::MODES[i].description);
        }
        ImGui::Separator();
        if (ImGui::Selectable("Other (raw value)", cmd_mode_idx_ == MODE_COUNT)) cmd_mode_idx_ = MODE_COUNT;
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_ForTooltip))
            ImGui::SetTooltip("Send any mode number, e.g. one added to newer boat firmware");
        ImGui::EndCombo();
    }
    if (cmd_mode_idx_ < MODE_COUNT && ImGui::IsItemHovered(ImGuiHoveredFlags_ForTooltip))
        ImGui::SetTooltip("%s", names::MODES[cmd_mode_idx_].description);
    ImGui::SameLine();
    ImGui::Checkbox("Armed", &cmd_armed_);
    ImGui::SameLine();
    const uint8_t mode_raw = cmd_mode_idx_ < MODE_COUNT ? static_cast<uint8_t>(names::MODES[cmd_mode_idx_].mode)
                                                        : static_cast<uint8_t>(cmd_mode_raw_);
    if (ImGui::Button("Send", ImVec2(-FLT_MIN, 0))) {
        if (cmd_armed_) {
            // Arming can make a boat move: confirm first.
            confirm_arm_open_ = true;
            confirm_reenable_open_ = false;
            confirm_appearing_ = true;
            popup_target_ = cmd_target_;
            popup_mode_ = mode_raw;
            popup_armed_ = true;
        } else {
            commander_.set_mode(cmd_target_, static_cast<boat::mode::Mode>(mode_raw),
                                boat::mode::ArmedState::DISARMED);
        }
    }
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_ForTooltip))
        ImGui::SetTooltip("Send SetMode %s %s to %s%s", names::mode_name(mode_raw).c_str(),
                          cmd_armed_ ? "ARMED" : "DISARMED", target_text(cmd_target_).c_str(),
                          cmd_armed_ ? " (asks for confirmation)" : "");
    if (cmd_mode_idx_ == MODE_COUNT) {
        ImGui::AlignTextToFramePadding();
        ImGui::TextDisabled("Raw value");
        ImGui::SameLine(em * 4.5f);
        ImGui::SetNextItemWidth(em * 7.0f);
        ImGui::InputInt("##moderaw", &cmd_mode_raw_, 1, 10);
        cmd_mode_raw_ = std::clamp(cmd_mode_raw_, 0, 255);
        ImGui::SameLine();
        ImGui::TextDisabled("= %s", names::mode_name(static_cast<uint8_t>(cmd_mode_raw_)).c_str());
    }
    ImGui::EndDisabled();
    if (!connected) ImGui::TextDisabled("Not connected: commands cannot be sent.");
    else if (!have_target) ImGui::TextDisabled("Select a boat or pick a target.");

    ImGui::Separator();
    if (selected_) {
        auto it = fleet_.boats().find(*selected_);
        if (it != fleet_.boats().end()) draw_boat_detail(it->second);
        else ImGui::TextDisabled("Boat %u not heard yet.", unsigned(*selected_));
    } else {
        ImGui::TextDisabled("Select a boat (map or table) for details.");
    }
    ImGui::End();
}

void App::draw_boat_detail(const BoatState& b) {
    const Clock::time_point now = Clock::now();
    ImGui::PushFont(fonts_.bold, 0.0f);
    ImGui::TextColored(boat_color(b.id), "Boat %u", unsigned(b.id));
    ImGui::PopFont();
    if (b.status) {
        ImGui::SameLine();
        const bool tripped = b.status->gate_state == static_cast<uint8_t>(boat::mode::GateState::TRIPPED);
        status_chip(names::gate_name(b.status->gate_state).c_str(), tripped ? colors::danger : colors::ok);
        ImGui::SameLine();
        status_chip(names::mode_name(b.status->mode).c_str(),
                    names::find_mode(b.status->mode) ? colors::accent : colors::warn);
        ImGui::SameLine();
        const bool armed = b.status->armed == static_cast<uint8_t>(boat::mode::ArmedState::ARMED);
        status_chip(names::armed_name(b.status->armed).c_str(), armed ? colors::warn : colors::muted);
    }

    // Wall-clock time for a steady_clock point, for "first heard at".
    auto wall = [&](Clock::time_point t) {
        const int64_t ago_us = std::chrono::duration_cast<std::chrono::microseconds>(now - t).count();
        return format_clock(unix_time_us() - ago_us);
    };

    if (ImGui::BeginTable("##detail", 2, ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_RowBg)) {
        ImGui::TableSetupColumn("k", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn("v", ImGuiTableColumnFlags_WidthStretch);
        auto row = [](const char* k) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextDisabled("%s", k);
            ImGui::TableNextColumn();
        };
        if (b.self_status) {
            const auto& self = b.self_status->self;
            const geo::LatLon ll = geo::from_e7(self.lat, self.lon);
            row("Position");
            ImGui::Text("%.7f, %.7f", ll.lat_deg, ll.lon_deg);
            row("Fix age");
            ImGui::TextColored(self.age_ms > 3000 ? colors::warn : ImGui::GetStyleColorVec4(ImGuiCol_Text), "%u ms",
                               unsigned(self.age_ms));
            row("Scalar");
            ImGui::Text("%.3f °C", static_cast<double>(self.scalar));
        }
        row("Heading");
        const float hdg = b.heading_deg();
        if (std::isfinite(hdg)) {
            const bool from_status = b.status && std::isfinite(b.status->heading_deg);
            ImGui::Text("%.1f°  (%s)", static_cast<double>(hdg), from_status ? "compass" : "course over ground");
        } else {
            ImGui::TextDisabled("unknown");
        }
        if (b.status && b.status->fault_flags) {
            row("Faults");
            ImGui::TextColored(colors::warn, "%s", names::fault_list(b.status->fault_flags).c_str());
        }
        row("First heard");
        ImGui::Text("%s", wall(b.first_heard).c_str());
        row("Last heard");
        const double age = age_s(b);
        ImGui::TextColored(age_color(age), "%s ago", format_duration(age).c_str());
        ImGui::EndTable();
    }

    ImGui::Spacing();
    if (!ImGui::TreeNodeEx("Sequence stats (per message type)", ImGuiTreeNodeFlags_DefaultOpen)) return;
    const ImGuiTableFlags tf = ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH;
    if (ImGui::BeginTable("##seq", 5, tf)) {
        ImGui::TableSetupColumn("Type");
        ImGui::TableSetupColumn("Recv");
        ImGui::TableSetupColumn("Lost");
        ImGui::TableSetupColumn("Dups");
        ImGui::TableSetupColumn("Restarts");
        ImGui::TableHeadersRow();
        for (const auto& [type, st] : b.seq) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(msg_type_name(type));
            ImGui::TableNextColumn();
            ImGui::Text("%llu", static_cast<unsigned long long>(st.received));
            ImGui::TableNextColumn();
            ImGui::TextColored(st.lost ? colors::warn : ImGui::GetStyleColorVec4(ImGuiCol_Text), "%llu (%.1f%%)",
                               static_cast<unsigned long long>(st.lost), st.loss_ratio() * 100.0);
            ImGui::TableNextColumn();
            ImGui::Text("%llu", static_cast<unsigned long long>(st.duplicates));
            ImGui::TableNextColumn();
            ImGui::Text("%llu", static_cast<unsigned long long>(st.restarts));
        }
        ImGui::EndTable();
    }
    ImGui::TreePop();
}

// Confirmation dialogs. Deliberately NOT modal popups: a modal would block
// the toolbar's DISABLE ALL (and every other control) while it is open.
// These are ordinary floating windows kept on top of the dock space.
void App::draw_confirm_popups() {
    if (!confirm_reenable_open_ && !confirm_arm_open_) return;
    const float em = ImGui::GetFontSize();
    const ImVec2 center = ImGui::GetMainViewport()->GetCenter();
    const ImGuiWindowFlags wf = ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse |
                                ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoSavedSettings;
    bool& open = confirm_reenable_open_ ? confirm_reenable_open_ : confirm_arm_open_;
    const char* title = confirm_reenable_open_ ? "Confirm re-enable" : "Confirm arming";

    ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    if (confirm_appearing_) ImGui::SetNextWindowFocus();
    confirm_appearing_ = false;
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImGui::GetStyleColorVec4(ImGuiCol_PopupBg));
    ImGui::PushStyleColor(ImGuiCol_Border, colors::warn);
    const bool visible = ImGui::Begin(title, &open, wf);
    ImGui::PopStyleColor(2);
    if (visible) {
        // Stay above docked panels even when the operator clicks elsewhere.
        ImGui::BringWindowToDisplayFront(ImGui::GetCurrentWindow());
        bool confirm = false;
        if (confirm_reenable_open_) {
            ImGui::Text("Re-enable %s?", target_text(popup_target_).c_str());
            ImGui::TextColored(colors::warn, "The output gate will be cleared. An armed boat may start moving.");
            ImGui::Spacing();
            ImGui::BeginDisabled(!link_.is_open());
            confirm = ImGui::Button("Re-enable", ImVec2(em * 7, 0));
            ImGui::EndDisabled();
            if (confirm) commander_.reenable(popup_target_);  // Commander logs success / failure
        } else {
            ImGui::Text("Set %s to %s and ARM?", target_text(popup_target_).c_str(),
                        names::mode_name(popup_mode_).c_str());
            if (const names::ModeInfo* m = names::find_mode(popup_mode_)) ImGui::TextDisabled("%s", m->description);
            else ImGui::TextColored(colors::warn, "Mode %u is not known to this app.", unsigned(popup_mode_));
            ImGui::TextColored(colors::warn, "An armed boat can drive its thrusters.");
            ImGui::Spacing();
            ImGui::BeginDisabled(!link_.is_open());
            confirm = ImGui::Button("Send", ImVec2(em * 7, 0));
            ImGui::EndDisabled();
            if (confirm) {
                commander_.set_mode(popup_target_, static_cast<boat::mode::Mode>(popup_mode_),
                                    popup_armed_ ? boat::mode::ArmedState::ARMED : boat::mode::ArmedState::DISARMED);
            }
        }
        ImGui::SameLine();
        const bool cancel = ImGui::Button("Cancel", ImVec2(em * 7, 0)) ||
                            (ImGui::IsWindowFocused() && ImGui::IsKeyPressed(ImGuiKey_Escape));
        if (confirm || cancel) open = false;
    }
    ImGui::End();
}

}  // namespace basestation::app
