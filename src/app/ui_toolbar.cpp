// Main menu bar and the always-visible toolbar: connection, recording,
// teleop state and the DISABLE ALL button.

#include "app.h"

#include <boat_defs/ids.h>

#include <imgui_internal.h>  // BeginViewportSideBar

#include <cstdio>

namespace basestation::app {

void App::draw_menu_bar() {
    if (!ImGui::BeginMainMenuBar()) return;

    if (ImGui::BeginMenu("File")) {
        if (ImGui::MenuItem("Quit", "Ctrl+Q")) quit_ = true;
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("View")) {
        ImGui::MenuItem("Fleet", nullptr, &settings_.show_fleet);
        ImGui::MenuItem("Boats", nullptr, &settings_.show_boats);
        ImGui::MenuItem("Commands", nullptr, &settings_.show_commands);
        ImGui::MenuItem("Teleop", nullptr, &settings_.show_teleop);
        ImGui::MenuItem("Plots", nullptr, &settings_.show_plots);
        ImGui::MenuItem("Link", nullptr, &settings_.show_link);
        ImGui::MenuItem("Events", nullptr, &settings_.show_events);
        ImGui::Separator();
        float& scale = ImGui::GetStyle().FontScaleMain;
        ImGui::SetNextItemWidth(ImGui::GetFontSize() * 8);
        ImGui::SliderFloat("Text size", &scale, 0.75f, 1.75f, "%.2fx");
        if (ImGui::MenuItem("Reset layout")) reset_layout_ = true;
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Help")) {
        const TeleopConfig& c = settings_.teleop;
        ImGui::TextDisabled("Teleop (Xbox controller)");
        ImGui::BulletText("Hold %s to drive (deadman)", to_string(c.deadman));
        ImGui::BulletText("%s: throttle, %s: turn", to_string(c.linear_axis), to_string(c.angular_axis));
        ImGui::BulletText("%s: DISABLE the teleop target", to_string(c.disable_button));
        ImGui::Separator();
        ImGui::TextDisabled("Map");
        ImGui::BulletText("Click a boat to select it; drag to pan, wheel to zoom");
        ImGui::BulletText("Double-click the map to fit everything");
        ImGui::EndMenu();
    }

    // Right side: local clock, useful when reading the event log.
    const std::string clock = format_clock(unix_time_us()).substr(0, 8);
    const float w = ImGui::CalcTextSize(clock.c_str()).x + ImGui::GetStyle().ItemSpacing.x * 2;
    ImGui::SameLine(ImGui::GetWindowWidth() - w);
    ImGui::TextDisabled("%s", clock.c_str());
    ImGui::EndMainMenuBar();

    if (ImGui::Shortcut(ImGuiMod_Ctrl | ImGuiKey_Q, ImGuiInputFlags_RouteGlobal)) quit_ = true;
}

void App::draw_toolbar() {
    ImGuiStyle& style = ImGui::GetStyle();
    const float pad_y = 6.0f * ui_scale_;
    const float height = ImGui::GetFrameHeight() + pad_y * 2;
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(style.WindowPadding.x, pad_y));
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.105f, 0.115f, 0.135f, 1.0f));
    const bool open = ImGui::BeginViewportSideBar("##toolbar", ImGui::GetMainViewport(), ImGuiDir_Up, height,
                                                  ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoSavedSettings |
                                                      ImGuiWindowFlags_NoScrollWithMouse);
    ImGui::PopStyleColor();
    ImGui::PopStyleVar();
    if (!open) {
        ImGui::End();
        return;
    }

    const bool open_link = link_.is_open();
    const bool busy = want_connected_ || open_link;  // connected or reconnecting
    const float em = ImGui::GetFontSize();

    // ---- port / baud / connect ----
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted("Port");
    ImGui::SameLine();
    ImGui::BeginDisabled(busy);
    ImGui::SetNextItemWidth(em * 11.5f);
    const bool enter = ImGui::InputTextWithHint("##port", "/dev/ttyUSB0", port_buf_.data(), port_buf_.size(),
                                                ImGuiInputTextFlags_EnterReturnsTrue);
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_ForTooltip | ImGuiHoveredFlags_AllowWhenDisabled))
        ImGui::SetTooltip("Serial device of the base-station ESP32.\nType any path (e.g. a /dev/pts/N from the "
                          "simulator) or pick one from the list.");
    ImGui::SameLine(0, 0);
    if (ImGui::BeginCombo("##portlist", nullptr, ImGuiComboFlags_NoPreview | ImGuiComboFlags_PopupAlignLeft)) {
        if (ImGui::IsWindowAppearing()) ports_ = list_serial_ports();
        if (ports_.empty()) ImGui::TextDisabled("No serial ports found");
        for (const SerialPortInfo& p : ports_) {
            std::string label = p.path;
            if (!p.description.empty()) label += "  (" + p.description + ")";
            if (ImGui::Selectable(label.c_str(), p.path == port_buf_.data()))
                std::snprintf(port_buf_.data(), port_buf_.size(), "%s", p.path.c_str());
        }
        ImGui::EndCombo();
    }
    ImGui::SameLine();
    ImGui::SetNextItemWidth(em * 5.5f);
    char baud_label[16];
    std::snprintf(baud_label, sizeof baud_label, "%u", baud_);
    if (ImGui::BeginCombo("##baud", baud_label)) {
        for (uint32_t b : common_baud_rates()) {
            std::snprintf(baud_label, sizeof baud_label, "%u", b);
            if (ImGui::Selectable(baud_label, b == baud_)) baud_ = b;
        }
        ImGui::EndCombo();
    }
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_ForTooltip)) ImGui::SetTooltip("Baud rate");
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (busy) {
        if (ImGui::Button("Disconnect", ImVec2(em * 5.5f, 0))) disconnect();
    } else {
        if (ImGui::Button("Connect", ImVec2(em * 5.5f, 0)) || enter) connect(port_buf_.data(), baud_, true);
    }

    // ---- link status ----
    ImGui::SameLine(0, em);
    if (open_link) {
        status_chip("LINKED", colors::ok);
        const LinkSession::Stats st = link_.stats();
        ImGui::SameLine();
        ImGui::AlignTextToFramePadding();
        ImGui::Text("%.0f fr/s", rx_rate_);
        if (st.rx_bad > 0) {
            ImGui::SameLine();
            ImGui::TextColored(colors::warn, "%llu bad", static_cast<unsigned long long>(st.rx_bad));
        }
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_ForTooltip))
            ImGui::SetTooltip("%s @ %u baud\nFrames received per second; corrupt frames dropped",
                              link_.path().c_str(), link_.baud());
    } else if (want_connected_) {
        status_chip(settings_.auto_reconnect ? "RECONNECTING" : "LINK LOST",
                    settings_.auto_reconnect ? colors::warn : colors::danger);
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_ForTooltip))
            ImGui::SetTooltip("%s: %s", conn_path_.c_str(), link_error_.c_str());
        ImGui::SameLine();
        ImGui::AlignTextToFramePadding();
        ImGui::TextColored(colors::danger, "%s", link_error_.c_str());
    } else {
        status_chip("OFFLINE", colors::muted);
        if (!link_error_.empty()) {
            ImGui::SameLine();
            ImGui::AlignTextToFramePadding();
            ImGui::TextColored(colors::danger, "%s", link_error_.c_str());
        }
    }

    // ---- recording ----
    ImGui::SameLine(0, em);
    if (ImGui::Checkbox("Rec", &record_)) settings_.record = record_;
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_ForTooltip))
        ImGui::SetTooltip("Record every frame to a session log on the next connect.\nLog folder: %s",
                          effective_log_dir().c_str());
    if (frame_log_.is_open()) {
        ImGui::SameLine();
        // Blink the dot slowly so recording is noticeable but not noisy.
        const bool on = static_cast<int>(ImGui::GetTime() * 1.5) % 2 == 0;
        ImGui::AlignTextToFramePadding();
        ImGui::TextColored(on ? colors::danger : ImVec4(0.5f, 0.2f, 0.2f, 1.0f), "REC");
        ImGui::SameLine(0, 4);
        ImGui::Text("%llu", static_cast<unsigned long long>(frame_log_.records()));
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_ForTooltip))
            ImGui::SetTooltip("Recording to %s", frame_log_.path().c_str());
    }

    // ---- teleop state ----
    ImGui::SameLine(0, em);
    ImGui::AlignTextToFramePadding();
    if (teleop_status_.enabled) {
        const bool driving = teleop_status_.last.driving;
        const ImVec4 c = driving ? colors::ok : colors::warn;
        ImGui::TextColored(c, "TELEOP -> %s", boat_label(teleop_status_.target).c_str());
        ImGui::SameLine();
        status_chip(driving ? "DRIVING" : (teleop_status_.last.reason[0] ? teleop_status_.last.reason : "idle"), c);
    } else {
        ImGui::TextDisabled("Teleop off");
    }

    // ---- DISABLE ALL (right-aligned, always visible) ----
    ImGui::PushFont(fonts_.bold, 0.0f);
    const char* label = "DISABLE ALL";
    const float bw = ImGui::CalcTextSize(label).x + style.FramePadding.x * 4;
    ImGui::SameLine(ImGui::GetWindowWidth() - bw - style.WindowPadding.x);
    ImGui::BeginDisabled(!open_link);
    ImGui::PushStyleColor(ImGuiCol_Button, colors::danger_bg);
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.85f, 0.18f, 0.18f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(1.0f, 0.25f, 0.25f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1, 1, 1, 1));
    if (ImGui::Button(label, ImVec2(bw, 0))) commander_.disable(boat::ids::BROADCAST_ID);
    ImGui::PopStyleColor(4);
    ImGui::EndDisabled();
    ImGui::PopFont();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_ForTooltip | ImGuiHoveredFlags_AllowWhenDisabled))
        ImGui::SetTooltip(open_link ? "Trip the output gate of EVERY boat (sent %d times)"
                                    : "Not connected (would trip the output gate of every boat, sent %d times)",
                          settings_.disable_repeats);

    ImGui::End();
}

}  // namespace basestation::app
