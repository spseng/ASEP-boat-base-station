// Formatting and small widgets shared by the panels.

#include "app.h"

#include <boat_defs/ids.h>

#include <cstdio>
#include <ctime>

namespace basestation::app {

std::string boat_label(uint8_t id) {
    if (id == boat::ids::BROADCAST_ID) return "ALL";
    return "B" + std::to_string(id);
}

const char* msg_type_name(uint8_t type) {
    switch (type) {
    case static_cast<uint8_t>(lora::MsgType::Disable): return "Disable";
    case static_cast<uint8_t>(lora::MsgType::SelfStatus): return "SelfStatus";
    case static_cast<uint8_t>(lora::MsgType::Command): return "Command";
    case static_cast<uint8_t>(lora::MsgType::Status): return "Status";
    case static_cast<uint8_t>(lora::MsgType::Reenable): return "Reenable";
    case static_cast<uint8_t>(lora::MsgType::SetMode): return "SetMode";
    case static_cast<uint8_t>(base::MsgType::RxInfo): return "RxInfo";
    case static_cast<uint8_t>(base::MsgType::BaseStatus): return "BaseStatus";
    default: return "unknown";
    }
}

ImVec4 age_color(double age_s) {
    if (age_s < 3.0) return colors::ok;
    if (age_s < 10.0) return colors::warn;
    return colors::danger;
}

std::string format_clock(int64_t unix_us) {
    const std::time_t secs = static_cast<std::time_t>(unix_us / 1000000);
    const int ms = static_cast<int>((unix_us / 1000) % 1000);
    std::tm tm{};
    localtime_r(&secs, &tm);
    char buf[32];
    std::snprintf(buf, sizeof buf, "%02d:%02d:%02d.%03d", tm.tm_hour, tm.tm_min, tm.tm_sec, ms);
    return buf;
}

std::string format_duration(double s) {
    char buf[48];
    if (s < 0) s = 0;
    if (s < 60) {
        std::snprintf(buf, sizeof buf, "%.1f s", s);
    } else if (s < 3600) {
        const int m = static_cast<int>(s / 60);
        std::snprintf(buf, sizeof buf, "%d min %02d s", m, static_cast<int>(s) % 60);
    } else {
        const int h = static_cast<int>(s / 3600);
        std::snprintf(buf, sizeof buf, "%d h %02d min", h, (static_cast<int>(s) % 3600) / 60);
    }
    return buf;
}

void help_marker(const char* text) {
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::BeginItemTooltip()) {
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 30.0f);
        ImGui::TextUnformatted(text);
        ImGui::PopTextWrapPos();
        ImGui::EndTooltip();
    }
}

void status_chip(const char* text, const ImVec4& color, float min_width) {
    const ImVec2 pad(ImGui::GetStyle().FramePadding.x, ImGui::GetStyle().FramePadding.y * 0.5f);
    const ImVec2 ts = ImGui::CalcTextSize(text);
    ImVec2 size(ts.x + pad.x * 2, ts.y + pad.y * 2);
    if (size.x < min_width) size.x = min_width;
    const ImVec2 p = ImGui::GetCursorScreenPos();
    ImDrawList* dl = ImGui::GetWindowDrawList();
    ImVec4 bg = color;
    bg.w = 0.22f;
    dl->AddRectFilled(p, ImVec2(p.x + size.x, p.y + size.y), ImGui::GetColorU32(bg),
                      ImGui::GetStyle().FrameRounding);
    dl->AddRect(p, ImVec2(p.x + size.x, p.y + size.y), ImGui::GetColorU32(color),
                ImGui::GetStyle().FrameRounding);
    dl->AddText(ImVec2(p.x + (size.x - ts.x) * 0.5f, p.y + pad.y), ImGui::GetColorU32(color), text);
    ImGui::Dummy(size);
}

}  // namespace basestation::app
