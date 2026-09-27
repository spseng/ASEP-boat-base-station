// Event log: timestamped, coloured by level, filterable.

#include "app.h"

#include <vector>

namespace basestation::app {

void App::draw_events() {
    if (!settings_.show_events) return;
    if (!ImGui::Begin("Events", &settings_.show_events)) {
        ImGui::End();
        return;
    }
    const float em = ImGui::GetFontSize();

    // Copy only when something changed (the log is shared with core threads).
    bool new_events = false;
    const uint64_t gen = events_.generation();
    if (gen != events_gen_) {
        events_cache_ = events_.snapshot();
        events_gen_ = gen;
        new_events = true;
    }

    events_filter_.Draw("##filter", em * 14);
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_ForTooltip))
        ImGui::SetTooltip("Filter, e.g. \"boat 2\", \"-Teleop\" to exclude, \"gate,fault\" for either");
    ImGui::SameLine();
    ImGui::Checkbox("Auto-scroll", &events_autoscroll_);
    ImGui::SameLine();
    if (ImGui::Button("Clear")) {
        events_.clear();
        events_cache_.clear();
    }
    ImGui::SameLine();
    int warns = 0, errors = 0;
    for (const Event& e : events_cache_) {
        if (e.level == EventLevel::Warn) ++warns;
        if (e.level == EventLevel::Error) ++errors;
    }
    ImGui::TextDisabled("%zu events", events_cache_.size());
    if (warns) {
        ImGui::SameLine();
        ImGui::TextColored(colors::warn, "%d warnings", warns);
    }
    if (errors) {
        ImGui::SameLine();
        ImGui::TextColored(colors::danger, "%d errors", errors);
    }

    std::vector<int> rows;
    rows.reserve(events_cache_.size());
    for (size_t i = 0; i < events_cache_.size(); ++i) {
        if (events_filter_.PassFilter(events_cache_[i].text.c_str())) rows.push_back(static_cast<int>(i));
    }

    const ImGuiTableFlags flags = ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY | ImGuiTableFlags_BordersInnerV |
                                  ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_Resizable;
    if (ImGui::BeginTable("##events", 3, flags)) {
        ImGui::TableSetupScrollFreeze(0, 1);
        const float pad = ImGui::GetStyle().CellPadding.x * 2;
        ImGui::TableSetupColumn("Time", ImGuiTableColumnFlags_WidthFixed, ImGui::CalcTextSize("00:00:00.000").x + pad);
        ImGui::TableSetupColumn("Level", ImGuiTableColumnFlags_WidthFixed, ImGui::CalcTextSize("ERROR").x + pad);
        ImGui::TableSetupColumn("Message", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableHeadersRow();
        ImGuiListClipper clipper;
        clipper.Begin(static_cast<int>(rows.size()));
        while (clipper.Step()) {
            for (int r = clipper.DisplayStart; r < clipper.DisplayEnd; ++r) {
                const Event& e = events_cache_[static_cast<size_t>(rows[static_cast<size_t>(r)])];
                const ImVec4 c = e.level == EventLevel::Error  ? colors::danger
                                 : e.level == EventLevel::Warn ? colors::warn
                                                               : ImGui::GetStyleColorVec4(ImGuiCol_Text);
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::TextDisabled("%s", format_clock(e.unix_us).c_str());
                ImGui::TableNextColumn();
                ImGui::TextColored(c, "%s", e.level == EventLevel::Error ? "ERROR" : e.level == EventLevel::Warn ? "WARN" : "info");
                ImGui::TableNextColumn();
                ImGui::TextColored(c, "%s", e.text.c_str());
            }
        }
        if (events_autoscroll_ && new_events) ImGui::SetScrollHereY(1.0f);
        ImGui::EndTable();
    }
    ImGui::End();
}

}  // namespace basestation::app
