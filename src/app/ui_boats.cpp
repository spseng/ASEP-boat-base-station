// Boats table: one row per boat heard, the fleet at a glance.

#include "app.h"

#include <boat_defs/mode.h>

#include <cmath>
#include <cstdio>

namespace basestation::app {

namespace {

void dash() { ImGui::TextDisabled("-"); }

void rssi_snr(float rssi, float snr) {
    if (!std::isfinite(rssi)) {
        dash();
        return;
    }
    if (std::isfinite(snr))
        ImGui::Text("%.0f / %.1f", static_cast<double>(rssi), static_cast<double>(snr));
    else
        ImGui::Text("%.0f / -", static_cast<double>(rssi));
}

}  // namespace

void App::draw_boats_table() {
    if (!settings_.show_boats) return;
    if (!ImGui::Begin("Boats", &settings_.show_boats)) {
        ImGui::End();
        return;
    }
    const auto& boats = fleet_.boats();
    if (boats.empty()) {
        ImGui::TextDisabled(link_.is_open() ? "No boats heard yet." : "Not connected.");
        ImGui::End();
        return;
    }

    // Not resizable on purpose: fixed-fit columns then keep auto-fitting to
    // their content (a resizable column freezes its first-frame width).
    const ImGuiTableFlags flags = ImGuiTableFlags_Reorderable |
                                  ImGuiTableFlags_Hideable | ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV |
                                  ImGuiTableFlags_BordersOuterH | ImGuiTableFlags_ScrollY | ImGuiTableFlags_ScrollX |
                                  ImGuiTableFlags_SizingFixedFit;
    ImGui::PushStyleVar(ImGuiStyleVar_CellPadding, ImVec2(4.0f * ui_scale_, 3.0f * ui_scale_));
    const bool table = ImGui::BeginTable("##boats", 12, flags);
    ImGui::PopStyleVar();
    if (table) {
        ImGui::TableSetupScrollFreeze(1, 1);
        ImGui::TableSetupColumn("ID", ImGuiTableColumnFlags_NoHide);
        ImGui::TableSetupColumn("Heard");
        ImGui::TableSetupColumn("Mode");
        ImGui::TableSetupColumn("Armed");
        ImGui::TableSetupColumn("Gate");
        ImGui::TableSetupColumn("Faults");
        ImGui::TableSetupColumn("Scalar");
        ImGui::TableSetupColumn("Heading");
        ImGui::TableSetupColumn("Base hears");
        ImGui::TableSetupColumn("Boat hears");
        ImGui::TableSetupColumn("Loss");
        ImGui::TableSetupColumn("Frames", ImGuiTableColumnFlags_DefaultHide);
        // Headers drawn by hand so some can explain themselves on hover.
        static const char* const tips[12] = {
            nullptr, "Time since any frame from this boat", nullptr, nullptr, "Output gate (Disable trips it)",
            "Fault flags from the boat's Status", "SelfStatus scalar value", "Compass heading, else course over ground",
            "RSSI dBm / SNR dB of this boat's frames at the base station (RxInfo)",
            "RSSI dBm / SNR dB of the base station's last frame at this boat (from its Status)",
            "Frames lost, from sequence-number gaps, all message types", "Frames received from this boat"};
        // Right-click a header to show / hide columns.
        ImGui::TableNextRow(ImGuiTableRowFlags_Headers);
        for (int col = 0; col < 12; ++col) {
            if (!ImGui::TableSetColumnIndex(col)) continue;
            ImGui::TableHeader(ImGui::TableGetColumnName(col));
            if (tips[col] && ImGui::IsItemHovered(ImGuiHoveredFlags_ForTooltip)) ImGui::SetTooltip("%s", tips[col]);
        }

        for (const auto& [id, b] : boats) {
            ImGui::TableNextRow();
            ImGui::PushID(id);
            const double age = age_s(b);
            const bool stale = age > 3.0;

            // ID + row selection.
            ImGui::TableNextColumn();
            ImGui::PushStyleColor(ImGuiCol_Text, boat_color(id));
            char label[16];
            std::snprintf(label, sizeof label, "%s", boat_label(id).c_str());
            if (ImGui::Selectable(label, selected_ == id,
                                  ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowOverlap))
                select_boat(id);
            ImGui::PopStyleColor();

            ImGui::TableNextColumn();
            ImGui::TextColored(age_color(age), "%s", format_duration(age).c_str());

            if (b.status) {
                const lora::Status& st = *b.status;
                ImGui::TableNextColumn();
                if (names::find_mode(st.mode)) {
                    ImGui::TextUnformatted(names::mode_name(st.mode).c_str());
                } else {
                    ImGui::TextColored(colors::warn, "%s", names::mode_name(st.mode).c_str());
                    if (ImGui::IsItemHovered(ImGuiHoveredFlags_ForTooltip))
                        ImGui::SetTooltip("Unknown mode: the boat runs firmware with a mode this app does not know");
                }

                ImGui::TableNextColumn();
                const bool armed = st.armed == static_cast<uint8_t>(boat::mode::ArmedState::ARMED);
                if (armed) ImGui::TextUnformatted(names::armed_name(st.armed).c_str());
                else ImGui::TextDisabled("%s", names::armed_name(st.armed).c_str());

                ImGui::TableNextColumn();
                const bool tripped = st.gate_state == static_cast<uint8_t>(boat::mode::GateState::TRIPPED);
                const bool enabled = st.gate_state == static_cast<uint8_t>(boat::mode::GateState::ENABLED);
                ImGui::TextColored(tripped ? colors::danger : enabled ? colors::ok : colors::warn, "%s",
                                   names::gate_name(st.gate_state).c_str());

                ImGui::TableNextColumn();
                if (st.fault_flags) ImGui::TextColored(colors::warn, "%s", names::fault_list(st.fault_flags).c_str());
                else dash();
            } else {
                for (int i = 0; i < 4; ++i) {
                    ImGui::TableNextColumn();
                    dash();
                }
            }

            ImGui::TableNextColumn();
            if (b.self_status) ImGui::Text("%.2f °C", static_cast<double>(b.self_status->self.scalar));
            else dash();

            ImGui::TableNextColumn();
            const float hdg = b.heading_deg();
            if (std::isfinite(hdg)) ImGui::Text("%03.0f°", static_cast<double>(hdg));
            else dash();

            ImGui::TableNextColumn();
            if (b.rx_info) rssi_snr(b.rx_info->rssi, b.rx_info->snr);
            else dash();

            ImGui::TableNextColumn();
            if (b.status) rssi_snr(b.status->gs_rssi, b.status->gs_snr);
            else dash();

            ImGui::TableNextColumn();
            const SeqStats total = b.total_seq();
            const double loss = total.loss_ratio() * 100.0;
            const ImVec4 lc = loss >= 30 ? colors::danger : loss >= 10 ? colors::warn : ImGui::GetStyleColorVec4(ImGuiCol_Text);
            ImGui::TextColored(lc, "%.1f %%", loss);

            ImGui::TableNextColumn();
            ImGui::Text("%llu", static_cast<unsigned long long>(b.frames));

            // Stale rows are dimmed as a whole so they read as "old news".
            if (stale) ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg1, IM_COL32(255, 184, 51, 18));
            ImGui::PopID();
        }
        ImGui::EndTable();
    }
    ImGui::End();
}

}  // namespace basestation::app
