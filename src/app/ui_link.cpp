// Link / diagnostics: serial link, recording, base-station health, decoder
// counters and per-boat sequence loss.

#include "app.h"

#include <cstdio>

namespace basestation::app {

namespace {

void kv_row(const char* k) {
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::TextDisabled("%s", k);
    ImGui::TableNextColumn();
}

void kv_u64(const char* k, uint64_t v, bool warn_if_nonzero = false) {
    kv_row(k);
    if (warn_if_nonzero && v) ImGui::TextColored(colors::warn, "%llu", static_cast<unsigned long long>(v));
    else ImGui::Text("%llu", static_cast<unsigned long long>(v));
}

bool begin_kv(const char* id) {
    return ImGui::BeginTable(id, 2, ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_RowBg);
}

}  // namespace

void App::draw_link() {
    if (!settings_.show_link) return;
    if (!ImGui::Begin("Link", &settings_.show_link)) {
        ImGui::End();
        return;
    }
    const float em = ImGui::GetFontSize();
    const Clock::time_point now = Clock::now();

    // Two columns of sections when there is room, one otherwise.
    const bool wide = ImGui::GetContentRegionAvail().x > em * 40;
    if (wide) {
        ImGui::BeginTable("##linkcols", 2, ImGuiTableFlags_SizingStretchSame);
        ImGui::TableNextColumn();
    }

    ImGui::SeparatorText("Serial link");
    if (begin_kv("##serial")) {
        kv_row("State");
        if (link_.is_open()) ImGui::TextColored(colors::ok, "connected");
        else if (want_connected_) ImGui::TextColored(colors::warn, "lost, %s", settings_.auto_reconnect ? "reconnecting" : "not reconnecting");
        else ImGui::TextDisabled("offline");
        kv_row("Port");
        ImGui::TextUnformatted(link_.is_open() ? link_.path().c_str() : (conn_path_.empty() ? "-" : conn_path_.c_str()));
        kv_row("Baud");
        ImGui::Text("%u", link_.is_open() ? link_.baud() : baud_);
        if (connected_at_) {
            kv_row("Connected for");
            ImGui::TextUnformatted(format_duration(std::chrono::duration<double>(now - *connected_at_).count()).c_str());
        }
        if (!link_error_.empty() || !link_.last_error().empty()) {
            kv_row("Last error");
            ImGui::PushTextWrapPos(0.0f);
            ImGui::TextColored(colors::danger, "%s", link_error_.empty() ? link_.last_error().c_str() : link_error_.c_str());
            ImGui::PopTextWrapPos();
        }
        ImGui::EndTable();
    }
    ImGui::Checkbox("Auto-reconnect", &settings_.auto_reconnect);
    help_marker("When the link drops by itself (e.g. USB unplugged), retry every second while the device exists");

    const LinkSession::Stats st = link_.stats();
    ImGui::SeparatorText("Traffic");
    if (begin_kv("##traffic")) {
        kv_row("Receive rate");
        ImGui::Text("%.1f frames/s", rx_rate_);
        kv_u64("RX frames", st.rx_frames);
        kv_u64("RX bytes", st.rx_bytes);
        kv_u64("RX corrupt", st.rx_bad, true);
        kv_u64("RX overflows", st.rx_overflows, true);
        kv_u64("TX frames", st.tx_frames);
        kv_u64("TX bytes", st.tx_bytes);
        kv_u64("TX errors", st.tx_errors, true);
        ImGui::EndTable();
    }

    ImGui::SeparatorText("Recording");
    if (log_dir_buf_[0] == '\0' && !log_dir_.empty()) std::snprintf(log_dir_buf_.data(), log_dir_buf_.size(), "%s", log_dir_.c_str());
    ImGui::SetNextItemWidth(-em * 5);
    const std::string def = effective_log_dir();
    if (ImGui::InputTextWithHint("##logdir", def.c_str(), log_dir_buf_.data(), log_dir_buf_.size())) {
        log_dir_ = log_dir_buf_.data();
        settings_.log_dir = log_dir_;
    }
    ImGui::SameLine();
    ImGui::TextUnformatted("Folder");
    help_marker("Session logs (.aseplog) are written here on each connect when Rec is on. Empty = default.");
    if (frame_log_.is_open()) {
        ImGui::TextColored(colors::danger, "REC");
        ImGui::SameLine();
        ImGui::TextWrapped("%s  (%llu records)", frame_log_.path().c_str(),
                           static_cast<unsigned long long>(frame_log_.records()));
    } else {
        ImGui::TextDisabled(record_ ? "Not recording (starts on connect)" : "Recording off");
    }

    if (wide) ImGui::TableNextColumn();

    ImGui::SeparatorText("Base station ESP32");
    const BaseStationState& bs = fleet_.base_station();
    if (bs.status) {
        if (begin_kv("##base")) {
            const base::BaseStatus& s = *bs.status;
            kv_row("Uptime");
            ImGui::TextUnformatted(format_duration(s.uptime_ms / 1000.0).c_str());
            kv_u64("LoRa RX ok", s.rx_ok);
            kv_u64("LoRa RX bad", s.rx_bad, true);
            kv_u64("LoRa TX", s.tx_count);
            kv_u64("Reboots seen", bs.reboots, true);
            kv_row("Status age");
            const double age = std::chrono::duration<double>(now - bs.status_time).count();
            ImGui::TextColored(age_color(age), "%s", format_duration(age).c_str());
            ImGui::EndTable();
        }
    } else {
        ImGui::TextDisabled("No BaseStatus received (optional message).");
    }

    ImGui::SeparatorText("Decoder");
    const FleetModel::Counters& c = fleet_.counters();
    if (begin_kv("##counters")) {
        kv_u64("Frames decoded", c.frames);
        kv_u64("Decode errors", c.decode_errors, true);
        kv_u64("Unknown types", c.unknown_types, true);
        kv_u64("Land frames heard", c.land_frames);
        kv_u64("Orphan RxInfo", c.orphan_rx_info, true);
        kv_u64("Invalid boat ids", c.invalid_ids, true);
        ImGui::EndTable();
    }
    if (wide) ImGui::EndTable();

    ImGui::SeparatorText("Sequence loss per boat and message type");
    const ImGuiTableFlags tf = ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV |
                               ImGuiTableFlags_BordersOuterH;
    if (ImGui::BeginTable("##seqall", 7, tf)) {
        ImGui::TableSetupColumn("Boat");
        ImGui::TableSetupColumn("Type");
        ImGui::TableSetupColumn("Received");
        ImGui::TableSetupColumn("Lost");
        ImGui::TableSetupColumn("Loss");
        ImGui::TableSetupColumn("Duplicates");
        ImGui::TableSetupColumn("Restarts");
        ImGui::TableHeadersRow();
        for (const auto& [id, b] : fleet_.boats()) {
            for (const auto& [type, sq] : b.seq) {
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::TextColored(boat_color(id), "%s", boat_label(id).c_str());
                ImGui::TableNextColumn();
                ImGui::TextUnformatted(msg_type_name(type));
                ImGui::TableNextColumn();
                ImGui::Text("%llu", static_cast<unsigned long long>(sq.received));
                ImGui::TableNextColumn();
                ImGui::Text("%llu", static_cast<unsigned long long>(sq.lost));
                ImGui::TableNextColumn();
                const double loss = sq.loss_ratio() * 100.0;
                ImGui::TextColored(loss >= 30 ? colors::danger : loss >= 10 ? colors::warn : ImGui::GetStyleColorVec4(ImGuiCol_Text),
                                   "%.1f %%", loss);
                ImGui::TableNextColumn();
                ImGui::Text("%llu", static_cast<unsigned long long>(sq.duplicates));
                ImGui::TableNextColumn();
                ImGui::Text("%llu", static_cast<unsigned long long>(sq.restarts));
            }
        }
        ImGui::EndTable();
    }
    ImGui::End();
}

}  // namespace basestation::app
