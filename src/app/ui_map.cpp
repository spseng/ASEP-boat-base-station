// Map tiles and the base-station position: the Map... and Base... popups of
// the Fleet view, the "Download map area" window, and the logic that keeps
// the base position and the view origin up to date.

#include "app.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace basestation::app {

namespace {

// A base GPS fix is re-cached only after it moves this far, so GPS jitter
// does not rewrite the settings file every second.
constexpr double GPS_CACHE_MIN_MOVE_M = 5.0;
// With "Use base station as map origin", the origin follows the base only
// when it moves more than this (GPS jitter would shake the view otherwise).
constexpr double ORIGIN_FOLLOW_M = 3.0;

double distance_between(geo::LatLon a, geo::LatLon b) {
    return geo::distance_m({}, geo::LocalFrame(a).to_local(b));
}

std::string format_bytes(uint64_t b) {
    char buf[32];
    if (b < 1024 * 1024) std::snprintf(buf, sizeof buf, "%.0f KB", static_cast<double>(b) / 1024.0);
    else std::snprintf(buf, sizeof buf, "%.1f MB", static_cast<double>(b) / (1024.0 * 1024.0));
    return buf;
}

void copy_to(std::array<char, 512>& buf, const std::string& s) {
    std::snprintf(buf.data(), buf.size(), "%s", s.c_str());
}

}  // namespace

// ---------------------------------------------------------------------------
// Base station position and view origin
// ---------------------------------------------------------------------------

void App::update_base_position() {
    BaseSettings& bs = settings_.base;
    std::optional<geo::LatLon> manual, cached;
    if (bs.manual_set) manual = geo::LatLon{bs.lat, bs.lon};
    if (bs.gps_cached) cached = geo::LatLon{bs.gps_lat, bs.gps_lon};
    base_pos_ = choose_base_position(fleet_.base_station(), Clock::now(), manual, bs.pin_manual, cached);

    // Cache live fixes so the next session knows where the base was.
    if (fleet_.base_station().has_fix()) {
        const base::BasePosition& p = *fleet_.base_station().position;
        const geo::LatLon fix = geo::from_e7(p.lat, p.lon);
        if (!bs.gps_cached || distance_between({bs.gps_lat, bs.gps_lon}, fix) > GPS_CACHE_MIN_MOVE_M) {
            bs.gps_cached = true;
            bs.gps_lat = fix.lat_deg;
            bs.gps_lon = fix.lon_deg;
        }
    }

    // The map needs an origin before any boat is heard; the base station is
    // the natural one. With "as origin", it follows the base.
    if (!base_pos_.known() || base_dragging_) return;
    if (settings_.base.as_origin) {
        if (!local_ || distance_between(local_->origin(), base_pos_.pos) > ORIGIN_FOLLOW_M)
            set_map_origin(base_pos_.pos);
    } else if (!local_) {
        set_map_origin(base_pos_.pos);
    }
}

void App::set_map_origin(geo::LatLon origin) {
    // Keep looking at the same place: convert the view centre to the new frame.
    if (local_ && map_view_valid_) {
        const geo::LatLon c = local_->to_geo({map_center_y_, map_center_x_});
        const geo::NorthEast ne = geo::LocalFrame(origin).to_local(c);
        map_pending_center_ = std::make_pair(ne.east_m, ne.north_m);
    }
    local_ = geo::LocalFrame(origin);
}

void App::set_manual_base(geo::LatLon p) {
    settings_.base.manual_set = true;
    settings_.base.lat = p.lat_deg;
    settings_.base.lon = p.lon_deg;
    base_edit_lat_ = p.lat_deg;
    base_edit_lon_ = p.lon_deg;
    // base_pos_ (and possibly the view origin) follow in the next frame's
    // update_link(), not in the middle of drawing the plot.
}

// ---------------------------------------------------------------------------
// Map source and cache
// ---------------------------------------------------------------------------

tiles::TileSource App::map_source() const {
    if (settings_.map.source == tiles::CUSTOM_SOURCE_ID) return tiles::custom_source(settings_.map.custom_url);
    if (const tiles::TileSource* s = tiles::find_builtin_source(settings_.map.source)) return *s;
    return tiles::builtin_sources().front();
}

std::string App::map_source_problem() const {
    if (settings_.map.source != tiles::CUSTOM_SOURCE_ID) return {};
    if (settings_.map.custom_url.empty()) return "Enter a tile URL";
    return tiles::check_url_template(settings_.map.custom_url);
}

std::string App::effective_tile_cache_dir() const {
    if (!opts_.tile_cache.empty()) return opts_.tile_cache;
    if (!settings_.map.cache_dir.empty()) return settings_.map.cache_dir;
    return pref_dir_.empty() ? std::string("tiles") : pref_dir_ + "tiles";
}

void App::open_map_layer() {
    const std::string dir = effective_tile_cache_dir();
    map_.reset();  // joins the old threads before new ones use the directory
    map_ = std::make_unique<MapLayer>(renderer_, dir, tiles::user_agent(BASESTATION_VERSION), settings_.map.offline);
    map_cache_dir_ = dir;
    copy_to(map_cache_buf_, settings_.map.cache_dir);
}

// ---------------------------------------------------------------------------
// Popups
// ---------------------------------------------------------------------------

void App::draw_map_controls() {
    MapSettings& m = settings_.map;
    tiles::TileService& svc = map_->service();
    const float field_w = ImGui::GetFontSize() * 22.0f;

    ImGui::Checkbox("Show map", &m.enabled);

    // Source: built-ins, then Custom.
    const auto& sources = tiles::builtin_sources();
    std::string preview = m.source == tiles::CUSTOM_SOURCE_ID ? "Custom" : map_source().name;
    ImGui::SetNextItemWidth(field_w);
    if (ImGui::BeginCombo("Source", preview.c_str())) {
        for (const tiles::TileSource& s : sources) {
            if (ImGui::Selectable(s.name.c_str(), m.source == s.id)) m.source = s.id;
        }
        if (ImGui::Selectable("Custom (URL template)", m.source == tiles::CUSTOM_SOURCE_ID)) {
            m.source = tiles::CUSTOM_SOURCE_ID;
        }
        ImGui::EndCombo();
    }
    if (m.source == tiles::CUSTOM_SOURCE_ID) {
        ImGui::SetNextItemWidth(field_w);
        if (ImGui::InputTextWithHint("URL", "http://localhost:8080/{z}/{x}/{y}.png", map_url_buf_.data(),
                                     map_url_buf_.size()))
            m.custom_url = map_url_buf_.data();
        help_marker("Any XYZ tile server, e.g. one you run on the laptop at the lake. {z} {x} {y} are "
                    "replaced by the zoom and tile numbers. PNG or JPEG tiles.");
    }
    const std::string problem = map_source_problem();
    if (!problem.empty()) ImGui::TextColored(colors::warn, "%s", problem.c_str());

    ImGui::SetNextItemWidth(field_w);
    ImGui::SliderFloat("Opacity", &m.opacity, 0.05f, 1.0f, "%.2f");

    const tiles::TileService::NetStatus net = svc.net_status();
    if (ImGui::Checkbox("Offline", &m.offline)) svc.set_offline(m.offline);
    help_marker("Never use the network: show only tiles already in the cache. Turn this on at the lake "
                "if the laptop has no internet (or a slow, metered one).");
    if (!net.downloads_available)
        ImGui::TextColored(colors::warn, "Built without libcurl: cached tiles only, no downloads.");

    if (ImGui::Button("Download area...")) open_download_window(map_view_valid_ && local_ ? 0 : 1, std::nullopt);
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_ForTooltip))
        ImGui::SetTooltip("Save the tiles of an area to the cache before going somewhere without internet");

    if (m.enabled && map_zoom_ >= 0) ImGui::TextDisabled("Showing zoom %d, %zu tiles in memory", map_zoom_, map_->texture_count());
    if (net.downloaded || net.failed)
        ImGui::TextDisabled("This session: %llu downloaded, %llu failed", static_cast<unsigned long long>(net.downloaded),
                            static_cast<unsigned long long>(net.failed));
    if (net.paused_s > 0) ImGui::TextColored(colors::warn, "Downloads paused for %.0f s after errors", net.paused_s);
    if (!net.last_error.empty()) {
        ImGui::PushTextWrapPos(ImGui::GetCursorPosX() + field_w * 1.3f);
        ImGui::TextDisabled("Last error: %s", net.last_error.c_str());
        ImGui::PopTextWrapPos();
    }

    ImGui::SeparatorText("Cache");
    const tiles::TileService::CacheStats cs = svc.cache_stats();
    if (cs.scanning) ImGui::TextDisabled("Counting...");
    else if (cs.valid) ImGui::Text("%s in %llu tiles", format_bytes(cs.bytes).c_str(), static_cast<unsigned long long>(cs.files));
    ImGui::SameLine();
    if (ImGui::SmallButton("Rescan")) svc.rescan_cache();
    ImGui::SameLine();
    if (ImGui::SmallButton("Clear cache...")) map_confirm_clear_ = true;
    if (map_confirm_clear_) {
        ImGui::TextColored(colors::warn, "Delete every cached tile? Offline maps are lost.");
        if (ImGui::Button("Delete")) {
            svc.clear_cache();
            map_->clear_textures();
            map_confirm_clear_ = false;
        }
        ImGui::SameLine();
        if (ImGui::Button("Keep")) map_confirm_clear_ = false;
    }
    ImGui::TextDisabled("%s", map_cache_dir_.c_str());
    if (!opts_.tile_cache.empty()) {
        ImGui::TextDisabled("(set by --tile-cache for this session)");
    } else {
        ImGui::SetNextItemWidth(field_w);
        ImGui::InputTextWithHint("##cachedir", "folder (empty = default)", map_cache_buf_.data(), map_cache_buf_.size());
        ImGui::SameLine();
        if (ImGui::Button("Use folder")) {
            m.cache_dir = map_cache_buf_.data();
            open_map_layer();
        }
        help_marker("Where tiles are stored, e.g. on a USB stick shared by several laptops. Empty = the "
                    "app's data folder.");
    }
}

void App::draw_base_controls() {
    BaseSettings& bs = settings_.base;
    if (!base_edit_init_) {
        base_edit_lat_ = bs.manual_set ? bs.lat : base_pos_.pos.lat_deg;
        base_edit_lon_ = bs.manual_set ? bs.lon : base_pos_.pos.lon_deg;
        base_edit_init_ = true;
    }
    if (base_pos_.known()) {
        ImGui::Text("Base station: %.7f, %.7f", base_pos_.pos.lat_deg, base_pos_.pos.lon_deg);
        ImGui::TextDisabled("Source: %s", to_string(base_pos_.source));
    } else {
        ImGui::TextColored(colors::warn, "Base station position not set");
    }

    // Future hardware: a GPS on the base ESP32 (BasePosition).
    const BaseStationState& st = fleet_.base_station();
    if (st.position) {
        const double age = std::chrono::duration<double>(Clock::now() - st.position_time).count();
        if (st.has_fix())
            ImGui::TextColored(age < 10 ? colors::ok : colors::warn, "Base GPS: fix, %u satellites, %s ago",
                               unsigned(st.position->satellites), format_duration(age).c_str());
        else
            ImGui::TextColored(colors::warn, "Base GPS: no fix (%u satellites)", unsigned(st.position->satellites));
    } else {
        ImGui::TextDisabled("Base GPS: none reported");
    }

    ImGui::SeparatorText("Set by hand");
    const float w = ImGui::GetFontSize() * 9.0f;
    ImGui::SetNextItemWidth(w);
    ImGui::InputDouble("Lat", &base_edit_lat_, 0, 0, "%.7f");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(w);
    ImGui::InputDouble("Lon", &base_edit_lon_, 0, 0, "%.7f");
    ImGui::SameLine();
    const geo::LatLon edit{base_edit_lat_, base_edit_lon_};
    const bool edit_ok = plausible_position(edit);
    ImGui::BeginDisabled(!edit_ok);
    if (ImGui::Button("Set")) set_manual_base(edit);
    ImGui::EndDisabled();

    ImGui::BeginDisabled(!local_);
    if (ImGui::Button(base_place_mode_ ? "Done placing" : "Set from map")) {
        base_place_mode_ = !base_place_mode_;
        if (base_place_mode_) ImGui::CloseCurrentPopup();
    }
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_ForTooltip | ImGuiHoveredFlags_AllowWhenDisabled))
        ImGui::SetTooltip("Click the Fleet view to place the base station, or drag its marker. "
                          "Right-click the map for 'Set base station here'.");
    ImGui::SameLine();
    ImGui::BeginDisabled(!local_ || !map_view_valid_);
    if (ImGui::Button("Use view centre")) set_manual_base(local_->to_geo({map_center_y_, map_center_x_}));
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::BeginDisabled(!bs.manual_set);
    if (ImGui::Button("Clear")) {
        bs.manual_set = false;
        update_base_position();
    }
    ImGui::EndDisabled();
    ImGui::BeginDisabled(!(base_pos_.source == BasePositionChoice::Source::Gps));
    if (ImGui::Button("Keep GPS position as manual")) set_manual_base(base_pos_.pos);
    ImGui::EndDisabled();

    ImGui::Separator();
    ImGui::Checkbox("Prefer manual position over base GPS", &bs.pin_manual);
    help_marker("When the base ESP32 reports a GPS fix it normally wins over the position set by hand. "
                "Tick this to keep using the manual one (e.g. the GPS antenna is not where the radio is).");
    ImGui::Checkbox("Use base station as map origin", &bs.as_origin);
    help_marker("The Fleet view's 0,0 is the base station, the same in every session. Otherwise it is "
                "wherever the first position came from.");
}

// ---------------------------------------------------------------------------
// Download map area
// ---------------------------------------------------------------------------

void App::open_download_window(int area_mode, std::optional<geo::LatLon> point) {
    const tiles::TileSource src = map_source();
    dl_area_mode_ = area_mode;
    if (point) {
        dl_lat_ = point->lat_deg;
        dl_lon_ = point->lon_deg;
    } else if (base_pos_.known()) {
        dl_lat_ = base_pos_.pos.lat_deg;
        dl_lon_ = base_pos_.pos.lon_deg;
    } else if (local_) {
        dl_lat_ = local_->origin().lat_deg;
        dl_lon_ = local_->origin().lon_deg;
    }
    if (dl_area_mode_ == 1 && !base_pos_.known()) dl_area_mode_ = 2;
    dl_zmax_ = src.max_zoom;
    dl_zmin_ = std::clamp(map_zoom_ >= 0 ? map_zoom_ : 14, 0, dl_zmax_);
    dl_error_.clear();
    map_download_open_ = true;
    ImGui::SetWindowFocus("Download map area");
}

void App::draw_map_download_window() {
    if (!map_download_open_) return;
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_FirstUseEver, ImVec2(0.5f, 0.5f));
    // Auto-sized: the content changes with the area mode and the progress.
    if (!ImGui::Begin("Download map area", &map_download_open_,
                      ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::End();
        return;
    }
    tiles::TileService& svc = map_->service();
    const tiles::TileSource src = map_source();
    const std::string problem = map_source_problem();
    ImGui::Text("Source: %s", src.name.c_str());
    if (!problem.empty()) ImGui::TextColored(colors::warn, "%s", problem.c_str());

    const bool have_view = local_ && map_view_valid_;
    ImGui::BeginDisabled(!have_view);
    ImGui::RadioButton("Current view", &dl_area_mode_, 0);
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::BeginDisabled(!base_pos_.known());
    ImGui::RadioButton("Around base station", &dl_area_mode_, 1);
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::RadioButton("Around a point", &dl_area_mode_, 2);
    if ((dl_area_mode_ == 0 && !have_view) || (dl_area_mode_ == 1 && !base_pos_.known())) dl_area_mode_ = 2;

    const float w = ImGui::GetFontSize() * 9.0f;
    if (dl_area_mode_ == 2) {
        ImGui::SetNextItemWidth(w);
        ImGui::InputDouble("Lat", &dl_lat_, 0, 0, "%.6f");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(w);
        ImGui::InputDouble("Lon", &dl_lon_, 0, 0, "%.6f");
    }
    if (dl_area_mode_ != 0) {
        ImGui::SetNextItemWidth(w);
        ImGui::DragFloat("Radius (m)", &dl_radius_m_, 10.0f, 50.0f, 20000.0f, "%.0f");
        dl_radius_m_ = std::clamp(dl_radius_m_, 50.0f, 20000.0f);
    }

    ImGui::SetNextItemWidth(w);
    ImGui::SliderInt("Min zoom", &dl_zmin_, 0, src.max_zoom);
    ImGui::SameLine();
    ImGui::SetNextItemWidth(w);
    ImGui::SliderInt("Max zoom", &dl_zmax_, 0, src.max_zoom);
    dl_zmin_ = std::clamp(dl_zmin_, 0, src.max_zoom);
    dl_zmax_ = std::clamp(dl_zmax_, dl_zmin_, src.max_zoom);
    if (map_zoom_ >= 0) ImGui::TextDisabled("The view is at zoom %d; each extra level is ~4x the tiles.", map_zoom_);

    tiles::GeoBox box{};
    if (dl_area_mode_ == 0 && have_view) {
        const geo::LatLon sw = local_->to_geo({map_center_y_ - map_span_y_ / 2, map_center_x_ - map_span_x_ / 2});
        const geo::LatLon ne = local_->to_geo({map_center_y_ + map_span_y_ / 2, map_center_x_ + map_span_x_ / 2});
        box = tiles::GeoBox{sw.lat_deg, sw.lon_deg, ne.lat_deg, ne.lon_deg};
    } else {
        const geo::LatLon c = dl_area_mode_ == 1 ? base_pos_.pos : geo::LatLon{dl_lat_, dl_lon_};
        box = tiles::box_around(c, dl_radius_m_);
    }
    const bool box_ok = box.south >= -85 && box.north <= 85 && box.west >= -180 && box.east <= 180;
    const uint64_t count = box_ok ? tiles::count_tiles(box, dl_zmin_, dl_zmax_) : 0;
    const bool too_many = count > tiles::DEFAULT_PREFETCH_CAP;
    const double mb = static_cast<double>(count) * src.avg_tile_kb / 1024.0;
    if (!box_ok) {
        ImGui::TextColored(colors::warn, "Invalid position");
    } else if (too_many) {
        ImGui::TextColored(colors::danger, "%llu tiles (~%.0f MB): over the limit of %llu.",
                           static_cast<unsigned long long>(count), mb,
                           static_cast<unsigned long long>(tiles::DEFAULT_PREFETCH_CAP));
        ImGui::TextColored(colors::danger, "Make the area smaller or lower the max zoom.");
    } else {
        ImGui::Text("%llu tiles, about %.1f MB", static_cast<unsigned long long>(count), mb);
    }
    if (!src.usage_note.empty()) {
        ImGui::PushTextWrapPos(ImGui::GetCursorPosX() + ImGui::GetFontSize() * 28.0f);
        ImGui::TextDisabled("%s", src.usage_note.c_str());
        ImGui::PopTextWrapPos();
    }

    const tiles::TileService::PrefetchStatus pf = svc.prefetch_status();
    const tiles::TileService::NetStatus net = svc.net_status();
    if (pf.active) {
        const float frac = pf.total ? static_cast<float>(pf.processed()) / static_cast<float>(pf.total) : 0.0f;
        char overlay[64];
        std::snprintf(overlay, sizeof overlay, "%zu / %zu", pf.processed(), pf.total);
        ImGui::ProgressBar(frac, ImVec2(-1, 0), overlay);
        ImGui::TextDisabled("%zu downloaded, %zu already cached, %zu failed", pf.downloaded, pf.cached, pf.failed);
        if (net.paused_s > 0) ImGui::TextColored(colors::warn, "Paused %.0f s after errors", net.paused_s);
        if (ImGui::Button("Cancel")) svc.cancel_prefetch();
    } else {
        const bool can = box_ok && count > 0 && !too_many && problem.empty() && net.downloads_available &&
                         !settings_.map.offline;
        ImGui::BeginDisabled(!can);
        if (ImGui::Button("Download")) {
            const tiles::PrefetchPlan plan = tiles::plan_prefetch(box, dl_zmin_, dl_zmax_);
            dl_error_.clear();
            if (!plan.ok)
                dl_error_ = plan.error;
            else if (svc.start_prefetch(src, plan.tiles, &dl_error_))
                events_.info("Downloading " + std::to_string(plan.tiles.size()) + " map tiles (" + src.name + ")");
        }
        ImGui::EndDisabled();
        if (settings_.map.offline) {
            ImGui::SameLine();
            ImGui::TextColored(colors::warn, "Offline mode is on");
        } else if (!net.downloads_available) {
            ImGui::SameLine();
            ImGui::TextColored(colors::warn, "Built without libcurl");
        }
        if (!dl_error_.empty()) ImGui::TextColored(colors::danger, "%s", dl_error_.c_str());
        if (!pf.message.empty()) ImGui::TextDisabled("Last download: %s", pf.message.c_str());
    }
    ImGui::End();
}

}  // namespace basestation::app
