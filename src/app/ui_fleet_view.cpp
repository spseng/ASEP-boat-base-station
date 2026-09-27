// Fleet view: a local east/north map in metres with each boat's trail,
// position, heading and state. Click a boat to select it.

#include "app.h"

#include <boat_defs/ids.h>
#include <boat_defs/mode.h>

#include <implot.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

namespace basestation::app {

namespace {

constexpr double PI = 3.14159265358979323846;

struct TrailGetterCtx {
    const std::deque<TrailPoint>* trail;
    const geo::LocalFrame* frame;
};

ImPlotPoint trail_getter(int idx, void* data) {
    const auto* ctx = static_cast<const TrailGetterCtx*>(data);
    const TrailPoint& p = (*ctx->trail)[static_cast<size_t>(idx)];
    const geo::NorthEast ne = ctx->frame->to_local({p.lat_deg, p.lon_deg});
    return ImPlotPoint(ne.east_m, ne.north_m);
}

struct Bounds {
    double min_x = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    bool valid() const { return min_x <= max_x; }
    void add(double x, double y) {
        min_x = std::min(min_x, x);
        max_x = std::max(max_x, x);
        min_y = std::min(min_y, y);
        max_y = std::max(max_y, y);
    }
};

// Outlined text so labels stay readable over trails and grid lines.
void outlined_text(ImDrawList* dl, ImVec2 p, ImU32 col, const char* text) {
    const ImU32 shadow = IM_COL32(0, 0, 0, 200);
    dl->AddText(ImVec2(p.x + 1, p.y + 1), shadow, text);
    dl->AddText(ImVec2(p.x - 1, p.y + 1), shadow, text);
    dl->AddText(p, col, text);
}

void dashed_circle(ImDrawList* dl, ImVec2 c, float r, ImU32 col, float thickness, int dashes) {
    const float step = static_cast<float>(2.0 * PI) / static_cast<float>(dashes);
    for (int i = 0; i < dashes; ++i) {
        const float a0 = step * static_cast<float>(i);
        dl->PathArcTo(c, r, a0, a0 + step * 0.55f, 6);
        dl->PathStroke(col, ImDrawFlags_None, thickness);
    }
}

bool has_position(const BoatState& b) {
    return b.self_status && (b.self_status->self.lat != 0 || b.self_status->self.lon != 0);
}

}  // namespace

void App::draw_fleet_view() {
    if (!settings_.show_fleet) return;
    if (!ImGui::Begin("Fleet", &settings_.show_fleet)) {
        ImGui::End();
        return;
    }
    const float s = ImGui::GetFontSize() / 16.0f;  // pixel sizes follow text size
    const auto& boats = fleet_.boats();

    // ---- controls ----
    if (ImGui::Button("Fit")) map_fit_once_ = true;
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_ForTooltip)) ImGui::SetTooltip("Zoom to show every boat and trail");
    ImGui::SameLine();
    ImGui::BeginDisabled(!local_);
    if (ImGui::Button("Re-center")) map_recenter_ = true;
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_ForTooltip))
        ImGui::SetTooltip("Move the map origin to the current fleet centroid (keeps the zoom)");
    ImGui::SameLine();
    ImGui::Checkbox("Auto-fit", &map_auto_fit_);
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_ForTooltip))
        ImGui::SetTooltip("Keep everything in view. Turns off when you pan or zoom.");
    ImGui::SameLine();
    ImGui::Checkbox("Trails", &map_show_trails_);
    ImGui::SameLine();
    ImGui::Checkbox("Labels", &map_show_labels_);
    if (local_) {
        ImGui::SameLine();
        const geo::LatLon o = local_->origin();
        char buf[96];
        std::snprintf(buf, sizeof buf, "origin %.6f, %.6f", o.lat_deg, o.lon_deg);
        ImGui::SameLine(std::max(ImGui::GetCursorPosX(),
                                 ImGui::GetWindowContentRegionMax().x - ImGui::CalcTextSize(buf).x));
        ImGui::AlignTextToFramePadding();
        ImGui::TextDisabled("%s", buf);
    }

    // ---- positions in the local frame ----
    struct Marker {
        uint8_t id;
        double e, n;
        const BoatState* b;
    };
    std::vector<Marker> markers;
    if (local_) {
        for (const auto& [id, b] : boats) {
            if (!has_position(b)) continue;
            const geo::NorthEast ne =
                local_->to_local(geo::from_e7(b.self_status->self.lat, b.self_status->self.lon));
            markers.push_back({id, ne.east_m, ne.north_m, &b});
        }
    }

    if (map_recenter_ && !markers.empty()) {
        // Average in geodetic coordinates: fine at fleet scale.
        double lat = 0, lon = 0;
        for (const Marker& m : markers) {
            const geo::LatLon ll = geo::from_e7(m.b->self_status->self.lat, m.b->self_status->self.lon);
            lat += ll.lat_deg;
            lon += ll.lon_deg;
        }
        local_ = geo::LocalFrame({lat / static_cast<double>(markers.size()), lon / static_cast<double>(markers.size())});
        for (Marker& m : markers) {
            const geo::NorthEast ne =
                local_->to_local(geo::from_e7(m.b->self_status->self.lat, m.b->self_status->self.lon));
            m.e = ne.east_m;
            m.n = ne.north_m;
        }
    }

    // ---- view limits ----
    bool set_limits = false;
    double x0 = -60, x1 = 60, y0 = -60, y1 = 60;
    if (map_recenter_) {
        const double hx = map_span_x_ / 2, hy = map_span_y_ / 2;
        x0 = -hx; x1 = hx; y0 = -hy; y1 = hy;
        set_limits = true;
        map_auto_fit_ = false;
    } else if ((map_auto_fit_ || map_fit_once_) && local_) {
        Bounds bb;
        for (const Marker& m : markers) bb.add(m.e, m.n);
        if (map_show_trails_) {
            for (const auto& [id, b] : boats) {
                for (const TrailPoint& p : b.trail) {
                    const geo::NorthEast ne = local_->to_local({p.lat_deg, p.lon_deg});
                    bb.add(ne.east_m, ne.north_m);
                }
            }
        }
        if (bb.valid()) {
            // Generous padding (labels sit right of the markers) and never
            // closer than ~40 m across.
            const double cx = (bb.min_x + bb.max_x) / 2, cy = (bb.min_y + bb.max_y) / 2;
            const double span = std::max(bb.max_x - bb.min_x, bb.max_y - bb.min_y);
            const double hx = std::max(20.0, (bb.max_x - bb.min_x) * 0.5 + span * 0.22 + 5.0);
            const double hy = std::max(20.0, (bb.max_y - bb.min_y) * 0.5 + span * 0.12 + 5.0);
            x0 = cx - hx; x1 = cx + hx; y0 = cy - hy; y1 = cy + hy;
            set_limits = true;
        }
    }
    map_recenter_ = false;
    map_fit_once_ = false;

    const ImPlotFlags flags = ImPlotFlags_Equal | ImPlotFlags_NoLegend | ImPlotFlags_NoBoxSelect;
    if (ImPlot::BeginPlot("##fleetmap", ImVec2(-1, -1), flags)) {
        ImPlot::SetupAxes("East (m)", "North (m)");
        ImPlot::SetupAxesLimits(x0, x1, y0, y1, set_limits ? ImPlotCond_Always : ImPlotCond_Once);
        ImPlot::SetupFinish();

        // Stop auto-fitting as soon as the operator takes over the view.
        const ImGuiIO& io = ImGui::GetIO();
        if (ImPlot::IsPlotHovered() && (ImGui::IsMouseDragging(ImGuiMouseButton_Left) || io.MouseWheel != 0.0f))
            map_auto_fit_ = false;

        // Trails.
        if (local_ && map_show_trails_) {
            for (const auto& [id, b] : boats) {
                if (b.trail.size() < 2) continue;
                TrailGetterCtx ctx{&b.trail, &*local_};
                const float alpha = is_stale(b) ? 0.30f : 0.75f;
                ImPlotSpec spec;
                ImVec4 c = boat_color(id);
                c.w = alpha;
                spec.LineColor = c;
                spec.LineWeight = (selected_ == id ? 2.5f : 1.6f) * s;
                char label[16];
                std::snprintf(label, sizeof label, "##trail%u", unsigned(id));
                ImPlot::PlotLineG(label, trail_getter, &ctx, static_cast<int>(b.trail.size()), spec);
            }
        }

        // Markers, drawn on top with the draw list for full control.
        ImPlot::PushPlotClipRect();
        ImDrawList* dl = ImPlot::GetPlotDrawList();

        // Origin crosshair.
        if (local_) {
            const ImVec2 o = ImPlot::PlotToPixels(0.0, 0.0);
            const ImU32 oc = IM_COL32(150, 155, 165, 120);
            dl->AddLine(ImVec2(o.x - 6 * s, o.y), ImVec2(o.x + 6 * s, o.y), oc, 1.0f);
            dl->AddLine(ImVec2(o.x, o.y - 6 * s), ImVec2(o.x, o.y + 6 * s), oc, 1.0f);
        }

        const ImVec2 mouse = ImGui::GetMousePos();
        const Marker* hovered = nullptr;
        float hovered_d2 = (16 * s) * (16 * s);
        for (const Marker& m : markers) {
            const BoatState& b = *m.b;
            const bool stale = is_stale(b);
            const float alpha = stale ? 0.40f : 1.0f;
            const ImVec2 p = ImPlot::PlotToPixels(m.e, m.n);
            const ImU32 col = boat_color_u32(m.id, alpha);

            const float dx = mouse.x - p.x, dy = mouse.y - p.y;
            if (ImPlot::IsPlotHovered() && dx * dx + dy * dy < hovered_d2) {
                hovered_d2 = dx * dx + dy * dy;
                hovered = &m;
            }

            // Teleop target: dashed ring, green while driving.
            const uint8_t tt = teleop_target_;
            if (tt != 0 && (tt == m.id || tt == boat::ids::BROADCAST_ID)) {
                const bool driving = teleop_status_.enabled && teleop_status_.last.driving;
                const ImVec4 rc = !teleop_status_.enabled ? colors::muted : driving ? colors::ok : colors::warn;
                dashed_circle(dl, p, 17 * s, ImGui::GetColorU32(rc), 2.0f * s, 10);
            }
            // Selection: solid white ring.
            if (selected_ == m.id) dl->AddCircle(p, 12 * s, IM_COL32(255, 255, 255, 230), 0, 2.0f * s);

            // Heading arrow.
            const float hdg = b.heading_deg();
            if (std::isfinite(hdg)) {
                const float a = hdg * static_cast<float>(PI / 180.0);
                const ImVec2 dir(std::sin(a), -std::cos(a));  // north up, pixels y down
                const ImVec2 perp(-dir.y, dir.x);
                const float len = 26 * s;
                const ImVec2 tip(p.x + dir.x * len, p.y + dir.y * len);
                dl->AddLine(p, tip, col, 2.0f * s);
                const float ah = 7 * s, aw = 4.5f * s;
                const ImVec2 base(tip.x - dir.x * ah, tip.y - dir.y * ah);
                dl->AddTriangleFilled(tip, ImVec2(base.x + perp.x * aw, base.y + perp.y * aw),
                                      ImVec2(base.x - perp.x * aw, base.y - perp.y * aw), col);
            }

            // Position: filled dot; hollow when the boat's own fix is old.
            const bool old_fix = b.self_status->self.age_ms > 3000;
            if (old_fix) {
                dl->AddCircleFilled(p, 6.5f * s, IM_COL32(20, 22, 26, 230));
                dl->AddCircle(p, 6.5f * s, col, 0, 2.0f * s);
            } else {
                dl->AddCircleFilled(p, 6.5f * s, col);
                dl->AddCircle(p, 6.5f * s, IM_COL32(0, 0, 0, 200), 0, 1.5f * s);
            }

            if (map_show_labels_ || selected_ == m.id) {
                // Right of the rings, so selection / teleop marks never cover it.
                const ImVec2 lp(p.x + 20 * s, p.y - 17 * s);
                const std::string label = boat_label(m.id);
                outlined_text(dl, lp, col, label.c_str());
                float y = lp.y + ImGui::GetFontSize();
                auto line = [&](const char* text, const ImVec4& c) {
                    ImVec4 cc = c;
                    cc.w *= alpha;
                    outlined_text(dl, ImVec2(lp.x, y), ImGui::GetColorU32(cc), text);
                    y += ImGui::GetFontSize();
                };
                if (stale) line("stale", colors::warn);
                if (b.status) {
                    if (b.status->gate_state == static_cast<uint8_t>(boat::mode::GateState::TRIPPED))
                        line("TRIPPED", colors::danger);
                    if (b.status->fault_flags) line(names::fault_list(b.status->fault_flags).c_str(), colors::warn);
                }
                if (old_fix) line("old fix", colors::warn);
            }
        }
        ImPlot::PopPlotClipRect();

        if (hovered) {
            const BoatState& b = *hovered->b;
            if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) select_boat(hovered->id);
            ImGui::BeginTooltip();
            ImGui::TextColored(boat_color(hovered->id), "Boat %u", unsigned(hovered->id));
            if (b.status) {
                const auto& st = *b.status;
                const bool known = names::find_mode(st.mode) != nullptr;
                ImGui::Text("Mode:");
                ImGui::SameLine();
                ImGui::TextColored(known ? ImGui::GetStyleColorVec4(ImGuiCol_Text) : colors::warn, "%s",
                                   names::mode_name(st.mode).c_str());
                ImGui::Text("Armed: %s", names::armed_name(st.armed).c_str());
                const bool tripped = st.gate_state == static_cast<uint8_t>(boat::mode::GateState::TRIPPED);
                ImGui::Text("Gate:");
                ImGui::SameLine();
                ImGui::TextColored(tripped ? colors::danger : colors::ok, "%s",
                                   names::gate_name(st.gate_state).c_str());
                if (st.fault_flags)
                    ImGui::TextColored(colors::warn, "Faults: %s", names::fault_list(st.fault_flags).c_str());
            } else {
                ImGui::TextDisabled("No Status received yet");
            }
            ImGui::Text("Scalar: %.2f °C", static_cast<double>(b.self_status->self.scalar));
            ImGui::Text("Position: %.1f m E, %.1f m N", hovered->e, hovered->n);
            const double age = age_s(b);
            ImGui::Text("Heard:");
            ImGui::SameLine();
            ImGui::TextColored(age_color(age), "%s ago", format_duration(age).c_str());
            ImGui::TextDisabled("Click to select");
            ImGui::EndTooltip();
        }

        const ImPlotRect lim = ImPlot::GetPlotLimits();
        map_span_x_ = lim.X.Size();
        map_span_y_ = lim.Y.Size();
        ImPlot::EndPlot();
    }

    if (!local_) {
        // Overlay a hint in the middle of the empty map.
        const char* msg = link_.is_open() ? "Waiting for boat positions..." : "Not connected";
        const ImVec2 ws = ImGui::GetWindowSize();
        const ImVec2 ts = ImGui::CalcTextSize(msg);
        ImGui::GetWindowDrawList()->AddText(
            ImVec2(ImGui::GetWindowPos().x + (ws.x - ts.x) / 2, ImGui::GetWindowPos().y + ws.y / 2),
            ImGui::GetColorU32(colors::muted), msg);
    }
    ImGui::End();
}

}  // namespace basestation::app
