// Time-series plots: scalar per boat, link quality in both directions.
// The three plots share one x range (seconds since the app started).

#include "app.h"

#include <implot.h>

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace basestation::app {

namespace {

ImPlotPoint sample_getter(int idx, void* data) {
    const auto* d = static_cast<const std::deque<Sample>*>(data);
    const Sample& s = (*d)[static_cast<size_t>(idx)];
    return ImPlotPoint(s.t, static_cast<double>(s.v));
}

// "m:ss" tick labels for the time axis.
int time_formatter(double value, char* buf, int size, void*) {
    const bool neg = value < 0;
    const long v = static_cast<long>(std::fabs(value) + 0.5);
    return std::snprintf(buf, static_cast<size_t>(size), "%s%ld:%02ld", neg ? "-" : "", v / 60, v % 60);
}

void plot_series(const char* label, const std::deque<Sample>& data, const ImVec4& color, float weight,
                 ImPlotMarker marker = ImPlotMarker_None) {
    if (data.empty()) return;
    ImPlotSpec spec;
    spec.LineColor = color;
    spec.LineWeight = weight;
    spec.Marker = marker;
    spec.MarkerSize = 2.5f;
    spec.MarkerFillColor = color;
    // The getter only reads; ImPlot's API takes void*.
    ImPlot::PlotLineG(label, sample_getter, const_cast<std::deque<Sample>*>(&data), static_cast<int>(data.size()),
                      spec);
}

}  // namespace

void App::draw_plots() {
    if (!settings_.show_plots) return;
    if (!ImGui::Begin("Plots", &settings_.show_plots)) {
        ImGui::End();
        return;
    }
    const float em = ImGui::GetFontSize();
    const float s = em / 16.0f;

    ImGui::Checkbox("Follow", &plot_follow_);
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_ForTooltip))
        ImGui::SetTooltip("Scroll with time. Turns off when you pan or zoom a plot.");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(em * 12);
    float window_min = settings_.plot_window_s / 60.0f;
    if (ImGui::SliderFloat("Window", &window_min, 0.5f, 15.0f, "%.1f min", ImGuiSliderFlags_Logarithmic))
        settings_.plot_window_s = std::round(window_min * 60.0f);
    ImGui::SameLine();
    ImGui::TextDisabled("x: time since start (m:ss)");

    const double now_s = fleet_.seconds(Clock::now());
    if (plot_follow_) {
        // Until a full window has elapsed, show 0..now rather than empty
        // negative time.
        plot_x_max_ = std::max(now_s + 1.0, 10.0);
        plot_x_min_ = std::max(0.0, plot_x_max_ - settings_.plot_window_s);
    }

    const auto& boats = fleet_.boats();
    const float h = (ImGui::GetContentRegionAvail().y - ImGui::GetStyle().ItemSpacing.y * 2) / 3.0f;
    const ImPlotFlags pf = ImPlotFlags_NoMenus | ImPlotFlags_NoBoxSelect;
    const ImPlotAxisFlags yf = ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit;
    bool user_moved = false;

    auto setup = [&](const char* ylabel, bool last) {
        ImPlot::SetupAxis(ImAxis_X1, nullptr, last ? ImPlotAxisFlags_None : ImPlotAxisFlags_NoTickLabels);
        ImPlot::SetupAxisFormat(ImAxis_X1, time_formatter);
        ImPlot::SetupAxisLinks(ImAxis_X1, &plot_x_min_, &plot_x_max_);
        ImPlot::SetupAxis(ImAxis_Y1, ylabel, yf);
        ImPlot::SetupLegend(ImPlotLocation_NorthWest, ImPlotLegendFlags_Horizontal);
        ImPlot::SetupFinish();
        const ImGuiIO& io = ImGui::GetIO();
        if (ImPlot::IsPlotHovered() && (ImGui::IsMouseDragging(ImGuiMouseButton_Left) || io.MouseWheel != 0.0f))
            user_moved = true;
    };

    if (ImPlot::BeginAlignedPlots("##plots")) {
        if (ImPlot::BeginPlot("Scalar", ImVec2(-1, h), pf)) {
            setup("°C", false);
            for (const auto& [id, b] : boats) {
                const std::string label = boat_label(id);
                plot_series(label.c_str(), b.scalar_history, boat_color(id), 2.0f * s);
            }
            ImPlot::EndPlot();
        }
        if (ImPlot::BeginPlot("RSSI (solid: base hears boat, dots: boat hears base)", ImVec2(-1, h), pf)) {
            setup("dBm", false);
            for (const auto& [id, b] : boats) {
                const std::string label = boat_label(id);
                plot_series(label.c_str(), b.rx_rssi_history, boat_color(id), 2.0f * s);
                ImVec4 c = boat_color(id);
                c.w = 0.65f;
                const std::string gs = label + " at boat";
                plot_series(gs.c_str(), b.gs_rssi_history, c, 1.0f * s, ImPlotMarker_Circle);
            }
            ImPlot::EndPlot();
        }
        if (ImPlot::BeginPlot("SNR at base", ImVec2(-1, h), pf)) {
            setup("dB", true);
            for (const auto& [id, b] : boats) {
                const std::string label = boat_label(id);
                plot_series(label.c_str(), b.rx_snr_history, boat_color(id), 2.0f * s);
            }
            ImPlot::EndPlot();
        }
        ImPlot::EndAlignedPlots();
    }
    if (user_moved) plot_follow_ = false;
    ImGui::End();
}

}  // namespace basestation::app
