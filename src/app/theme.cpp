#include "theme.h"

#include <implot.h>

#include <sys/stat.h>

namespace basestation::app {

namespace {

bool file_exists(const char* path) {
    struct stat st {};
    return ::stat(path, &st) == 0 && S_ISREG(st.st_mode);
}

// First existing path wins. macOS paths first, then common Linux packages.
ImFont* add_first_font(const char* const* candidates, float size) {
    ImGuiIO& io = ImGui::GetIO();
    for (const char* const* p = candidates; *p; ++p) {
        if (!file_exists(*p)) continue;
        ImFontConfig cfg;
        cfg.Flags |= ImFontFlags_NoLoadError;
        if (ImFont* f = io.Fonts->AddFontFromFileTTF(*p, size, &cfg)) return f;
    }
    return nullptr;
}

}  // namespace

Fonts load_fonts(float base_size_px) {
    static const char* const regular[] = {
        "/System/Library/Fonts/Supplemental/Arial.ttf",
        "/Library/Fonts/Arial.ttf",
        "/usr/share/fonts/truetype/noto/NotoSans-Regular.ttf",
        "/usr/share/fonts/noto/NotoSans-Regular.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
        nullptr,
    };
    static const char* const bold[] = {
        "/System/Library/Fonts/Supplemental/Arial Bold.ttf",
        "/Library/Fonts/Arial Bold.ttf",
        "/usr/share/fonts/truetype/noto/NotoSans-Bold.ttf",
        "/usr/share/fonts/noto/NotoSans-Bold.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
        "/usr/share/fonts/dejavu/DejaVuSans-Bold.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf",
        nullptr,
    };
    static const char* const mono[] = {
        "/System/Library/Fonts/Menlo.ttc",
        "/usr/share/fonts/truetype/noto/NotoSansMono-Regular.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
        "/usr/share/fonts/dejavu/DejaVuSansMono.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationMono-Regular.ttf",
        nullptr,
    };

    Fonts f;
    f.regular = add_first_font(regular, base_size_px);
    if (!f.regular) {
        // Embedded, scalable, always available.
        ImFontConfig cfg;
        cfg.SizePixels = base_size_px;
        f.regular = ImGui::GetIO().Fonts->AddFontDefaultVector(&cfg);
    }
    f.bold = add_first_font(bold, base_size_px);
    if (!f.bold) f.bold = f.regular;
    f.mono = add_first_font(mono, base_size_px);
    if (!f.mono) f.mono = f.regular;
    ImGui::GetIO().FontDefault = f.regular;
    ImGui::GetStyle().FontSizeBase = base_size_px;
    return f;
}

void apply_theme(float ui_scale) {
    ImGuiStyle& style = ImGui::GetStyle();
    ImGui::StyleColorsDark(&style);

    style.WindowRounding = 6.0f;
    style.ChildRounding = 4.0f;
    style.FrameRounding = 4.0f;
    style.PopupRounding = 4.0f;
    style.GrabRounding = 4.0f;
    style.TabRounding = 4.0f;
    style.ScrollbarRounding = 6.0f;
    style.WindowPadding = ImVec2(10, 8);
    style.FramePadding = ImVec2(7, 4);
    style.ItemSpacing = ImVec2(8, 6);
    style.ItemInnerSpacing = ImVec2(6, 4);
    style.CellPadding = ImVec2(6, 3);
    style.ScrollbarSize = 13.0f;
    style.WindowBorderSize = 1.0f;
    style.FrameBorderSize = 0.0f;
    style.TabBarBorderSize = 1.0f;
    style.DockingSeparatorSize = 3.0f;

    ImVec4* c = style.Colors;
    const ImVec4 bg{0.095f, 0.105f, 0.12f, 1.0f};
    const ImVec4 panel{0.12f, 0.13f, 0.15f, 1.0f};
    const ImVec4 frame{0.18f, 0.20f, 0.23f, 1.0f};
    const ImVec4 frame_hi{0.24f, 0.27f, 0.31f, 1.0f};
    const ImVec4 accent{0.26f, 0.47f, 0.75f, 1.0f};
    const ImVec4 accent_hi{0.33f, 0.56f, 0.86f, 1.0f};

    c[ImGuiCol_Text] = ImVec4(0.90f, 0.92f, 0.94f, 1.0f);
    c[ImGuiCol_TextDisabled] = ImVec4(0.50f, 0.53f, 0.57f, 1.0f);
    c[ImGuiCol_WindowBg] = panel;
    c[ImGuiCol_ChildBg] = ImVec4(0, 0, 0, 0);
    c[ImGuiCol_PopupBg] = ImVec4(0.13f, 0.14f, 0.16f, 0.98f);
    c[ImGuiCol_Border] = ImVec4(0.25f, 0.27f, 0.30f, 0.8f);
    c[ImGuiCol_FrameBg] = frame;
    c[ImGuiCol_FrameBgHovered] = frame_hi;
    c[ImGuiCol_FrameBgActive] = ImVec4(0.28f, 0.32f, 0.37f, 1.0f);
    c[ImGuiCol_TitleBg] = bg;
    c[ImGuiCol_TitleBgActive] = ImVec4(0.14f, 0.16f, 0.19f, 1.0f);
    c[ImGuiCol_TitleBgCollapsed] = bg;
    c[ImGuiCol_MenuBarBg] = ImVec4(0.11f, 0.12f, 0.14f, 1.0f);
    c[ImGuiCol_ScrollbarBg] = ImVec4(0, 0, 0, 0);
    c[ImGuiCol_CheckMark] = accent_hi;
    c[ImGuiCol_SliderGrab] = accent;
    c[ImGuiCol_SliderGrabActive] = accent_hi;
    c[ImGuiCol_Button] = ImVec4(0.21f, 0.24f, 0.28f, 1.0f);
    c[ImGuiCol_ButtonHovered] = ImVec4(0.27f, 0.31f, 0.37f, 1.0f);
    c[ImGuiCol_ButtonActive] = ImVec4(0.31f, 0.36f, 0.43f, 1.0f);
    c[ImGuiCol_Header] = ImVec4(0.22f, 0.30f, 0.42f, 0.8f);
    c[ImGuiCol_HeaderHovered] = ImVec4(0.26f, 0.36f, 0.50f, 0.8f);
    c[ImGuiCol_HeaderActive] = ImVec4(0.28f, 0.40f, 0.56f, 1.0f);
    c[ImGuiCol_Separator] = ImVec4(0.25f, 0.27f, 0.30f, 1.0f);
    c[ImGuiCol_Tab] = ImVec4(0.14f, 0.15f, 0.17f, 1.0f);
    c[ImGuiCol_TabHovered] = ImVec4(0.26f, 0.36f, 0.50f, 1.0f);
    c[ImGuiCol_TabSelected] = ImVec4(0.20f, 0.26f, 0.35f, 1.0f);
    c[ImGuiCol_TabSelectedOverline] = accent_hi;
    c[ImGuiCol_TabDimmed] = ImVec4(0.12f, 0.13f, 0.15f, 1.0f);
    c[ImGuiCol_TabDimmedSelected] = ImVec4(0.17f, 0.20f, 0.25f, 1.0f);
    c[ImGuiCol_TabDimmedSelectedOverline] = ImVec4(0.30f, 0.40f, 0.55f, 1.0f);
    c[ImGuiCol_DockingPreview] = ImVec4(0.33f, 0.56f, 0.86f, 0.5f);
    c[ImGuiCol_DockingEmptyBg] = bg;
    c[ImGuiCol_TableHeaderBg] = ImVec4(0.16f, 0.18f, 0.21f, 1.0f);
    c[ImGuiCol_TableBorderStrong] = ImVec4(0.25f, 0.27f, 0.30f, 1.0f);
    c[ImGuiCol_TableBorderLight] = ImVec4(0.20f, 0.22f, 0.25f, 1.0f);
    c[ImGuiCol_TableRowBgAlt] = ImVec4(1.0f, 1.0f, 1.0f, 0.03f);
    c[ImGuiCol_PlotLines] = accent_hi;
    c[ImGuiCol_ModalWindowDimBg] = ImVec4(0.0f, 0.0f, 0.0f, 0.55f);

    style.ScaleAllSizes(ui_scale);
    style.FontScaleDpi = ui_scale;

    ImPlotStyle& ps = ImPlot::GetStyle();
    ImPlot::StyleColorsDark(&ps);
    ps.Colors[ImPlotCol_FrameBg] = ImVec4(0, 0, 0, 0);
    ps.Colors[ImPlotCol_PlotBg] = ImVec4(0.07f, 0.08f, 0.09f, 1.0f);
    ps.Colors[ImPlotCol_PlotBorder] = ImVec4(0.25f, 0.27f, 0.30f, 1.0f);
    ps.Colors[ImPlotCol_LegendBg] = ImVec4(0.10f, 0.11f, 0.13f, 0.85f);
    ps.Colors[ImPlotCol_AxisGrid] = ImVec4(1, 1, 1, 0.10f);
    ps.PlotPadding = ImVec2(6 * ui_scale, 6 * ui_scale);
    ps.LabelPadding = ImVec2(4 * ui_scale, 3 * ui_scale);
    ps.LegendPadding = ImVec2(8 * ui_scale, 8 * ui_scale);
    ps.PlotMinSize = ImVec2(120 * ui_scale, 80 * ui_scale);
    ps.Use24HourClock = true;
}

ImVec4 boat_color(uint8_t id) {
    // Ten ids (boat_defs BOAT_ID_MIN..MAX); others wrap.
    static const ImVec4 palette[] = {
        {0.31f, 0.76f, 0.97f, 1.0f},  // 1 sky blue
        {1.00f, 0.60f, 0.25f, 1.0f},  // 2 orange
        {0.78f, 0.47f, 0.95f, 1.0f},  // 3 violet
        {0.95f, 0.45f, 0.68f, 1.0f},  // 4 pink
        {0.30f, 0.85f, 0.78f, 1.0f},  // 5 teal
        {0.86f, 0.90f, 0.46f, 1.0f},  // 6 lime
        {0.55f, 0.60f, 1.00f, 1.0f},  // 7 periwinkle
        {0.80f, 0.66f, 0.52f, 1.0f},  // 8 tan
        {0.70f, 0.78f, 0.84f, 1.0f},  // 9 steel
        {1.00f, 0.90f, 0.40f, 1.0f},  // 10 yellow
    };
    constexpr int n = static_cast<int>(sizeof palette / sizeof palette[0]);
    const int i = id == 0 ? 0 : (id - 1) % n;
    return palette[i];
}

ImU32 boat_color_u32(uint8_t id, float alpha) {
    ImVec4 c = boat_color(id);
    c.w *= alpha;
    return ImGui::ColorConvertFloat4ToU32(c);
}

}  // namespace basestation::app
