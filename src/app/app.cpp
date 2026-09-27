#include "app.h"

#include <imgui.h>
#include <implot.h>

namespace basestation::app {

App::App(const Options& opts, float ui_scale) : opts_(opts), ui_scale_(ui_scale) {}
App::~App() = default;

void App::handle_event(const SDL_Event&) {}

void App::frame() {
    ImGui::DockSpaceOverViewport();
    ImGui::Begin("Placeholder");
    ImGui::Text("ASEP base station shell");
    ImGui::End();
    ImPlot::ShowDemoWindow();
}

}  // namespace basestation::app
