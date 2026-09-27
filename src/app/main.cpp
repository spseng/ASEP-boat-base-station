// ASEP base station: entry point and platform shell (SDL3 window, renderer,
// Dear ImGui / ImPlot setup, main loop, screenshots). The UI itself lives in
// App (app.h).

#include "app.h"
#include "cli.h"

#include <SDL3/SDL.h>
#include <imgui.h>
#include <imgui_impl_sdl3.h>
#include <imgui_impl_sdlrenderer3.h>
#include <implot.h>

#include <cfloat>
#include <cstdio>
#include <string>

namespace {

// Reads back the current frame and writes it as PNG. Must be called after
// rendering and before SDL_RenderPresent.
bool save_screenshot(SDL_Renderer* renderer, const std::string& path) {
    SDL_Surface* surface = SDL_RenderReadPixels(renderer, nullptr);
    if (!surface) {
        std::fprintf(stderr, "screenshot: %s\n", SDL_GetError());
        return false;
    }
    const bool ok = SDL_SavePNG(surface, path.c_str());
    if (!ok) std::fprintf(stderr, "screenshot: %s\n", SDL_GetError());
    SDL_DestroySurface(surface);
    return ok;
}

}  // namespace

int main(int argc, char** argv) {
    basestation::app::Options opts;
    std::string cli_error;
    if (!basestation::app::parse_cli(argc, argv, opts, &cli_error)) {
        std::fprintf(stderr, "%s\n\n%s", cli_error.c_str(), basestation::app::usage());
        return 2;
    }
    if (opts.show_help) {
        std::fputs(basestation::app::usage(), stdout);
        return 0;
    }

    // Teleop must keep working when another window has focus (e.g. the
    // operator clicks into a terminal). The deadman button still gates it.
    SDL_SetHint(SDL_HINT_JOYSTICK_ALLOW_BACKGROUND_EVENTS, "1");
    SDL_SetHint(SDL_HINT_APP_NAME, "ASEP Base Station");

    if (!SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMEPAD)) {
        std::fprintf(stderr, "SDL_Init failed: %s\n", SDL_GetError());
        return 1;
    }

    const float scale = SDL_GetDisplayContentScale(SDL_GetPrimaryDisplay());
    const float ui_scale = scale > 0.0f ? scale : 1.0f;
    const int width = opts.width > 0 ? opts.width : static_cast<int>(1440 * ui_scale);
    const int height = opts.height > 0 ? opts.height : static_cast<int>(900 * ui_scale);

    SDL_Window* window = SDL_CreateWindow("ASEP Base Station", width, height,
                                          SDL_WINDOW_RESIZABLE | SDL_WINDOW_HIGH_PIXEL_DENSITY);
    if (!window) {
        std::fprintf(stderr, "SDL_CreateWindow failed: %s\n", SDL_GetError());
        SDL_Quit();
        return 1;
    }
    SDL_Renderer* renderer = SDL_CreateRenderer(window, nullptr);
    if (!renderer) {
        std::fprintf(stderr, "SDL_CreateRenderer failed: %s\n", SDL_GetError());
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }
    SDL_SetRenderVSync(renderer, 1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard | ImGuiConfigFlags_DockingEnable;
    // The gamepad drives boats, never the UI.
    io.ConfigFlags &= ~ImGuiConfigFlags_NavEnableGamepad;

    ImGui_ImplSDL3_InitForSDLRenderer(window, renderer);
    ImGui_ImplSDLRenderer3_Init(renderer);

    {
        basestation::app::App app(opts, ui_scale, renderer);

        const Uint64 start_ms = SDL_GetTicks();
        bool screenshot_done = false;
        bool running = true;
        while (running) {
            SDL_Event event;
            while (SDL_PollEvent(&event)) {
                ImGui_ImplSDL3_ProcessEvent(&event);
                app.handle_event(event);
                if (event.type == SDL_EVENT_QUIT) running = false;
                if (event.type == SDL_EVENT_WINDOW_CLOSE_REQUESTED &&
                    event.window.windowID == SDL_GetWindowID(window)) {
                    running = false;
                }
            }
            if (app.quit_requested()) running = false;

            ImGui_ImplSDLRenderer3_NewFrame();
            ImGui_ImplSDL3_NewFrame();
            // Screenshots: keep the (virtual) mouse out of the window so no
            // hover highlight or tooltip ends up in the picture.
            if (!opts.screenshot_path.empty()) io.AddMousePosEvent(-FLT_MAX, -FLT_MAX);
            ImGui::NewFrame();

            app.frame();

            ImGui::Render();
            SDL_SetRenderScale(renderer, io.DisplayFramebufferScale.x, io.DisplayFramebufferScale.y);
            SDL_SetRenderDrawColorFloat(renderer, 0.06f, 0.07f, 0.08f, 1.0f);
            SDL_RenderClear(renderer);
            ImGui_ImplSDLRenderer3_RenderDrawData(ImGui::GetDrawData(), renderer);

            // --screenshot: capture once after the delay, then quit. Used for
            // headless verification and documentation.
            if (!opts.screenshot_path.empty() && !screenshot_done &&
                SDL_GetTicks() - start_ms >= static_cast<Uint64>(opts.screenshot_after_s * 1000.0)) {
                screenshot_done = true;
                const bool ok = save_screenshot(renderer, opts.screenshot_path);
                std::printf("screenshot %s: %s\n", ok ? "saved" : "FAILED", opts.screenshot_path.c_str());
                running = false;
            }

            SDL_RenderPresent(renderer);
        }
    }

    ImGui_ImplSDLRenderer3_Shutdown();
    ImGui_ImplSDL3_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
