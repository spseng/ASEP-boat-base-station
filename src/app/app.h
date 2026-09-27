#pragma once

#include "cli.h"

#include <SDL3/SDL_events.h>

namespace basestation::app {

class App {
public:
    App(const Options& opts, float ui_scale);
    ~App();
    App(const App&) = delete;
    App& operator=(const App&) = delete;

    void handle_event(const SDL_Event& event);
    void frame();  // builds the whole UI for one frame
    bool quit_requested() const { return quit_; }

private:
    Options opts_;
    float ui_scale_;
    bool quit_ = false;
};

}  // namespace basestation::app
