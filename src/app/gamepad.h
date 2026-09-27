#pragma once

// SDL3 gamepad -> basestation::GamepadSnapshot. Uses the first connected
// gamepad; when it is unplugged, falls over to another one if present.

#include <basestation/core/event_log.h>
#include <basestation/core/teleop.h>

#include <SDL3/SDL_events.h>
#include <SDL3/SDL_gamepad.h>

#include <optional>
#include <string>

namespace basestation::app {

class Gamepad {
public:
    Gamepad() = default;
    ~Gamepad();
    Gamepad(const Gamepad&) = delete;
    Gamepad& operator=(const Gamepad&) = delete;

    // Handles add/remove; returns the button if `e` is a button press on the
    // active gamepad (edge, not level), so the caller can act exactly once.
    std::optional<GamepadButton> handle_event(const SDL_Event& e, EventLog& events);

    // Current state, stamped now. connected = false when there is no gamepad.
    GamepadSnapshot snapshot() const;

    bool connected() const { return pad_ != nullptr; }
    const std::string& name() const { return name_; }

private:
    void open(SDL_JoystickID id, EventLog& events);
    void close();

    SDL_Gamepad* pad_ = nullptr;
    SDL_JoystickID id_ = 0;
    std::string name_;
};

}  // namespace basestation::app
