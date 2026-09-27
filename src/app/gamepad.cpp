#include "gamepad.h"

#include <SDL3/SDL_stdinc.h>

#include <algorithm>

namespace basestation::app {

namespace {

std::optional<GamepadButton> map_button(Uint8 b) {
    switch (static_cast<SDL_GamepadButton>(b)) {
    case SDL_GAMEPAD_BUTTON_SOUTH: return GamepadButton::South;
    case SDL_GAMEPAD_BUTTON_EAST: return GamepadButton::East;
    case SDL_GAMEPAD_BUTTON_WEST: return GamepadButton::West;
    case SDL_GAMEPAD_BUTTON_NORTH: return GamepadButton::North;
    case SDL_GAMEPAD_BUTTON_LEFT_SHOULDER: return GamepadButton::LeftShoulder;
    case SDL_GAMEPAD_BUTTON_RIGHT_SHOULDER: return GamepadButton::RightShoulder;
    case SDL_GAMEPAD_BUTTON_BACK: return GamepadButton::Back;
    case SDL_GAMEPAD_BUTTON_START: return GamepadButton::Start;
    case SDL_GAMEPAD_BUTTON_GUIDE: return GamepadButton::Guide;
    case SDL_GAMEPAD_BUTTON_LEFT_STICK: return GamepadButton::LeftStick;
    case SDL_GAMEPAD_BUTTON_RIGHT_STICK: return GamepadButton::RightStick;
    case SDL_GAMEPAD_BUTTON_DPAD_UP: return GamepadButton::DpadUp;
    case SDL_GAMEPAD_BUTTON_DPAD_DOWN: return GamepadButton::DpadDown;
    case SDL_GAMEPAD_BUTTON_DPAD_LEFT: return GamepadButton::DpadLeft;
    case SDL_GAMEPAD_BUTTON_DPAD_RIGHT: return GamepadButton::DpadRight;
    default: return std::nullopt;
    }
}

SDL_GamepadButton to_sdl(GamepadButton b) {
    switch (b) {
    case GamepadButton::South: return SDL_GAMEPAD_BUTTON_SOUTH;
    case GamepadButton::East: return SDL_GAMEPAD_BUTTON_EAST;
    case GamepadButton::West: return SDL_GAMEPAD_BUTTON_WEST;
    case GamepadButton::North: return SDL_GAMEPAD_BUTTON_NORTH;
    case GamepadButton::LeftShoulder: return SDL_GAMEPAD_BUTTON_LEFT_SHOULDER;
    case GamepadButton::RightShoulder: return SDL_GAMEPAD_BUTTON_RIGHT_SHOULDER;
    case GamepadButton::Back: return SDL_GAMEPAD_BUTTON_BACK;
    case GamepadButton::Start: return SDL_GAMEPAD_BUTTON_START;
    case GamepadButton::Guide: return SDL_GAMEPAD_BUTTON_GUIDE;
    case GamepadButton::LeftStick: return SDL_GAMEPAD_BUTTON_LEFT_STICK;
    case GamepadButton::RightStick: return SDL_GAMEPAD_BUTTON_RIGHT_STICK;
    case GamepadButton::DpadUp: return SDL_GAMEPAD_BUTTON_DPAD_UP;
    case GamepadButton::DpadDown: return SDL_GAMEPAD_BUTTON_DPAD_DOWN;
    case GamepadButton::DpadLeft: return SDL_GAMEPAD_BUTTON_DPAD_LEFT;
    case GamepadButton::DpadRight: return SDL_GAMEPAD_BUTTON_DPAD_RIGHT;
    case GamepadButton::Count: break;
    }
    return SDL_GAMEPAD_BUTTON_INVALID;
}

// SDL axes are int16; normalise symmetrically so full travel is +-1 both ways.
float norm(Sint16 v) { return std::clamp(static_cast<float>(v) / 32767.0f, -1.0f, 1.0f); }

}  // namespace

Gamepad::~Gamepad() { close(); }

void Gamepad::open(SDL_JoystickID id, EventLog& events) {
    SDL_Gamepad* pad = SDL_OpenGamepad(id);
    if (!pad) {
        events.warn(std::string("Gamepad could not be opened: ") + SDL_GetError());
        return;
    }
    pad_ = pad;
    id_ = id;
    const char* n = SDL_GetGamepadName(pad);
    name_ = n ? n : "Gamepad";
    events.info("Gamepad connected: " + name_);
}

void Gamepad::close() {
    if (pad_) SDL_CloseGamepad(pad_);
    pad_ = nullptr;
    id_ = 0;
    name_.clear();
}

std::optional<GamepadButton> Gamepad::handle_event(const SDL_Event& e, EventLog& events) {
    switch (e.type) {
    case SDL_EVENT_GAMEPAD_ADDED:
        // SDL also reports pads that were present at startup this way.
        if (!pad_) open(e.gdevice.which, events);
        break;
    case SDL_EVENT_GAMEPAD_REMOVED:
        if (pad_ && e.gdevice.which == id_) {
            events.warn("Gamepad disconnected: " + name_);
            close();
            // Fall over to any other pad that is still attached.
            int count = 0;
            if (SDL_JoystickID* ids = SDL_GetGamepads(&count)) {
                for (int i = 0; i < count && !pad_; ++i) {
                    if (ids[i] != e.gdevice.which) open(ids[i], events);
                }
                SDL_free(ids);
            }
        }
        break;
    case SDL_EVENT_GAMEPAD_BUTTON_DOWN:
        if (pad_ && e.gbutton.which == id_) return map_button(e.gbutton.button);
        break;
    default:
        break;
    }
    return std::nullopt;
}

GamepadSnapshot Gamepad::snapshot() const {
    GamepadSnapshot s;
    s.t = Clock::now();
    if (!pad_) return s;
    s.connected = true;
    s.name = name_;
    auto set_axis = [&](GamepadAxis a, float v) { s.axes[static_cast<size_t>(a)] = v; };
    // SDL's Y axes are positive DOWN; the snapshot contract is UP positive.
    set_axis(GamepadAxis::LeftX, norm(SDL_GetGamepadAxis(pad_, SDL_GAMEPAD_AXIS_LEFTX)));
    set_axis(GamepadAxis::LeftY, -norm(SDL_GetGamepadAxis(pad_, SDL_GAMEPAD_AXIS_LEFTY)));
    set_axis(GamepadAxis::RightX, norm(SDL_GetGamepadAxis(pad_, SDL_GAMEPAD_AXIS_RIGHTX)));
    set_axis(GamepadAxis::RightY, -norm(SDL_GetGamepadAxis(pad_, SDL_GAMEPAD_AXIS_RIGHTY)));
    // Triggers are 0..32767.
    set_axis(GamepadAxis::LeftTrigger,
             std::clamp(norm(SDL_GetGamepadAxis(pad_, SDL_GAMEPAD_AXIS_LEFT_TRIGGER)), 0.0f, 1.0f));
    set_axis(GamepadAxis::RightTrigger,
             std::clamp(norm(SDL_GetGamepadAxis(pad_, SDL_GAMEPAD_AXIS_RIGHT_TRIGGER)), 0.0f, 1.0f));
    for (size_t i = 0; i < static_cast<size_t>(GamepadButton::Count); ++i) {
        s.buttons[i] = SDL_GetGamepadButton(pad_, to_sdl(static_cast<GamepadButton>(i)));
    }
    return s;
}

}  // namespace basestation::app
