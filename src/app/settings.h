#pragma once

// Persistent operator preferences, stored as a plain key=value text file so
// it can be inspected and hand-edited. Unknown keys are ignored and missing
// keys keep their defaults, so old files keep working as fields are added.

#include <basestation/core/teleop.h>

#include <cstdint>
#include <string>

namespace basestation::app {

struct Settings {
    // Connection
    std::string port;
    uint32_t baud = 0;  // 0 = boat::serial::BAUD_RATE (filled in by load)
    bool record = true;
    std::string log_dir;  // empty = <pref dir>/logs
    bool auto_reconnect = true;

    // Teleop and commands
    TeleopConfig teleop{};
    int disable_repeats = 3;
    int repeat_interval_ms = 100;

    // View
    float plot_window_s = 120.0f;
    float text_scale = 1.0f;  // style.FontScaleMain
    bool show_fleet = true;
    bool show_boats = true;
    bool show_commands = true;
    bool show_teleop = true;
    bool show_plots = true;
    bool show_link = true;
    bool show_events = true;
};

// Returns false if the file could not be read (out keeps its defaults).
bool load_settings(const std::string& path, Settings& out);
// Writes atomically (temp file + rename). False with *error on failure.
bool save_settings(const std::string& path, const Settings& s, std::string* error);

// Serialised form, exposed so "did anything change?" is a string compare.
std::string settings_to_string(const Settings& s);

}  // namespace basestation::app
