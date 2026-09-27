#pragma once

// Persistent operator preferences, stored as a plain key=value text file so
// it can be inspected and hand-edited. Unknown keys are ignored and missing
// keys keep their defaults, so old files keep working as fields are added.

#include <basestation/core/teleop.h>

#include <cstdint>
#include <string>

namespace basestation::app {

// Map tiles under the Fleet view.
struct MapSettings {
    bool enabled = false;
    std::string source = "esri-world-imagery";  // tiles::TileSource id, or "custom"
    std::string custom_url;                     // URL template for "custom"
    float opacity = 0.85f;
    bool offline = false;    // never download, show cached tiles only
    std::string cache_dir;   // empty = <pref dir>/tiles
};

// Base-station position: set by hand (cached here), or from the base GPS.
struct BaseSettings {
    bool manual_set = false;
    double lat = 0;
    double lon = 0;
    bool pin_manual = false;  // prefer the manual position over the base GPS
    bool as_origin = false;   // anchor the Fleet view at the base station
    // Last GPS fix, used when nothing better is known (e.g. before the base
    // GPS has a fix in the next session).
    bool gps_cached = false;
    double gps_lat = 0;
    double gps_lon = 0;
};

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

    MapSettings map{};
    BaseSettings base{};
};

// Returns false if the file could not be read (out keeps its defaults).
bool load_settings(const std::string& path, Settings& out);
// Writes atomically (temp file + rename). False with *error on failure.
bool save_settings(const std::string& path, const Settings& s, std::string* error);

// Serialised form, exposed so "did anything change?" is a string compare.
std::string settings_to_string(const Settings& s);

}  // namespace basestation::app
