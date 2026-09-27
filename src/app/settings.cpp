#include "settings.h"

#include <boat_defs/serial.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>

namespace basestation::app {

namespace {

std::string trim(const std::string& s) {
    const char* ws = " \t\r\n";
    const size_t b = s.find_first_not_of(ws);
    if (b == std::string::npos) return {};
    const size_t e = s.find_last_not_of(ws);
    return s.substr(b, e - b + 1);
}

std::string fmt_float(float v) {
    char buf[32];
    std::snprintf(buf, sizeof buf, "%g", static_cast<double>(v));
    return buf;
}

bool parse_bool(const std::string& v, bool& out) {
    if (v == "1" || v == "true" || v == "on" || v == "yes") { out = true; return true; }
    if (v == "0" || v == "false" || v == "off" || v == "no") { out = false; return true; }
    return false;
}

// Out-of-range or malformed values are ignored rather than clamped, so a
// typo cannot silently turn into an extreme setting.
bool parse_float(const std::string& v, float lo, float hi, float& out) {
    char* end = nullptr;
    errno = 0;
    const float f = std::strtof(v.c_str(), &end);
    if (errno || end == v.c_str() || *end || !std::isfinite(f) || f < lo || f > hi) return false;
    out = f;
    return true;
}

bool parse_int(const std::string& v, long lo, long hi, long& out) {
    char* end = nullptr;
    errno = 0;
    const long n = std::strtol(v.c_str(), &end, 10);
    if (errno || end == v.c_str() || *end || n < lo || n > hi) return false;
    out = n;
    return true;
}

// Enums are stored by their display name (to_string), so reordering the
// enum in core cannot remap a saved choice to a different control.
template <class E>
bool parse_enum(const std::string& v, E& out) {
    for (int i = 0; i < static_cast<int>(E::Count); ++i) {
        if (v == to_string(static_cast<E>(i))) {
            out = static_cast<E>(i);
            return true;
        }
    }
    return false;
}

}  // namespace

std::string settings_to_string(const Settings& s) {
    const TeleopConfig& t = s.teleop;
    std::ostringstream o;
    o << "# ASEP base station settings (key = value)\n";
    o << "port = " << s.port << "\n";
    o << "baud = " << s.baud << "\n";
    o << "record = " << (s.record ? 1 : 0) << "\n";
    o << "log_dir = " << s.log_dir << "\n";
    o << "auto_reconnect = " << (s.auto_reconnect ? 1 : 0) << "\n";
    o << "teleop.linear_axis = " << to_string(t.linear_axis) << "\n";
    o << "teleop.angular_axis = " << to_string(t.angular_axis) << "\n";
    o << "teleop.invert_linear = " << (t.invert_linear ? 1 : 0) << "\n";
    o << "teleop.invert_angular = " << (t.invert_angular ? 1 : 0) << "\n";
    o << "teleop.deadzone = " << fmt_float(t.deadzone) << "\n";
    o << "teleop.expo = " << fmt_float(t.expo) << "\n";
    o << "teleop.max_linear_mps = " << fmt_float(t.max_linear_mps) << "\n";
    o << "teleop.max_angular_radps = " << fmt_float(t.max_angular_radps) << "\n";
    o << "teleop.require_deadman = " << (t.require_deadman ? 1 : 0) << "\n";
    o << "teleop.deadman = " << to_string(t.deadman) << "\n";
    o << "teleop.disable_button = " << to_string(t.disable_button) << "\n";
    o << "teleop.rate_hz = " << fmt_float(t.rate_hz) << "\n";
    o << "teleop.input_stale_ms = " << t.input_stale_ms << "\n";
    o << "teleop.stop_burst = " << t.stop_burst << "\n";
    o << "commander.disable_repeats = " << s.disable_repeats << "\n";
    o << "commander.repeat_interval_ms = " << s.repeat_interval_ms << "\n";
    o << "view.plot_window_s = " << fmt_float(s.plot_window_s) << "\n";
    o << "view.text_scale = " << fmt_float(s.text_scale) << "\n";
    o << "view.show_fleet = " << (s.show_fleet ? 1 : 0) << "\n";
    o << "view.show_boats = " << (s.show_boats ? 1 : 0) << "\n";
    o << "view.show_commands = " << (s.show_commands ? 1 : 0) << "\n";
    o << "view.show_teleop = " << (s.show_teleop ? 1 : 0) << "\n";
    o << "view.show_plots = " << (s.show_plots ? 1 : 0) << "\n";
    o << "view.show_link = " << (s.show_link ? 1 : 0) << "\n";
    o << "view.show_events = " << (s.show_events ? 1 : 0) << "\n";
    return o.str();
}

bool load_settings(const std::string& path, Settings& s) {
    if (s.baud == 0) s.baud = boat::serial::BAUD_RATE;
    std::ifstream in(path);
    if (!in) return false;

    std::map<std::string, std::string> kv;
    std::string line;
    while (std::getline(in, line)) {
        const std::string l = trim(line);
        if (l.empty() || l[0] == '#') continue;
        const size_t eq = l.find('=');
        if (eq == std::string::npos) continue;
        kv[trim(l.substr(0, eq))] = trim(l.substr(eq + 1));
    }
    auto get = [&](const char* key) -> const std::string* {
        auto it = kv.find(key);
        return it == kv.end() ? nullptr : &it->second;
    };
    auto get_bool = [&](const char* key, bool& out) {
        if (const std::string* v = get(key)) parse_bool(*v, out);
    };
    auto get_float = [&](const char* key, float lo, float hi, float& out) {
        if (const std::string* v = get(key)) parse_float(*v, lo, hi, out);
    };
    auto get_int = [&](const char* key, long lo, long hi, int& out) {
        long n = 0;
        if (const std::string* v = get(key); v && parse_int(*v, lo, hi, n)) out = static_cast<int>(n);
    };

    if (const std::string* v = get("port")) s.port = *v;
    {
        long b = 0;
        if (const std::string* v = get("baud"); v && parse_int(*v, 300, 4000000, b))
            s.baud = static_cast<uint32_t>(b);
    }
    get_bool("record", s.record);
    if (const std::string* v = get("log_dir")) s.log_dir = *v;
    get_bool("auto_reconnect", s.auto_reconnect);

    TeleopConfig& t = s.teleop;
    if (const std::string* v = get("teleop.linear_axis")) parse_enum(*v, t.linear_axis);
    if (const std::string* v = get("teleop.angular_axis")) parse_enum(*v, t.angular_axis);
    get_bool("teleop.invert_linear", t.invert_linear);
    get_bool("teleop.invert_angular", t.invert_angular);
    get_float("teleop.deadzone", 0.0f, 0.9f, t.deadzone);
    get_float("teleop.expo", 0.0f, 1.0f, t.expo);
    get_float("teleop.max_linear_mps", 0.0f, 10.0f, t.max_linear_mps);
    get_float("teleop.max_angular_radps", 0.0f, 10.0f, t.max_angular_radps);
    get_bool("teleop.require_deadman", t.require_deadman);
    if (const std::string* v = get("teleop.deadman")) parse_enum(*v, t.deadman);
    if (const std::string* v = get("teleop.disable_button")) parse_enum(*v, t.disable_button);
    get_float("teleop.rate_hz", 1.0f, 50.0f, t.rate_hz);
    get_int("teleop.input_stale_ms", 50, 5000, t.input_stale_ms);
    get_int("teleop.stop_burst", 0, 20, t.stop_burst);

    get_int("commander.disable_repeats", 1, 20, s.disable_repeats);
    get_int("commander.repeat_interval_ms", 10, 2000, s.repeat_interval_ms);

    get_float("view.plot_window_s", 30.0f, 900.0f, s.plot_window_s);
    get_float("view.text_scale", 0.6f, 2.5f, s.text_scale);
    get_bool("view.show_fleet", s.show_fleet);
    get_bool("view.show_boats", s.show_boats);
    get_bool("view.show_commands", s.show_commands);
    get_bool("view.show_teleop", s.show_teleop);
    get_bool("view.show_plots", s.show_plots);
    get_bool("view.show_link", s.show_link);
    get_bool("view.show_events", s.show_events);
    return true;
}

bool save_settings(const std::string& path, const Settings& s, std::string* error) {
    // Write to a sibling temp file and rename, so a crash mid-write never
    // leaves a truncated settings file behind.
    const std::string tmp = path + ".tmp";
    {
        std::ofstream out(tmp, std::ios::trunc);
        if (!out) {
            if (error) *error = "cannot write " + tmp + ": " + std::strerror(errno);
            return false;
        }
        out << settings_to_string(s);
        out.flush();
        if (!out) {
            if (error) *error = "write failed: " + tmp;
            return false;
        }
    }
    if (std::rename(tmp.c_str(), path.c_str()) != 0) {
        if (error) *error = "cannot rename " + tmp + ": " + std::strerror(errno);
        std::remove(tmp.c_str());
        return false;
    }
    return true;
}

}  // namespace basestation::app
