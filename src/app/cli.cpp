#include "cli.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace basestation::app {

const char* usage() {
    return "Usage: asep_base_station [options]\n"
           "\n"
           "  --port PATH             open this serial port at startup\n"
           "  --baud RATE             baud rate (default: last used, else 115200)\n"
           "  --log-dir DIR           directory for session logs (default: app data dir)\n"
           "  --no-record             do not write a session log\n"
           "  --size WxH              initial window size in pixels\n"
           "  --screenshot FILE.png   save a screenshot after a delay, then exit\n"
           "  --screenshot-after S    delay before the screenshot (default 3)\n"
           "  -h, --help              show this help\n"
           "\n"
           "For screenshots / testing:\n"
           "  --fresh                 ignore saved settings and window layout; save nothing\n"
           "  --select ID             select boat ID at startup\n"
           "  --focus NAME            bring a panel to the front (Fleet, Boats, Commands,\n"
           "                          Teleop, Plots, Link, Events)\n"
           "  --test-disable ID       send Disable to boat ID (255 = all) 3 s after connecting\n"
           "  --test-teleop ID        enable teleop to boat ID at startup\n";
}

bool parse_cli(int argc, char** argv, Options& out, std::string* error) {
    auto need_value = [&](int& i, const char* name) -> const char* {
        if (i + 1 >= argc) {
            *error = std::string("missing value for ") + name;
            return nullptr;
        }
        return argv[++i];
    };

    for (int i = 1; i < argc; ++i) {
        const char* a = argv[i];
        if (!std::strcmp(a, "-h") || !std::strcmp(a, "--help")) {
            out.show_help = true;
        } else if (!std::strcmp(a, "--port")) {
            const char* v = need_value(i, a);
            if (!v) return false;
            out.port = v;
            out.connect = true;
        } else if (!std::strcmp(a, "--baud")) {
            const char* v = need_value(i, a);
            if (!v) return false;
            char* end = nullptr;
            const unsigned long b = std::strtoul(v, &end, 10);
            if (!end || *end || b == 0 || b > 4000000) {
                *error = std::string("invalid baud rate: ") + v;
                return false;
            }
            out.baud = static_cast<uint32_t>(b);
        } else if (!std::strcmp(a, "--log-dir")) {
            const char* v = need_value(i, a);
            if (!v) return false;
            out.log_dir = v;
        } else if (!std::strcmp(a, "--no-record")) {
            out.record = false;
        } else if (!std::strcmp(a, "--size")) {
            const char* v = need_value(i, a);
            if (!v) return false;
            if (std::sscanf(v, "%dx%d", &out.width, &out.height) != 2 || out.width < 320 ||
                out.height < 240) {
                *error = std::string("invalid size (expected WxH): ") + v;
                return false;
            }
        } else if (!std::strcmp(a, "--screenshot")) {
            const char* v = need_value(i, a);
            if (!v) return false;
            out.screenshot_path = v;
        } else if (!std::strcmp(a, "--screenshot-after")) {
            const char* v = need_value(i, a);
            if (!v) return false;
            out.screenshot_after_s = std::atof(v);
            if (out.screenshot_after_s < 0) out.screenshot_after_s = 0;
        } else if (!std::strcmp(a, "--fresh")) {
            out.fresh = true;
        } else if (!std::strcmp(a, "--select") || !std::strcmp(a, "--test-disable") ||
                   !std::strcmp(a, "--test-teleop")) {
            const char* v = need_value(i, a);
            if (!v) return false;
            char* end = nullptr;
            const long id = std::strtol(v, &end, 10);
            if (!end || *end || id < 1 || id > 255) {
                *error = std::string("invalid boat id for ") + a + ": " + v;
                return false;
            }
            if (!std::strcmp(a, "--select")) out.select_boat = static_cast<int>(id);
            else if (!std::strcmp(a, "--test-disable")) out.test_disable = static_cast<int>(id);
            else out.test_teleop = static_cast<int>(id);
        } else if (!std::strcmp(a, "--focus")) {
            const char* v = need_value(i, a);
            if (!v) return false;
            out.focus_window = v;
        } else {
            *error = std::string("unknown option: ") + a;
            return false;
        }
    }
    return true;
}

}  // namespace basestation::app
