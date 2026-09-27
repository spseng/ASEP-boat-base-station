#pragma once

#include <cstdint>
#include <string>

namespace basestation::app {

struct Options {
    std::string port;           // --port: serial device to open at startup
    uint32_t baud = 0;          // --baud: 0 = saved setting / default
    bool connect = false;       // true when --port was given
    std::string log_dir;        // --log-dir: where session logs are written
    bool record = true;         // --no-record disables session logging
    std::string screenshot_path;     // --screenshot PATH: save PNG and exit
    double screenshot_after_s = 3.0; // --screenshot-after SECONDS
    int width = 0;              // --size WxH
    int height = 0;
    bool show_help = false;
};

// Returns false with *error set on invalid arguments.
bool parse_cli(int argc, char** argv, Options& out, std::string* error);
const char* usage();

}  // namespace basestation::app
