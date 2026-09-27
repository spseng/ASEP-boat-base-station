#pragma once

// Minimal blocking HTTP(S) GET on top of libcurl, for map tiles. When the
// app is built without libcurl (BASESTATION_HAVE_CURL undefined),
// available() is false and get() always fails: the map then shows cached
// tiles only.

#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

namespace basestation::http {

struct Options {
    std::string user_agent;
    long connect_timeout_s = 10;
    long timeout_s = 30;             // whole transfer
    size_t max_bytes = 4u << 20;     // larger bodies are refused (no tile is this big)
};

struct Response {
    bool ok = false;                 // transfer completed with HTTP 2xx
    long status = 0;                 // HTTP status, 0 if no response
    bool transport_error = false;    // DNS / connect / TLS / timeout: the network, not the server
    std::string content_type;
    std::string error;               // why it failed
    std::vector<uint8_t> body;
};

// True if built with libcurl.
bool available();

// Initialises libcurl. Must run once on the main thread before any other
// thread calls get(); safe to call again.
void global_init();

// Blocking GET, following up to 3 redirects. `abort`, when set, is polled
// during the transfer and cancels it.
Response get(const std::string& url, const Options& opts, const std::atomic<bool>* abort = nullptr);

}  // namespace basestation::http
