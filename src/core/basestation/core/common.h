#pragma once

#include <wirelink/framing.h>

#include <chrono>
#include <cstdint>

namespace basestation {

using Clock = std::chrono::steady_clock;

// A frame received from the serial link, stamped on arrival.
struct RxFrame {
    Clock::time_point t;
    wirelink::Frame frame;
};

// Wall-clock time in microseconds since the Unix epoch (for log files).
inline int64_t unix_time_us() {
    using namespace std::chrono;
    return duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
}

}  // namespace basestation
