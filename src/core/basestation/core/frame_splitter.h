#pragma once

// Splits a raw byte stream into wirelink frames.
//
// Frames are 0x00-delimited COBS. Every run of non-zero bytes between two
// 0x00s is one candidate frame, decoded with upstream
// wirelink::framing::unwrap (COBS + length + CRC16). Unlike
// wirelink::Link::feed, this reports corrupt frames separately from
// "nothing yet", accepts a single shared delimiter between frames as well as
// the doubled "00 .. 00 00 .. 00" form upstream emits, and discards runs
// longer than a frame can be without buffering them.

#include <wirelink/framing.h>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace basestation {

class FrameSplitter {
public:
    struct Stats {
        uint64_t bytes = 0;       // total bytes fed
        uint64_t frames_ok = 0;   // decoded frames
        uint64_t frames_bad = 0;  // delimited runs that failed COBS/len/CRC
        uint64_t overflows = 0;   // runs longer than wirelink::MAX_ENCODED
    };

    // Appends every complete, valid frame found in `data` to `out`.
    void feed(const uint8_t* data, size_t len, std::vector<wirelink::Frame>& out);

    const Stats& stats() const { return stats_; }
    void reset();  // drops any partial frame and zeroes stats

private:
    wirelink::bytes::Encoded run_{};  // bytes since the last 0x00
    bool overflowed_ = false;         // current run exceeded capacity
    bool synced_ = false;             // seen at least one 0x00
    Stats stats_{};
};

}  // namespace basestation
