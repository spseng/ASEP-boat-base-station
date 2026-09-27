#include <basestation/core/frame_splitter.h>

#include <algorithm>
#include <optional>

namespace basestation {

namespace {

// unwrap() wants the delimited packet, so put the delimiters back.
std::optional<wirelink::Frame> decode_run(const wirelink::bytes::Encoded& run) {
    wirelink::bytes::Packet packet;
    packet.data[0] = wirelink::START_BYTE;
    std::copy(run.data.begin(), run.data.begin() + run.len, packet.data.begin() + 1);
    packet.data[run.len + 1] = wirelink::END_BYTE;
    packet.len = run.len + 2;
    return wirelink::framing::unwrap(packet);
}

}  // namespace

void FrameSplitter::feed(const uint8_t* data, size_t len, std::vector<wirelink::Frame>& out) {
    stats_.bytes += len;
    for (size_t i = 0; i < len; ++i) {
        const uint8_t b = data[i];
        if (b != wirelink::END_BYTE) {
            // Before the first delimiter we may have joined mid-frame; those
            // bytes are not evidence of corruption.
            if (!synced_ || overflowed_) continue;
            if (run_.len >= run_.data.size()) {
                overflowed_ = true;
                run_.len = 0;
                ++stats_.overflows;
                continue;
            }
            run_.data[run_.len++] = b;
            continue;
        }

        // Empty runs are the doubled delimiter between frames, not errors.
        if (synced_ && !overflowed_ && run_.len > 0) {
            if (auto frame = decode_run(run_)) {
                out.push_back(*frame);
                ++stats_.frames_ok;
            } else {
                ++stats_.frames_bad;
            }
        }
        synced_ = true;
        overflowed_ = false;
        run_.len = 0;
    }
}

void FrameSplitter::reset() {
    run_.len = 0;
    overflowed_ = false;
    synced_ = false;
    stats_ = Stats{};
}

}  // namespace basestation
