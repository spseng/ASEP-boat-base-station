#include <basestation/core/frame_splitter.h>
#include <basestation/proto/codec.h>
#include <basestation/proto/lora.h>

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <random>
#include <vector>

using namespace basestation;
using Bytes = std::vector<uint8_t>;

namespace {

// Upstream's on-wire form: 00 <cobs> 00.
Bytes packet_for(const wirelink::Frame& f, uint8_t seq) {
    const auto p = wirelink::framing::wrap(f, seq);
    REQUIRE(p);
    return Bytes(p->data.begin(), p->data.begin() + p->len);
}

wirelink::Frame command(uint8_t rx_id, int16_t lin) {
    wirelink::Frame f{};
    REQUIRE(codec::pack(lora::Command{rx_id, lin, static_cast<int16_t>(-lin)}, f));
    return f;
}

void append(Bytes& out, const Bytes& in) { out.insert(out.end(), in.begin(), in.end()); }

std::vector<wirelink::Frame> feed_all(FrameSplitter& s, const Bytes& data) {
    std::vector<wirelink::Frame> out;
    s.feed(data.data(), data.size(), out);
    return out;
}

void check_command(const wirelink::Frame& f, uint8_t rx_id, int16_t lin, uint8_t seq) {
    CHECK(f.seq == seq);
    lora::Command c{};
    REQUIRE(codec::unpack(f, c));
    CHECK(c.rx_id == rx_id);
    CHECK(c.lin_vel == lin);
    CHECK(c.ang_vel == -lin);
}

// 20 frames in upstream's doubled-delimiter form.
Bytes stream_of_20() {
    Bytes data;
    for (uint8_t i = 0; i < 20; ++i) append(data, packet_for(command(i, static_cast<int16_t>(i * 100 - 1000)), i));
    return data;
}

void check_stream_of_20(const std::vector<wirelink::Frame>& frames) {
    REQUIRE(frames.size() == 20);
    for (uint8_t i = 0; i < 20; ++i) check_command(frames[i], i, static_cast<int16_t>(i * 100 - 1000), i);
}

}  // namespace

TEST_CASE("splitter decodes a single frame", "[io]") {
    FrameSplitter s;
    const Bytes p = packet_for(command(3, 500), 7);
    const auto frames = feed_all(s, p);
    REQUIRE(frames.size() == 1);
    check_command(frames[0], 3, 500, 7);
    CHECK(s.stats().bytes == p.size());
    CHECK(s.stats().frames_ok == 1);
    CHECK(s.stats().frames_bad == 0);
    CHECK(s.stats().overflows == 0);
}

TEST_CASE("splitter handles back-to-back frames with doubled delimiters", "[io]") {
    FrameSplitter s;
    check_stream_of_20(feed_all(s, stream_of_20()));
    CHECK(s.stats().frames_ok == 20);
    CHECK(s.stats().frames_bad == 0);
}

TEST_CASE("splitter handles a single shared delimiter between frames", "[io]") {
    Bytes data{0x00};
    for (uint8_t i = 0; i < 5; ++i) {
        const Bytes p = packet_for(command(i, i), i);
        data.insert(data.end(), p.begin() + 1, p.end());  // drop the leading 00
    }
    FrameSplitter s;
    const auto frames = feed_all(s, data);
    REQUIRE(frames.size() == 5);
    for (uint8_t i = 0; i < 5; ++i) check_command(frames[i], i, i, i);
    CHECK(s.stats().frames_bad == 0);
}

TEST_CASE("splitter is independent of how bytes are chunked", "[io]") {
    const Bytes data = stream_of_20();

    SECTION("one byte at a time") {
        FrameSplitter s;
        std::vector<wirelink::Frame> frames;
        for (uint8_t b : data) s.feed(&b, 1, frames);
        check_stream_of_20(frames);
    }
    SECTION("random chunk sizes") {
        std::mt19937 rng(1234);
        for (int trial = 0; trial < 20; ++trial) {
            FrameSplitter s;
            std::vector<wirelink::Frame> frames;
            size_t pos = 0;
            while (pos < data.size()) {
                const size_t n = std::min<size_t>(data.size() - pos, rng() % 40);
                s.feed(data.data() + pos, n, frames);  // n may be 0
                pos += n;
            }
            check_stream_of_20(frames);
            CHECK(s.stats().bytes == data.size());
        }
    }
}

TEST_CASE("splitter ignores bytes before the first delimiter", "[io]") {
    // Tail of a frame we joined halfway through, then a clean frame.
    const Bytes p = packet_for(command(1, 2), 3);
    Bytes data(p.begin() + 4, p.end() - 1);
    append(data, packet_for(command(4, 5), 6));

    FrameSplitter s;
    const auto frames = feed_all(s, data);
    REQUIRE(frames.size() == 1);
    check_command(frames[0], 4, 5, 6);
    CHECK(s.stats().frames_bad == 0);
    CHECK(s.stats().overflows == 0);
}

TEST_CASE("splitter counts a corrupt frame and recovers", "[io]") {
    Bytes bad = packet_for(command(1, 1000), 1);
    bad[bad.size() / 2] ^= 0x55;  // stays non-zero, so the run keeps its length
    REQUIRE(bad[bad.size() / 2] != 0);

    Bytes data = bad;
    append(data, packet_for(command(2, 2000), 2));
    FrameSplitter s;
    const auto frames = feed_all(s, data);
    REQUIRE(frames.size() == 1);
    check_command(frames[0], 2, 2000, 2);
    CHECK(s.stats().frames_bad == 1);
    CHECK(s.stats().frames_ok == 1);
}

TEST_CASE("splitter reports a short run as a bad frame", "[io]") {
    FrameSplitter s;
    const auto frames = feed_all(s, Bytes{0x00, 0x05, 0x01, 0x00});
    CHECK(frames.empty());
    CHECK(s.stats().frames_bad == 1);
}

TEST_CASE("splitter discards an overlong run and recovers", "[io]") {
    Bytes data{0x00};
    data.insert(data.end(), 300, 0x11);
    append(data, packet_for(command(9, -7), 42));

    FrameSplitter s;
    const auto frames = feed_all(s, data);
    REQUIRE(frames.size() == 1);
    check_command(frames[0], 9, -7, 42);
    CHECK(s.stats().overflows == 1);
    CHECK(s.stats().frames_bad == 0);
}

TEST_CASE("splitter ignores empty runs", "[io]") {
    FrameSplitter s;
    CHECK(feed_all(s, Bytes{0x00, 0x00, 0x00}).empty());
    CHECK(s.stats().frames_ok == 0);
    CHECK(s.stats().frames_bad == 0);
    CHECK(s.stats().bytes == 3);

    const auto frames = feed_all(s, packet_for(command(1, 1), 1));
    CHECK(frames.size() == 1);
}

TEST_CASE("splitter reset drops a partial frame and zeroes stats", "[io]") {
    const Bytes p = packet_for(command(1, 1), 1);
    FrameSplitter s;
    std::vector<wirelink::Frame> frames;
    s.feed(p.data(), p.size() - 3, frames);
    s.reset();
    CHECK(s.stats().bytes == 0);
    // The remainder is unsynchronised garbage after a reset.
    s.feed(p.data() + p.size() - 3, 3, frames);
    CHECK(frames.empty());
    CHECK(s.stats().frames_bad == 0);
}
