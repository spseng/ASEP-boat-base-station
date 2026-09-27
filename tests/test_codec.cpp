#include <basestation/proto/base.h>
#include <basestation/proto/codec.h>
#include <basestation/proto/lora.h>

#include <wirelink/msg/serial.h>
#include <wirelink/wire.h>

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstring>
#include <limits>
#include <vector>

using namespace basestation;
namespace ser = wirelink::msg::serial;
using wirelink::msg::common::LinkStatus;
using wirelink::msg::common::PeerEntry;

namespace {

std::vector<uint8_t> payload_bytes(const wirelink::Frame& f) {
    return {f.payload.data.begin(), f.payload.data.begin() + f.len};
}

template <class T>
std::vector<uint8_t> encode(const T& msg) {
    wirelink::Frame f{};
    REQUIRE(codec::pack(msg, f));
    return payload_bytes(f);
}

// pack -> unpack -> pack must reproduce the same bytes; returns the decoded
// message for field-level checks.
template <class T>
T round_trip(const T& msg) {
    wirelink::Frame f{};
    REQUIRE(codec::pack(msg, f));
    CHECK(f.type == static_cast<uint8_t>(T::TYPE));
    CHECK(f.payload.len == f.len);
    CHECK(f.len == codec::wire_size(msg));
    T out{};
    REQUIRE(codec::unpack(f, out));
    CHECK(encode(out) == payload_bytes(f));
    return out;
}

// Our codec and upstream's must agree byte for byte and decode each other.
template <class T>
void check_upstream_compat(const T& msg) {
    wirelink::Frame ours{};
    wirelink::Frame theirs{};
    REQUIRE(codec::pack(msg, ours));
    REQUIRE(wirelink::codec::pack(msg, theirs));
    CHECK(ours.type == theirs.type);
    REQUIRE(ours.len == theirs.len);
    CHECK(payload_bytes(ours) == payload_bytes(theirs));

    T from_theirs{};
    REQUIRE(codec::unpack(theirs, from_theirs));
    CHECK(encode(from_theirs) == payload_bytes(theirs));

    T from_ours{};
    REQUIRE(wirelink::codec::unpack(ours, from_ours));
    wirelink::Frame again{};
    REQUIRE(wirelink::codec::pack(from_ours, again));
    CHECK(payload_bytes(again) == payload_bytes(ours));
}

PeerEntry peer(uint8_t id) {
    return PeerEntry{id, -337123456 - id, 1512345678 + id, 0.25f * id, 1000u * id};
}

LinkStatus link(uint8_t id) {
    return LinkStatus{id, -97.5f - id, 7.25f + id, 3u * id};
}

}  // namespace

TEST_CASE("lora messages round-trip", "[codec]") {
    SECTION("Disable") {
        const auto out = round_trip(lora::Disable{0xFF});
        CHECK(out.rx_id == 0xFF);
    }
    SECTION("SelfStatus") {
        const auto out = round_trip(lora::SelfStatus{peer(3)});
        CHECK(out.self.id == 3);
        CHECK(out.self.lat == -337123459);
        CHECK(out.self.lon == 1512345681);
        CHECK(out.self.scalar == 0.75f);
        CHECK(out.self.age_ms == 3000u);
    }
    SECTION("Command with negative velocities") {
        const auto out = round_trip(lora::Command{2, -1234, std::numeric_limits<int16_t>::min()});
        CHECK(out.rx_id == 2);
        CHECK(out.lin_vel == -1234);
        CHECK(out.ang_vel == std::numeric_limits<int16_t>::min());
    }
    SECTION("Status with NaN fields") {
        const float nan = std::numeric_limits<float>::quiet_NaN();
        const auto out = round_trip(lora::Status{4, 2, 1, 0, 0xBEEF, nan, -120.5f, nan});
        CHECK(out.tx_id == 4);
        CHECK(out.mode == 2);
        CHECK(out.armed == 1);
        CHECK(out.gate_state == 0);
        CHECK(out.fault_flags == 0xBEEF);
        CHECK(std::isnan(out.heading_deg));
        CHECK(out.gs_rssi == -120.5f);
        CHECK(std::isnan(out.gs_snr));
    }
    SECTION("Reenable") {
        CHECK(round_trip(lora::Reenable{7}).rx_id == 7);
    }
    SECTION("SetMode") {
        const auto out = round_trip(lora::SetMode{1, 3, 1});
        CHECK(out.rx_id == 1);
        CHECK(out.mode == 3);
        CHECK(out.armed == 1);
    }
}

TEST_CASE("base-local messages round-trip", "[codec]") {
    const auto info = round_trip(base::RxInfo{-101.25f, -3.5f});
    CHECK(info.rssi == -101.25f);
    CHECK(info.snr == -3.5f);

    const auto st = round_trip(base::BaseStatus{0xFFFFFFF0u, 12, 3, 0x01020304u});
    CHECK(st.uptime_ms == 0xFFFFFFF0u);
    CHECK(st.rx_ok == 12u);
    CHECK(st.rx_bad == 3u);
    CHECK(st.tx_count == 0x01020304u);
    CHECK(base::is_local_type(static_cast<uint8_t>(base::RxInfo::TYPE)));
    CHECK_FALSE(base::is_local_type(static_cast<uint8_t>(lora::SetMode::TYPE)));
}

TEST_CASE("wire sizes", "[codec]") {
    CHECK(codec::wire_size(lora::Disable{}) == 1);
    CHECK(codec::wire_size(lora::SelfStatus{}) == 17);
    CHECK(codec::wire_size(lora::Command{}) == 5);
    CHECK(codec::wire_size(lora::Status{}) == 18);
    CHECK(codec::wire_size(lora::Reenable{}) == 1);
    CHECK(codec::wire_size(lora::SetMode{}) == 3);
    CHECK(codec::wire_size(base::RxInfo{}) == 8);
    CHECK(codec::wire_size(base::BaseStatus{}) == 16);

    // The whole frame (type, seq, len, payload, crc) must fit a LoRa packet.
    for (size_t n : {codec::wire_size(lora::Disable{}), codec::wire_size(lora::SelfStatus{}),
                     codec::wire_size(lora::Command{}), codec::wire_size(lora::Status{}),
                     codec::wire_size(lora::Reenable{}), codec::wire_size(lora::SetMode{})}) {
        CHECK(n > 0);
        CHECK(n + 5 <= lora::MAX_LORA_PAYLOAD);
    }
}

TEST_CASE("fields are little-endian two's complement", "[codec]") {
    CHECK(encode(lora::Command{0x11, -2, 0x0102}) == std::vector<uint8_t>{0x11, 0xFE, 0xFF, 0x02, 0x01});
    const auto st = encode(lora::SelfStatus{PeerEntry{1, -1, 2, 0.0f, 0}});
    CHECK(std::vector<uint8_t>(st.begin() + 1, st.begin() + 5) == std::vector<uint8_t>{0xFF, 0xFF, 0xFF, 0xFF});
}

TEST_CASE("unpack rejects mismatched frames", "[codec]") {
    wirelink::Frame f{};
    REQUIRE(codec::pack(lora::Command{1, 100, -100}, f));

    SECTION("wrong type") {
        lora::SetMode m{};
        CHECK_FALSE(codec::unpack(f, m));
        f.type = static_cast<uint8_t>(lora::SetMode::TYPE);
        lora::Command c{};
        CHECK_FALSE(codec::unpack(f, c));
    }
    SECTION("short payload") {
        f.len = 4;
        lora::Command c{};
        CHECK_FALSE(codec::unpack(f, c));
        f.len = 0;
        CHECK_FALSE(codec::unpack(f, c));
    }
    SECTION("trailing bytes") {
        f.len = 6;
        lora::Command c{};
        CHECK_FALSE(codec::unpack(f, c));
    }
}

TEST_CASE("List count above capacity is rejected", "[codec]") {
    ser::PeerTable table{};
    table.peer_entries.count = ser::MAX_PEER + 1;
    wirelink::Frame f{};
    CHECK_FALSE(codec::pack(table, f));

    // A received frame claiming too many entries must not decode.
    wirelink::Frame bad{};
    bad.type = static_cast<uint8_t>(ser::PeerTable::TYPE);
    bad.len = 1 + 17 * (ser::MAX_PEER + 1);
    bad.payload.data[0] = ser::MAX_PEER + 1;
    ser::PeerTable out{};
    CHECK_FALSE(codec::unpack(bad, out));
    CHECK(out.peer_entries.count == 0);
}

TEST_CASE("byte-compatible with upstream wirelink::codec", "[codec]") {
    const float nan = std::numeric_limits<float>::quiet_NaN();

    check_upstream_compat(ser::Attitude{0.1f, -0.2f, 0.3f, 0.9f, -1.5f, 2.5f, nan, 0.0f, -9.81f, 1e-3f, 0x3F});
    check_upstream_compat(ser::GPS{-337123456, 1512345678, 0x0123456789ABCDEFull, 4, 11, 0.8f, 1.25f, 359.5f});

    ser::PeerTable table{};
    SECTION("empty peer table") { check_upstream_compat(table); }
    SECTION("full peer table") {
        for (uint8_t i = 0; i < ser::MAX_PEER; ++i) table.peer_entries.items[i] = peer(i);
        table.peer_entries.count = ser::MAX_PEER;
        check_upstream_compat(table);
    }
    SECTION("partial peer table") {
        table.peer_entries.items[0] = peer(1);
        table.peer_entries.items[1] = peer(2);
        table.peer_entries.items[2] = peer(9);
        table.peer_entries.count = 3;
        check_upstream_compat(table);
    }

    check_upstream_compat(ser::OwnScalar{-42.125f});

    ser::Status status{};
    status.boat_id = 5;
    status.gate_state = 1;
    status.fault_flags = 0x8001;
    status.links.items[0] = link(1);
    status.links.items[1] = link(2);
    status.links.count = 2;
    check_upstream_compat(status);

    check_upstream_compat(ser::MotorCommand{1100, 1900});
    check_upstream_compat(ser::BroadcastPayload{-1, -2147483647 - 1, nan});
    check_upstream_compat(ser::Config{3, 2, 9, 125000, 915000000, 1000, 1500, 2000, 1});
    check_upstream_compat(ser::Heartbeat{2});
}

TEST_CASE("pack -> wrap -> unwrap -> unpack", "[codec]") {
    wirelink::Frame f{};
    REQUIRE(codec::pack(lora::Status{9, 1, 1, 2, 0x0100, 271.5f, -80.0f, 9.5f}, f));
    const auto packet = wirelink::framing::wrap(f, 200);
    REQUIRE(packet);
    // Only the two delimiters may be zero.
    CHECK(packet->data[0] == 0);
    CHECK(packet->data[packet->len - 1] == 0);
    for (size_t i = 1; i + 1 < packet->len; ++i) CHECK(packet->data[i] != 0);

    const auto back = wirelink::framing::unwrap(*packet);
    REQUIRE(back);
    CHECK(back->seq == 200);
    CHECK(back->type == f.type);
    lora::Status s{};
    REQUIRE(codec::unpack(*back, s));
    CHECK(s.tx_id == 9);
    CHECK(s.fault_flags == 0x0100);
    CHECK(s.heading_deg == 271.5f);
    CHECK(s.gs_rssi == -80.0f);
    CHECK(s.gs_snr == 9.5f);
}
