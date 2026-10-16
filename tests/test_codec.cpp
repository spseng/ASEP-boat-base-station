// The protocol layer on upstream wirelink::codec: every message the base
// station sends or receives (wirelink::msg::lora + basestation::base) must
// round-trip and have the expected wire size; plus msgs.h (state byte) and names.h.

#include <basestation/proto/msgs.h>

#include <wirelink/msg/lora.h>
#include <wirelink/wire.h>

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstring>
#include <limits>
#include <vector>

using namespace basestation;
using wirelink::msg::common::PeerEntry;
namespace codec = wirelink::codec;

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

// Encoded size of a message, in bytes on the wire (not sizeof).
template <class T>
size_t wire_size(const T& msg) {
    wirelink::Frame f{};
    return codec::pack(msg, f) ? f.len : 0;
}

// pack -> unpack -> pack must reproduce the same bytes; returns the decoded
// message for field-level checks.
template <class T>
T round_trip(const T& msg) {
    wirelink::Frame f{};
    REQUIRE(codec::pack(msg, f));
    CHECK(f.type == static_cast<uint8_t>(T::TYPE));
    T out{};
    REQUIRE(codec::unpack(f, out));
    CHECK(encode(out) == payload_bytes(f));
    return out;
}

PeerEntry peer(uint8_t id) {
    return PeerEntry{id, -337123456 - id, 1512345678 + id, 0.25f * id};
}

lora::SelfStatus self_status(uint8_t id) {
    using namespace boat::mode;
    return lora::SelfStatus{peer(id), 271.5f, state::encode(Mode::AUTONOMOUS, ArmedState::ARMED, GateState::ENABLED),
                            fault::GPS_FAILURE, -80.0f, 9.5f};
}

}  // namespace

TEST_CASE("lora messages round-trip", "[codec]") {
    SECTION("Disable") {
        const auto out = round_trip(lora::Disable{0xFF});
        CHECK(out.rx_id == 0xFF);
    }
    SECTION("SelfStatus") {
        const auto out = round_trip(self_status(3));
        CHECK(out.self.id == 3);
        CHECK(out.self.lat == -337123459);
        CHECK(out.self.lon == 1512345681);
        CHECK(out.self.scalar == 0.75f);
        CHECK(out.heading_deg == 271.5f);
        CHECK(out.state == 0x31);
        CHECK(out.faults == boat::mode::fault::GPS_FAILURE);
        CHECK(out.gs_rssi == -80.0f);
        CHECK(out.gs_snr == 9.5f);
    }
    SECTION("SelfStatus with NaN fields") {
        const float nan = std::numeric_limits<float>::quiet_NaN();
        lora::SelfStatus s = self_status(4);
        s.heading_deg = s.gs_rssi = s.gs_snr = nan;
        s.faults = 0xFF;
        const auto out = round_trip(s);
        CHECK(std::isnan(out.heading_deg));
        CHECK(std::isnan(out.gs_rssi));
        CHECK(std::isnan(out.gs_snr));
        CHECK(out.faults == 0xFF);
    }
    SECTION("Command with negative velocities") {
        const auto out = round_trip(lora::Command{2, -1234, std::numeric_limits<int16_t>::min()});
        CHECK(out.rx_id == 2);
        CHECK(out.lin_vel == -1234);
        CHECK(out.ang_vel == std::numeric_limits<int16_t>::min());
    }
    SECTION("Heartbeat") {
        wirelink::Frame f{};
        REQUIRE(codec::pack(lora::Heartbeat{}, f));
        CHECK(f.type == 3);
        CHECK(f.len == 0);
        lora::Heartbeat h{};
        CHECK(codec::unpack(f, h));
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
    const auto pos = round_trip(base::BasePosition{-337123456, 1512345678, 2, 11});
    CHECK(pos.lat == -337123456);
    CHECK(pos.lon == 1512345678);
    CHECK(pos.fix_quality == 2);
    CHECK(pos.satellites == 11);
    CHECK(static_cast<uint8_t>(base::BasePosition::TYPE) == 0x82);
    CHECK(base::is_local_type(static_cast<uint8_t>(base::BasePosition::TYPE)));

    CHECK(base::is_local_type(static_cast<uint8_t>(base::RxInfo::TYPE)));
    CHECK_FALSE(base::is_local_type(static_cast<uint8_t>(lora::SetMode::TYPE)));
}

TEST_CASE("wire sizes", "[codec]") {
    CHECK(wire_size(lora::Disable{}) == 1);
    CHECK(wire_size(lora::SelfStatus{}) == 27);
    CHECK(wire_size(lora::Command{}) == 5);
    CHECK(wire_size(lora::Heartbeat{}) == 0);
    CHECK(wire_size(lora::Reenable{}) == 1);
    CHECK(wire_size(lora::SetMode{}) == 3);
    CHECK(wire_size(base::RxInfo{}) == 8);
    CHECK(wire_size(base::BaseStatus{}) == 16);
    CHECK(wire_size(base::BasePosition{}) == 10);

    // The whole frame (type, seq, len, payload, crc) must fit a LoRa packet.
    for (size_t n : {wire_size(lora::Disable{}), wire_size(lora::SelfStatus{}), wire_size(lora::Command{}),
                     wire_size(lora::Heartbeat{}), wire_size(lora::Reenable{}), wire_size(lora::SetMode{})}) {
        CHECK(n + 5 <= lora::MAX_LORA_PAYLOAD);
    }
}

TEST_CASE("fields are little-endian two's complement", "[codec]") {
    CHECK(encode(lora::Command{0x11, -2, 0x0102}) == std::vector<uint8_t>{0x11, 0xFE, 0xFF, 0x02, 0x01});
    lora::SelfStatus s{};
    s.self = PeerEntry{1, -1, 2, 0.0f};
    s.state = 0x25;
    s.faults = 0x81;
    const auto b = encode(s);
    CHECK(std::vector<uint8_t>(b.begin() + 1, b.begin() + 5) == std::vector<uint8_t>{0xFF, 0xFF, 0xFF, 0xFF});
    // self (13 bytes), heading (4), then state and faults.
    CHECK(b[17] == 0x25);
    CHECK(b[18] == 0x81);
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
    SECTION("Heartbeat with a payload") {
        wirelink::Frame h{};
        REQUIRE(codec::pack(lora::Heartbeat{}, h));
        h.len = 1;
        lora::Heartbeat out{};
        CHECK_FALSE(codec::unpack(h, out));
    }
}

TEST_CASE("pack -> wrap -> unwrap -> unpack", "[codec]") {
    wirelink::Frame f{};
    REQUIRE(codec::pack(self_status(9), f));
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
    lora::SelfStatus s{};
    REQUIRE(codec::unpack(*back, s));
    CHECK(s.self.id == 9);
    CHECK(s.faults == boat::mode::fault::GPS_FAILURE);
    CHECK(s.heading_deg == 271.5f);
    CHECK(s.gs_rssi == -80.0f);
    CHECK(s.gs_snr == 9.5f);
}

// ---------------------------------------------------------------------------
// msgs.h: SelfStatus::state is mode (bits 0-3) | armed (4) | gate (5).
// ---------------------------------------------------------------------------

TEST_CASE("state: encode / decode the SelfStatus state byte", "[codec][state]") {
    using namespace boat::mode;
    CHECK(state::encode(Mode::MANUAL, ArmedState::DISARMED, GateState::TRIPPED) == 0x00);
    CHECK(state::encode(Mode::MANUAL, ArmedState::ARMED, GateState::ENABLED) == 0x30);
    CHECK(state::encode(Mode::EMERGENCY_STOP, ArmedState::DISARMED, GateState::ENABLED) == 0x23);
    CHECK(state::encode(Mode::RETURN_TO_HOME, ArmedState::ARMED, GateState::TRIPPED) == 0x12);
    CHECK(state::encode(uint8_t{0x1F}, ArmedState::DISARMED, GateState::TRIPPED) == 0x0F);  // 4 bits only

    const state::State a = state::decode(0x31);
    CHECK(a.mode == static_cast<uint8_t>(Mode::AUTONOMOUS));
    CHECK(a.armed == static_cast<uint8_t>(ArmedState::ARMED));
    CHECK(a.gate == static_cast<uint8_t>(GateState::ENABLED));
    CHECK(a.reserved == 0);
    CHECK(a.is_armed());
    CHECK_FALSE(a.is_tripped());

    const state::State b = state::decode(0xC9);  // reserved bits, unknown mode 9, disarmed, tripped
    CHECK(b.mode == 9);
    CHECK_FALSE(b.is_armed());
    CHECK(b.is_tripped());
    CHECK(b.reserved == 0xC0);

    for (int raw = 0; raw < 0x40; ++raw) {
        const state::State s = state::decode(static_cast<uint8_t>(raw));
        CHECK(state::encode(s.mode, static_cast<ArmedState>(s.armed), static_cast<GateState>(s.gate)) == raw);
    }
}

// ---------------------------------------------------------------------------
// names.h: display names must degrade gracefully for values this build does
// not know (a boat on newer firmware may report a new mode).
// ---------------------------------------------------------------------------
#include <basestation/proto/names.h>

TEST_CASE("names: known and unknown modes", "[codec][names]") {
    using basestation::names::mode_name;
    CHECK(mode_name(static_cast<uint8_t>(boat::mode::Mode::MANUAL)) == "MANUAL");
    CHECK(mode_name(static_cast<uint8_t>(boat::mode::Mode::EMERGENCY_STOP)) == "EMERGENCY_STOP");
    CHECK(mode_name(42) == "MODE 42");
    CHECK(basestation::names::find_mode(42) == nullptr);
    CHECK(basestation::names::armed_name(7) == "ARMED? 7");
    CHECK(basestation::names::gate_name(0) == "TRIPPED");
}

TEST_CASE("names: every boat_defs mode has an entry", "[codec][names]") {
    for (uint8_t raw = 0; raw <= static_cast<uint8_t>(boat::mode::Mode::EMERGENCY_STOP); ++raw)
        CHECK(basestation::names::find_mode(raw) != nullptr);
}

TEST_CASE("names: fault lists", "[codec][names]") {
    namespace fault = boat::mode::fault;
    CHECK(basestation::names::fault_list(0).empty());
    CHECK(basestation::names::fault_list(fault::GPS_FAILURE | fault::BATTERY_LOW) == "GPS_FAILURE, BATTERY_LOW");
    // LoRa SelfStatus faults are uint8, the serial link's Status faults uint16.
    CHECK(basestation::names::fault_list(uint8_t{fault::MOTOR_FAILURE | 0x80}) == "MOTOR_FAILURE, BIT7");
    CHECK(basestation::names::fault_list(1u << 15) == "BIT15");
}
