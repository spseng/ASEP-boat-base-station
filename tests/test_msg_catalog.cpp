#include <basestation/core/msg_catalog.h>
#include <basestation/proto/base.h>
#include <basestation/proto/codec.h>
#include <basestation/proto/lora.h>

#include <boat_defs/mode.h>

#include <wirelink/msg/serial.h>

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>
#include <string>

using namespace basestation;
using namespace basestation::catalog;
namespace ser = wirelink::msg::serial;
using wirelink::msg::common::LinkStatus;
using wirelink::msg::common::PeerEntry;

namespace {

constexpr float NaN = std::numeric_limits<float>::quiet_NaN();

template <class T>
wirelink::Frame frame_of(T msg) {
    wirelink::Frame f{};
    REQUIRE(codec::pack(msg, f));
    return f;
}

template <class T>
Decoded dec(Proto p, T msg) { return decode(p, frame_of(msg)); }

// "name=value name=value ..." as wl_mon echo prints it.
std::string text(const Decoded& d) {
    std::string s;
    for (const auto& f : d.fields) s += (s.empty() ? "" : " ") + (f.name.empty() ? f.value : f.name + "=" + f.value);
    return s;
}

bool flagged(const Decoded& d, Severity sev, const std::string& what) {
    for (const auto& p : d.problems)
        if (p.severity == sev && p.text.find(what) != std::string::npos) return true;
    return false;
}

bool clean(const Decoded& d) { return d.problems.empty(); }

// Leaves of the struct must match the catalog's name list, so a struct
// changed upstream fails here rather than mislabelling fields.
template <class T>
void check_spec(Proto p) {
    const MsgSpec* s = find(p, static_cast<uint8_t>(T::TYPE));
    REQUIRE(s);
    INFO(s->name);
    CHECK(s->fields.size() == leaf_count<T>());
}

PeerEntry peer(uint8_t id) { return PeerEntry{id, 420000000, -710000000, 0.25f, 120}; }

ser::Attitude attitude(float x, float y, float z, float w) {
    return ser::Attitude{x, y, z, w, 0.01f, 0, 0, 0, 0, 9.81f, 0xFF};
}

ser::GPS gps() { return ser::GPS{423601234, -710589123, 1700000000000ull, 1, 9, 0.9f, 1.5f, 270.0f}; }

ser::Config config() { return ser::Config{3, 2, 9, 125000, 915000000, 1100, 1500, 1900, 1}; }

lora::Status status() {
    return lora::Status{2, 1, 1, 1, 0, 90.0f, -97.5f, 7.5f};
}

}  // namespace

TEST_CASE("every catalog spec matches its struct's leaf count", "[mon]") {
    check_spec<lora::Disable>(Proto::Lora);
    check_spec<lora::SelfStatus>(Proto::Lora);
    check_spec<lora::Command>(Proto::Lora);
    check_spec<lora::Status>(Proto::Lora);
    check_spec<lora::Reenable>(Proto::Lora);
    check_spec<lora::SetMode>(Proto::Lora);
    check_spec<base::RxInfo>(Proto::Lora);
    check_spec<base::BaseStatus>(Proto::Lora);
    check_spec<base::BasePosition>(Proto::Lora);
    CHECK(messages(Proto::Lora).size() == 9);

    check_spec<ser::Attitude>(Proto::Serial);
    check_spec<ser::GPS>(Proto::Serial);
    check_spec<ser::PeerTable>(Proto::Serial);
    check_spec<ser::OwnScalar>(Proto::Serial);
    check_spec<ser::Status>(Proto::Serial);
    check_spec<ser::MotorCommand>(Proto::Serial);
    check_spec<ser::BroadcastPayload>(Proto::Serial);
    check_spec<ser::Config>(Proto::Serial);
    check_spec<ser::Heartbeat>(Proto::Serial);
    // No fields upstream yet; once it gets some, give it a spec.
    CHECK_FALSE(codec::detail::has_fields<ser::ReceivedCommand, LeafCounter>::value);
    CHECK(find(Proto::Serial, static_cast<uint8_t>(ser::ReceivedCommand::TYPE))->fields.empty());
    CHECK(messages(Proto::Serial).size() == 10);

    CHECK(find(Proto::Serial, "attitude") == find(Proto::Serial, uint8_t{1}));
    CHECK(find(Proto::Lora, "nope") == nullptr);
}

TEST_CASE("decode produces readable fields", "[mon]") {
    const Decoded st = dec(Proto::Lora, status());
    CHECK(st.name == "Status");
    CHECK(st.sender == 2);
    CHECK(text(st) == "tx_id=2 mode=AUTONOMOUS armed=ARMED gate=ENABLED faults=none heading=90.0deg "
                      "gs_rssi=-97.5dBm gs_snr=7.5dB");
    CHECK(clean(st));

    const Decoded ss = dec(Proto::Lora, lora::SelfStatus{peer(4)});
    CHECK(ss.sender == 4);
    CHECK(text(ss) == "id=4 lat=42.0000000 lon=-71.0000000 scalar=0.25 age=120ms");

    CHECK(text(dec(Proto::Lora, lora::Disable{255})) == "rx_id=ALL");

    const Decoded att = dec(Proto::Serial, attitude(0, 0, 0.7071f, 0.7071f));
    CHECK(text(att) == "q=(0.00,0.00,0.71,0.71) gyro=(0.01,0.00,0.00) acc=(0.00,0.00,9.81) calib=0xFF");
    CHECK(clean(att));

    ser::PeerTable pt{};
    pt.peer_entries.count = 2;
    pt.peer_entries.items[0] = peer(3);
    pt.peer_entries.items[1] = peer(5);
    const Decoded p = dec(Proto::Serial, pt);
    CHECK(text(p) == "peers=2 [3,42.0000000,-71.0000000,0.25,120ms] [5,42.0000000,-71.0000000,0.25,120ms]");
    CHECK(clean(p));

    ser::Status s{};
    s.boat_id = 3;
    s.gate_state = 1;
    s.fault_flags = boat::mode::fault::GPS_FAILURE | boat::mode::fault::BATTERY_LOW;
    s.links.count = 1;
    s.links.items[0] = LinkStatus{0, -101.0f, -3.5f, 7};
    const Decoded sd = dec(Proto::Serial, s);
    CHECK(text(sd) == "boat_id=3 gate=ENABLED faults=GPS_FAILURE|BATTERY_LOW links=1 [0,-101.0dBm,-3.5dB,7]");
    CHECK(clean(sd));

    // ReceivedCommand has no fields upstream yet: hex, and not a problem.
    wirelink::Frame rc{};
    rc.type = static_cast<uint8_t>(ser::ReceivedCommand::TYPE);
    rc.len = 3;
    rc.payload.data[0] = 0x01;
    rc.payload.data[1] = 0xAB;
    rc.payload.data[2] = 0xFF;
    const Decoded r = decode(Proto::Serial, rc);
    CHECK(r.name == "ReceivedCommand");
    CHECK(text(r) == "payload=01abff");
    CHECK(clean(r));
}

TEST_CASE("unknown types and layout mismatches are reported", "[mon]") {
    wirelink::Frame f{};
    f.type = 0x2A;
    f.len = 2;
    f.payload.data[0] = 0xDE;
    f.payload.data[1] = 0xAD;
    const Decoded u = decode(Proto::Lora, f);
    CHECK_FALSE(u.known);
    CHECK(u.name == "type=0x2A");
    CHECK(text(u) == "len=2 dead");
    CHECK(flagged(u, Severity::Warning, "unknown type"));

    wirelink::Frame longer = frame_of(lora::Command{1, 100, -100});
    ++longer.len;  // one trailing byte
    const Decoded l = decode(Proto::Lora, longer);
    CHECK(flagged(l, Severity::Error, "len 6, expected 5 (layout mismatch?)"));

    wirelink::Frame shorter = frame_of(ser::GPS{});
    --shorter.len;
    CHECK(flagged(decode(Proto::Serial, shorter), Severity::Error, "layout mismatch"));

    // A list count above capacity cannot be unpacked.
    wirelink::Frame big{};
    big.type = static_cast<uint8_t>(ser::PeerTable::TYPE);
    big.len = 1;
    big.payload.data[0] = static_cast<uint8_t>(ser::MAX_PEER + 1);
    CHECK(flagged(decode(Proto::Serial, big), Severity::Error, "layout mismatch"));
}

TEST_CASE("lora and serial decode the same type number differently", "[mon]") {
    const wirelink::Frame st = frame_of(status());
    const Decoded as_lora = decode(Proto::Lora, st);
    const Decoded as_serial = decode(Proto::Serial, st);
    CHECK(as_lora.name == "Status");
    CHECK(clean(as_lora));
    CHECK(as_serial.name == "PeerTable");  // type 3 on the boat's serial link
    CHECK(as_serial.has(Severity::Error));

    const wirelink::Frame ss = frame_of(lora::SelfStatus{peer(1)});
    CHECK(decode(Proto::Lora, ss).name == "SelfStatus");
    CHECK(decode(Proto::Serial, ss).name == "Attitude");
}

TEST_CASE("plausibility: floats", "[mon]") {
    CHECK(clean(dec(Proto::Serial, attitude(0, 0, 0, 1))));
    CHECK(flagged(dec(Proto::Serial, attitude(0.8f, 0, 0, 1.645f)), Severity::Error, "quat norm 1.83"));
    CHECK(flagged(dec(Proto::Serial, attitude(0, 0, 0, NaN)), Severity::Error, "q.w is NaN"));
    ser::Attitude inf = attitude(0, 0, 0, 1);
    inf.lin_acc_z = std::numeric_limits<float>::infinity();
    CHECK(flagged(dec(Proto::Serial, inf), Severity::Error, "acc.z is inf"));

    // NaN is documented as "unknown" for these three.
    lora::Status s = status();
    s.heading_deg = s.gs_rssi = s.gs_snr = NaN;
    CHECK(clean(dec(Proto::Lora, s)));
    CHECK(flagged(dec(Proto::Serial, ser::OwnScalar{NaN}), Severity::Error, "scalar is NaN"));

    CHECK(clean(dec(Proto::Serial, ser::OwnScalar{21.5f})));
    CHECK(flagged(dec(Proto::Serial, ser::OwnScalar{130.0f}), Severity::Error, "scalar 130°C out of range"));
    CHECK(flagged(dec(Proto::Serial, ser::OwnScalar{-60.0f}), Severity::Error, "out of range"));
}

TEST_CASE("plausibility: positions and GPS", "[mon]") {
    CHECK(clean(dec(Proto::Serial, gps())));
    lora::SelfStatus ss{peer(1)};
    ss.self.lat = 900000001;
    CHECK(flagged(dec(Proto::Lora, ss), Severity::Error, "lat 90.0000001 out of range"));
    CHECK(flagged(dec(Proto::Serial, ser::BroadcastPayload{0, -1800000001, 1.0f}), Severity::Error, "lon"));
    CHECK(clean(dec(Proto::Serial, ser::BroadcastPayload{0, 1800000000, 1.0f})));

    ser::GPS g = gps();
    g.lat = g.lon = 0;
    CHECK(flagged(dec(Proto::Serial, g), Severity::Error, "fix at 0,0"));
    g.fix_quality = 0;
    CHECK(clean(dec(Proto::Serial, g)));  // no fix: 0,0 is just "nothing yet"
    CHECK(flagged(dec(Proto::Lora, base::BasePosition{0, 0, 1, 5}), Severity::Error, "fix at 0,0"));

    g = gps();
    g.hdop = -1;
    CHECK(flagged(dec(Proto::Serial, g), Severity::Error, "hdop -1 < 0"));
    g = gps();
    g.satellites = 65;
    CHECK(flagged(dec(Proto::Serial, g), Severity::Error, "sats 65"));
    g = gps();
    g.speed_over_ground = -0.1f;
    CHECK(flagged(dec(Proto::Serial, g), Severity::Error, "sog"));
    g = gps();
    g.course_over_ground = 360.5f;
    CHECK(flagged(dec(Proto::Serial, g), Severity::Error, "cog"));
    g.course_over_ground = 360.0f;
    CHECK(clean(dec(Proto::Serial, g)));
}

TEST_CASE("plausibility: ids, modes, faults", "[mon]") {
    CHECK(flagged(dec(Proto::Lora, lora::SelfStatus{peer(0)}), Severity::Error, "id 0 is not a boat id"));
    CHECK(flagged(dec(Proto::Lora, lora::SelfStatus{peer(11)}), Severity::Error, "not a boat id"));
    CHECK(flagged(dec(Proto::Lora, lora::SelfStatus{peer(255)}), Severity::Error, "not a boat id"));
    CHECK(clean(dec(Proto::Lora, lora::Reenable{255})));  // broadcast makes sense for rx_id
    CHECK(flagged(dec(Proto::Lora, lora::Reenable{0}), Severity::Error, "rx_id 0"));

    lora::Status s = status();
    s.mode = 9;  // maybe newer firmware: only a warning
    Decoded d = dec(Proto::Lora, s);
    CHECK(flagged(d, Severity::Warning, "unknown mode 9"));
    CHECK_FALSE(d.has(Severity::Error));
    CHECK(text(d).find("mode=9?") != std::string::npos);

    s = status();
    s.armed = 2;
    CHECK(flagged(dec(Proto::Lora, s), Severity::Error, "armed 2"));
    s = status();
    s.gate_state = 7;
    CHECK(flagged(dec(Proto::Lora, s), Severity::Error, "gate 7"));
    s = status();
    s.fault_flags = boat::mode::fault::MOTOR_FAILURE | 0x0100;
    d = dec(Proto::Lora, s);
    CHECK(flagged(d, Severity::Warning, "undefined bits 0x0100"));
    CHECK_FALSE(d.has(Severity::Error));

    s = status();
    s.gs_rssi = 5;
    CHECK(flagged(dec(Proto::Lora, s), Severity::Error, "gs_rssi"));
    s = status();
    s.gs_snr = 31;
    CHECK(flagged(dec(Proto::Lora, s), Severity::Error, "gs_snr"));
    s = status();
    s.tx_id = 0;
    CHECK(flagged(dec(Proto::Lora, s), Severity::Error, "tx_id 0"));

    // Serial Status links may include the ground station.
    ser::Status st{};
    st.boat_id = 1;
    st.gate_state = 1;
    st.links.count = 1;
    st.links.items[0] = LinkStatus{0, -90.0f, 5.0f, 0};
    CHECK(clean(dec(Proto::Serial, st)));
    st.links.items[0].id = 200;
    CHECK(flagged(dec(Proto::Serial, st), Severity::Error, "links[0].id 200"));

    CHECK(flagged(dec(Proto::Serial, ser::Heartbeat{42}), Severity::Warning, "unknown mode 42"));
}

TEST_CASE("plausibility: motor command and config", "[mon]") {
    CHECK(clean(dec(Proto::Serial, ser::MotorCommand{1500, 2000})));
    CHECK(flagged(dec(Proto::Serial, ser::MotorCommand{999, 1500}), Severity::Error, "port 999us"));
    CHECK(flagged(dec(Proto::Serial, ser::MotorCommand{1500, 2001}), Severity::Error, "stbd"));

    CHECK(clean(dec(Proto::Serial, config())));
    ser::Config c = config();
    c.lora_bw_hz = 7812;  // 7.8125 kHz, rounded by the firmware: fine
    CHECK(clean(dec(Proto::Serial, c)));
    c.lora_bw_hz = 100000;
    CHECK(flagged(dec(Proto::Serial, c), Severity::Error, "not a LoRa bandwidth"));
    c = config();
    c.pwm_neutral = 1950;
    CHECK(flagged(dec(Proto::Serial, c), Severity::Error, "min<neutral<max"));
    c = config();
    c.lora_sf = 13;
    CHECK(flagged(dec(Proto::Serial, c), Severity::Error, "sf 13"));
    c = config();
    c.lora_freq_hz = 868000000;
    CHECK(flagged(dec(Proto::Serial, c), Severity::Error, "freq 868000000Hz out of range [902000000,928000000]"));
    c = config();
    c.boat_id = 0;
    CHECK(flagged(dec(Proto::Serial, c), Severity::Error, "boat_id 0"));
    c = config();
    c.output_enable = 2;
    CHECK(flagged(dec(Proto::Serial, c), Severity::Error, "output_enable"));

    CHECK(flagged(dec(Proto::Lora, base::RxInfo{-160.0f, 5.0f}), Severity::Error, "rssi"));
    CHECK(clean(dec(Proto::Lora, base::RxInfo{-120.0f, -12.0f})));
}
