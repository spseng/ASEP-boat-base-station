#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <basestation/core/fleet_model.h>
#include <basestation/core/geo.h>
#include <basestation/proto/codec.h>

#include <boat_defs/mode.h>
#include <boat_defs/units.h>

#include <cmath>
#include <string>

using namespace basestation;
using Catch::Matchers::WithinAbs;

namespace {

const Clock::time_point T0{};  // fixed epoch keeps the tests deterministic
const geo::LatLon HOME{35.0, 139.0};

Clock::time_point at(double s) {
    return T0 + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(s));
}

template <class T>
RxFrame make_rx(const T& msg, uint8_t seq = 0, double t = 0.0) {
    RxFrame r{at(t), {}};
    REQUIRE(codec::pack(msg, r.frame));
    r.frame.seq = seq;
    return r;
}

// SelfStatus for `id` at a north/east offset (metres) from HOME.
lora::SelfStatus self_at(uint8_t id, double north_m = 0, double east_m = 0, float scalar = 0) {
    const geo::LatLon p = geo::LocalFrame(HOME).to_geo({north_m, east_m});
    lora::SelfStatus s{};
    s.self.id = id;
    s.self.lat = boat::units::deg_to_e7(p.lat_deg);
    s.self.lon = boat::units::deg_to_e7(p.lon_deg);
    s.self.scalar = scalar;
    return s;
}

lora::Status status_of(uint8_t id, boat::mode::Mode mode = boat::mode::Mode::MANUAL,
                       boat::mode::ArmedState armed = boat::mode::ArmedState::DISARMED,
                       boat::mode::GateState gate = boat::mode::GateState::ENABLED,
                       uint16_t faults = 0, float heading = NAN, float gs_rssi = NAN) {
    return lora::Status{id,
                        static_cast<uint8_t>(mode),
                        static_cast<uint8_t>(armed),
                        static_cast<uint8_t>(gate),
                        faults,
                        heading,
                        gs_rssi,
                        NAN};
}

size_t count_events(const EventLog& log, EventLevel level, const std::string& text) {
    size_t n = 0;
    for (const auto& e : log.snapshot())
        if (e.level == level && e.text.find(text) != std::string::npos) ++n;
    return n;
}

}  // namespace

TEST_CASE("SeqStats: normal, wraparound, gaps, duplicates", "[model][fleet]") {
    SeqStats s;
    CHECK(s.loss_ratio() == 0.0);

    for (int q : {254, 255, 0, 1}) s.update(static_cast<uint8_t>(q));
    CHECK(s.initialised);
    CHECK(s.received == 4);
    CHECK(s.lost == 0);
    CHECK(s.restarts == 0);

    SeqStats g;
    g.update(10);
    g.update(14);
    CHECK(g.lost == 3);
    CHECK(g.received == 2);
    CHECK(g.last == 14);
    CHECK_THAT(g.loss_ratio(), WithinAbs(3.0 / 5.0, 1e-12));

    g.update(14);  // exact repeat
    g.update(12);  // small step back (late / duplicate)
    CHECK(g.duplicates == 2);
    CHECK(g.received == 2);
    CHECK(g.last == 14);
    g.update(15);
    CHECK(g.lost == 3);
    CHECK(g.received == 3);

    // Gap across the wrap: 250 -> 2 loses 251..255, 0, 1.
    SeqStats w;
    w.update(250);
    w.update(2);
    CHECK(w.lost == 7);
    CHECK(w.restarts == 0);
}

TEST_CASE("SeqStats: jump to 0 from mid-range counts as a restart", "[model][fleet]") {
    SeqStats s;
    s.update(100);
    s.update(0);
    CHECK(s.restarts == 1);
    CHECK(s.lost == 0);
    CHECK(s.received == 2);
    s.update(1);
    CHECK(s.lost == 0);
    CHECK(s.received == 3);
}

TEST_CASE("FleetModel: first frame creates the boat and logs it once", "[model][fleet]") {
    EventLog log;
    FleetModel m(&log, T0);
    m.ingest(make_rx(self_at(2), 0, 1.0));
    m.ingest(make_rx(self_at(2), 1, 2.0));
    m.ingest(make_rx(status_of(2), 0, 2.5));

    REQUIRE(m.boats().count(2) == 1);
    const BoatState& b = m.boats().at(2);
    CHECK(b.id == 2);
    CHECK(b.first_heard == at(1.0));
    CHECK(b.last_heard == at(2.5));
    CHECK(b.frames == 3);
    CHECK(b.self_status.has_value());
    CHECK(b.self_status_time == at(2.0));
    CHECK(b.status.has_value());
    CHECK(b.seq.at(static_cast<uint8_t>(lora::MsgType::SelfStatus)).received == 2);
    CHECK(b.seq.at(static_cast<uint8_t>(lora::MsgType::Status)).received == 1);
    CHECK(count_events(log, EventLevel::Info, "Boat 2 first heard") == 1);
    CHECK(m.counters().frames == 3);
}

TEST_CASE("FleetModel: per-type seq loss and total_seq", "[model][fleet]") {
    FleetModel m(nullptr, T0);
    for (int q : {254, 255, 0, 1}) m.ingest(make_rx(self_at(1), static_cast<uint8_t>(q)));
    m.ingest(make_rx(status_of(1), 10));
    m.ingest(make_rx(status_of(1), 14));
    m.ingest(make_rx(status_of(1), 14));

    const BoatState& b = m.boats().at(1);
    CHECK(b.seq.at(static_cast<uint8_t>(lora::MsgType::SelfStatus)).lost == 0);
    const SeqStats& st = b.seq.at(static_cast<uint8_t>(lora::MsgType::Status));
    CHECK(st.lost == 3);
    CHECK(st.duplicates == 1);

    const SeqStats total = b.total_seq();
    CHECK(total.initialised);
    CHECK(total.received == 6);
    CHECK(total.lost == 3);
    CHECK(total.duplicates == 1);
    CHECK(BoatState{}.total_seq().received == 0);
}

TEST_CASE("FleetModel: RxInfo attaches only directly after a boat frame", "[model][fleet]") {
    FleetModel m(nullptr, T0);
    const base::RxInfo info{-80.5f, 7.25f};

    m.ingest(make_rx(info));  // nothing before it
    CHECK(m.counters().orphan_rx_info == 1);

    m.ingest(make_rx(self_at(3), 0, 1.0));
    m.ingest(make_rx(info, 0, 1.0));
    const BoatState& b = m.boats().at(3);
    REQUIRE(b.rx_info.has_value());
    CHECK(b.rx_info->rssi == -80.5f);
    CHECK(b.rx_info->snr == 7.25f);
    CHECK(b.rx_rssi_history.size() == 1);
    CHECK(b.rx_snr_history.size() == 1);
    CHECK(m.counters().orphan_rx_info == 1);

    m.ingest(make_rx(info));  // second RxInfo in a row: owner already taken
    CHECK(m.counters().orphan_rx_info == 2);

    m.ingest(make_rx(lora::Disable{3}));  // land frame in between
    m.ingest(make_rx(info));
    CHECK(m.counters().orphan_rx_info == 3);

    m.ingest(make_rx(base::BaseStatus{1000, 1, 0, 0}));
    m.ingest(make_rx(info));
    CHECK(m.counters().orphan_rx_info == 4);

    m.ingest(make_rx(status_of(3), 0, 2.0));
    m.ingest(make_rx(base::RxInfo{-90.0f, -3.0f}, 0, 2.0));
    CHECK(b.rx_info->rssi == -90.0f);
    CHECK(b.rx_rssi_history.size() == 2);
    CHECK(m.counters().orphan_rx_info == 4);
    CHECK(m.counters().land_frames == 1);
}

TEST_CASE("FleetModel: Status transitions produce events", "[model][fleet]") {
    using namespace boat::mode;
    EventLog log;
    FleetModel m(&log, T0);

    m.ingest(make_rx(status_of(4, Mode::MANUAL, ArmedState::DISARMED, GateState::ENABLED)));
    const auto base_events = log.snapshot().size();  // just "first heard"
    CHECK(base_events == 1);

    m.ingest(make_rx(status_of(4, Mode::MANUAL, ArmedState::DISARMED, GateState::TRIPPED), 1));
    CHECK(count_events(log, EventLevel::Warn, "Boat 4 gate TRIPPED") == 1);

    m.ingest(make_rx(status_of(4, Mode::MANUAL, ArmedState::DISARMED, GateState::TRIPPED), 2));
    CHECK(count_events(log, EventLevel::Warn, "gate TRIPPED") == 1);  // no repeat

    m.ingest(make_rx(status_of(4, Mode::MANUAL, ArmedState::DISARMED, GateState::ENABLED), 3));
    CHECK(count_events(log, EventLevel::Info, "Boat 4 gate ENABLED") == 1);

    m.ingest(make_rx(status_of(4, Mode::AUTONOMOUS, ArmedState::ARMED, GateState::ENABLED), 4));
    CHECK(count_events(log, EventLevel::Info, "Boat 4 mode MANUAL -> AUTONOMOUS") == 1);
    CHECK(count_events(log, EventLevel::Info, "Boat 4 ARMED") == 1);

    m.ingest(make_rx(status_of(4, Mode::RETURN_TO_HOME, ArmedState::DISARMED, GateState::ENABLED,
                               fault::GPS_FAILURE | fault::BATTERY_LOW),
                     5));
    CHECK(count_events(log, EventLevel::Info, "mode AUTONOMOUS -> RETURN_TO_HOME") == 1);
    CHECK(count_events(log, EventLevel::Info, "Boat 4 DISARMED") == 1);
    CHECK(count_events(log, EventLevel::Warn, "Boat 4 fault: GPS_FAILURE, BATTERY_LOW") == 1);

    m.ingest(make_rx(status_of(4, Mode::RETURN_TO_HOME, ArmedState::DISARMED, GateState::ENABLED,
                               fault::BATTERY_LOW | fault::MOTOR_FAILURE),
                     6));
    CHECK(count_events(log, EventLevel::Warn, "Boat 4 fault: MOTOR_FAILURE") == 1);
    CHECK(count_events(log, EventLevel::Info, "Boat 4 fault cleared: GPS_FAILURE") == 1);

    // Nothing changed: no new events.
    const auto n = log.snapshot().size();
    m.ingest(make_rx(status_of(4, Mode::RETURN_TO_HOME, ArmedState::DISARMED, GateState::ENABLED,
                               fault::BATTERY_LOW | fault::MOTOR_FAILURE),
                     7));
    CHECK(log.snapshot().size() == n);
}

TEST_CASE("FleetModel: first Status reports a tripped gate and active faults", "[model][fleet]") {
    using namespace boat::mode;
    EventLog log;
    FleetModel m(&log, T0);
    m.ingest(make_rx(status_of(5, Mode::EMERGENCY_STOP, ArmedState::DISARMED, GateState::TRIPPED,
                               fault::SENSOR_FAILURE)));
    CHECK(count_events(log, EventLevel::Warn, "Boat 5 gate TRIPPED") == 1);
    CHECK(count_events(log, EventLevel::Warn, "Boat 5 fault: SENSOR_FAILURE") == 1);
    CHECK(count_events(log, EventLevel::Info, "mode") == 0);
}

TEST_CASE("FleetModel: gs_rssi history skips NaN", "[model][fleet]") {
    using namespace boat::mode;
    FleetModel m(nullptr, T0);
    m.ingest(make_rx(status_of(1), 0, 1.0));
    m.ingest(make_rx(status_of(1, Mode::MANUAL, ArmedState::DISARMED, GateState::ENABLED, 0, NAN, -95.0f), 1,
                     2.0));
    const BoatState& b = m.boats().at(1);
    REQUIRE(b.gs_rssi_history.size() == 1);
    CHECK(b.gs_rssi_history.front().v == -95.0f);
    CHECK_THAT(b.gs_rssi_history.front().t, WithinAbs(2.0, 1e-9));
}

TEST_CASE("FleetModel: invalid boat ids are ignored", "[model][fleet]") {
    FleetModel m(nullptr, T0);
    m.ingest(make_rx(self_at(0)));
    m.ingest(make_rx(self_at(11)));
    m.ingest(make_rx(status_of(255)));
    m.ingest(make_rx(base::RxInfo{-50, 5}));  // must not attach to anything
    CHECK(m.boats().empty());
    CHECK(m.counters().invalid_ids == 3);
    CHECK(m.counters().orphan_rx_info == 1);
    CHECK(m.counters().frames == 4);
}

TEST_CASE("FleetModel: unknown types and decode errors are counted", "[model][fleet]") {
    FleetModel m(nullptr, T0);

    RxFrame unknown{T0, {}};
    unknown.frame.type = 9;
    m.ingest(unknown);
    unknown.frame.type = 0x90;
    m.ingest(unknown);
    CHECK(m.counters().unknown_types == 2);

    RxFrame short_self = make_rx(self_at(1));
    --short_self.frame.len;
    m.ingest(short_self);
    RxFrame long_status = make_rx(status_of(1));
    ++long_status.frame.len;
    m.ingest(long_status);
    RxFrame short_base = make_rx(base::BaseStatus{1, 2, 3, 4});
    short_base.frame.len = 3;
    m.ingest(short_base);
    CHECK(m.counters().decode_errors == 3);
    CHECK(m.boats().empty());
    CHECK(m.counters().frames == 5);
}

TEST_CASE("FleetModel: land->boat frames are counted, not attributed", "[model][fleet]") {
    FleetModel m(nullptr, T0);
    m.ingest(make_rx(lora::Disable{1}));
    m.ingest(make_rx(lora::Command{1, 100, -200}));
    m.ingest(make_rx(lora::Reenable{1}));
    m.ingest(make_rx(lora::SetMode{1, 1, 1}));
    CHECK(m.counters().land_frames == 4);
    CHECK(m.boats().empty());
}

TEST_CASE("FleetModel: base station status and reboot detection", "[model][fleet]") {
    EventLog log;
    FleetModel m(&log, T0);
    m.ingest(make_rx(base::BaseStatus{5000, 10, 1, 3}, 0, 1.0));
    m.ingest(make_rx(base::BaseStatus{6000, 12, 1, 4}, 1, 2.0));
    CHECK(m.base_station().reboots == 0);
    REQUIRE(m.base_station().status.has_value());
    CHECK(m.base_station().status->rx_ok == 12);
    CHECK(m.base_station().status_time == at(2.0));

    m.ingest(make_rx(base::BaseStatus{200, 0, 0, 0}, 0, 3.0));
    CHECK(m.base_station().reboots == 1);
    CHECK(m.base_station().status->uptime_ms == 200);
    CHECK(count_events(log, EventLevel::Warn, "Base station rebooted") == 1);
}

TEST_CASE("FleetModel: histories are pruned by age and trail by count", "[model][fleet]") {
    FleetModel m(nullptr, T0);
    m.limits().history_seconds = 10.0;
    m.limits().max_trail_points = 5;
    for (int i = 0; i <= 20; ++i) {
        // 1 m north per second, so every fix is a new trail point.
        m.ingest(make_rx(self_at(1, i, 0, static_cast<float>(i)), static_cast<uint8_t>(i), i));
        m.ingest(make_rx(base::RxInfo{-70, 3}, 0, i));
    }
    const BoatState& b = m.boats().at(1);
    CHECK(b.scalar_history.front().t >= 10.0);
    CHECK(b.scalar_history.size() == 11);
    CHECK(b.scalar_history.back().v == 20.0f);
    CHECK(b.rx_rssi_history.front().t >= 10.0);
    CHECK(b.rx_snr_history.size() == 11);
    CHECK(b.trail.size() == 5);
    CHECK_THAT(b.trail.back().t, WithinAbs(20.0, 1e-9));
}

TEST_CASE("FleetModel: trail skips steps shorter than trail_min_step_m", "[model][fleet]") {
    FleetModel m(nullptr, T0);
    m.limits().trail_min_step_m = 0.2;
    m.ingest(make_rx(self_at(1, 0.0, 0.0), 0, 0));
    m.ingest(make_rx(self_at(1, 0.1, 0.0), 1, 1));  // too close
    CHECK(m.boats().at(1).trail.size() == 1);
    m.ingest(make_rx(self_at(1, 0.3, 0.0), 2, 2));
    CHECK(m.boats().at(1).trail.size() == 2);
    m.ingest(make_rx(self_at(1, 0.35, 0.05), 3, 3));  // measured from the last point, not the first
    CHECK(m.boats().at(1).trail.size() == 2);

    // No-fix (0, 0) positions never enter the trail.
    lora::SelfStatus nofix{};
    nofix.self.id = 1;
    m.ingest(make_rx(nofix, 4, 4));
    CHECK(m.boats().at(1).trail.size() == 2);
}

TEST_CASE("FleetModel: derived course and heading preference", "[model][fleet]") {
    FleetModel m(nullptr, T0);
    m.ingest(make_rx(self_at(1, 0, 0), 0, 0));
    m.ingest(make_rx(self_at(1, 0, 1), 1, 1));  // 1 m east: not enough baseline
    CHECK(std::isnan(m.boats().at(1).derived_course_deg));
    CHECK(std::isnan(m.boats().at(1).heading_deg()));

    m.ingest(make_rx(self_at(1, 0, 3), 2, 2));
    CHECK_THAT(m.boats().at(1).derived_course_deg, WithinAbs(90.0, 0.5));
    CHECK_THAT(m.boats().at(1).heading_deg(), WithinAbs(90.0, 0.5));

    // Turn south-west.
    m.ingest(make_rx(self_at(1, -3, 0), 3, 3));
    CHECK_THAT(m.boats().at(1).derived_course_deg, WithinAbs(225.0, 0.5));

    // Status heading wins when finite...
    m.ingest(make_rx(status_of(1, boat::mode::Mode::MANUAL, boat::mode::ArmedState::DISARMED,
                               boat::mode::GateState::ENABLED, 0, 12.5f),
                     0, 4));
    CHECK(m.boats().at(1).heading_deg() == 12.5f);
    // ...and derived course is the fallback when it is NaN.
    m.ingest(make_rx(status_of(1), 1, 5));
    CHECK_THAT(m.boats().at(1).heading_deg(), WithinAbs(225.0, 0.5));
}

TEST_CASE("FleetModel: clear forgets everything but t0 and limits", "[model][fleet]") {
    FleetModel m(nullptr, at(5.0));
    m.limits().history_seconds = 42.0;
    m.ingest(make_rx(self_at(1), 0, 6.0));
    m.ingest(make_rx(base::BaseStatus{1, 0, 0, 0}));
    m.clear();
    CHECK(m.boats().empty());
    CHECK(m.counters().frames == 0);
    CHECK_FALSE(m.base_station().status.has_value());
    CHECK(m.limits().history_seconds == 42.0);
    CHECK(m.t0() == at(5.0));
    CHECK_THAT(m.seconds(at(6.0)), WithinAbs(1.0, 1e-9));
    m.ingest(make_rx(base::RxInfo{-50, 5}));
    CHECK(m.counters().orphan_rx_info == 1);
}
