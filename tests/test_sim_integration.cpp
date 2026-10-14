// End-to-end: asep_fake_base on a pty <-> LinkSession + FleetModel + Commander.
// Skipped when the simulator was not built (ASEP_FAKE_BASE_PATH undefined).

#include <catch2/catch_test_macros.hpp>

#include <basestation/core/commander.h>
#include <basestation/core/fleet_model.h>
#include <basestation/core/link_session.h>
#include <basestation/core/serial_port.h>
#include <basestation/proto/msgs.h>

#include <boat_defs/ids.h>
#include <boat_defs/mode.h>
#include <boat_defs/units.h>

#include <wirelink/framing.h>
#include <wirelink/wire.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <string>
#include <thread>
#include <vector>

#include <poll.h>
#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

extern char** environ;

namespace {

using namespace basestation;
using namespace std::chrono_literals;
using boat::mode::ArmedState;
using boat::mode::GateState;
using boat::mode::Mode;

#ifdef ASEP_FAKE_BASE_PATH

// Runs the simulator with stdout on a pipe; kills it on destruction.
class SimProcess {
public:
    explicit SimProcess(std::vector<std::string> args) {
        int fds[2];
        if (::pipe(fds) != 0) return;
        out_ = fds[0];

        posix_spawn_file_actions_t fa;
        posix_spawn_file_actions_init(&fa);
        posix_spawn_file_actions_adddup2(&fa, fds[1], STDOUT_FILENO);
        posix_spawn_file_actions_addclose(&fa, fds[0]);
        posix_spawn_file_actions_addclose(&fa, fds[1]);

        args.insert(args.begin(), ASEP_FAKE_BASE_PATH);
        std::vector<char*> argv;
        for (auto& a : args) argv.push_back(a.data());
        argv.push_back(nullptr);
        if (posix_spawn(&pid_, argv[0], &fa, nullptr, argv.data(), environ) != 0) pid_ = -1;
        posix_spawn_file_actions_destroy(&fa);
        ::close(fds[1]);
    }

    ~SimProcess() {
        if (pid_ > 0) {
            ::kill(pid_, SIGTERM);
            int status = 0;
            ::waitpid(pid_, &status, 0);
        }
        if (out_ >= 0) ::close(out_);
    }

    bool running() const { return pid_ > 0; }

    // Reads one stdout line, waiting at most `timeout`.
    bool read_line(std::string& line, std::chrono::milliseconds timeout) {
        line.clear();
        const auto deadline = Clock::now() + timeout;
        while (Clock::now() < deadline) {
            pollfd p{out_, POLLIN, 0};
            const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now());
            if (::poll(&p, 1, static_cast<int>(std::max<long long>(1, left.count()))) <= 0) continue;
            char c;
            if (::read(out_, &c, 1) != 1) return false;
            if (c == '\n') return true;
            line += c;
        }
        return false;
    }

private:
    pid_t pid_ = -1;
    int out_ = -1;
};

// Pumps frames from the link into the model until `done` or the timeout.
bool pump_until(LinkSession& link, FleetModel& fleet, std::chrono::milliseconds timeout,
                const std::function<bool()>& done, const std::function<void()>& each = {}) {
    const auto deadline = Clock::now() + timeout;
    std::vector<RxFrame> frames;
    while (Clock::now() < deadline) {
        frames.clear();
        link.poll(frames);
        for (const auto& f : frames) fleet.ingest(f);
        if (done()) return true;
        if (each) each();
        std::this_thread::sleep_for(20ms);
    }
    return done();
}

// Local north/east metres of a boat's last SelfStatus (equirectangular).
struct Pos { double n, e; };
Pos position(const BoatState& b) {
    const double lat = boat::units::e7_to_deg(b.self_status->self.lat);
    const double lon = boat::units::e7_to_deg(b.self_status->self.lon);
    constexpr double M_PER_DEG = 6371008.8 * 3.14159265358979323846 / 180.0;
    return {lat * M_PER_DEG, lon * M_PER_DEG * std::cos(lat * 3.14159265358979323846 / 180.0)};
}
double distance(Pos a, Pos b) { return std::hypot(a.n - b.n, a.e - b.e); }

bool all_gates(const FleetModel& fleet, GateState g) {
    if (fleet.boats().size() != 3) return false;
    for (const auto& [id, b] : fleet.boats()) {
        if (!b.self_status || b.state.gate != static_cast<uint8_t>(g)) return false;
    }
    return true;
}

// Reads simulator stdout until a line starting with "TX " (a land frame the
// base station transmitted), or gives up.
std::string next_tx_line(SimProcess& sim) {
    std::string line;
    while (sim.read_line(line, 2000ms)) {
        if (line.rfind("TX ", 0) == 0) return line;
    }
    return "";
}

#endif

}  // namespace

TEST_CASE("simulator drives LinkSession, FleetModel and Commander end to end", "[sim]") {
#ifndef ASEP_FAKE_BASE_PATH
    SKIP("asep_fake_base not built");
#else
    SimProcess sim({"--seed", "1", "--quiet", "--loss", "0", "--corrupt", "0"});
    REQUIRE(sim.running());

    std::string line;
    REQUIRE(sim.read_line(line, 2000ms));
    REQUIRE(line.rfind("PORT ", 0) == 0);
    const std::string port = line.substr(5);

    LinkSession link;
    std::string error;
    REQUIRE(link.open(port, 115200, &error));
    FleetModel fleet;

    // Phase 1: every boat reports SelfStatus with RxInfo attached, and has
    // heard the base station's land slot (Heartbeats while the PC is
    // silent); the base station reports its own status.
    const bool fleet_up = pump_until(link, fleet, 1500ms, [&] {
        if (fleet.boats().size() != 3 || !fleet.base_station().status) return false;
        for (const auto& [id, b] : fleet.boats()) {
            if (!b.self_status || !b.rx_info || !std::isfinite(b.self_status->gs_rssi)) return false;
        }
        return true;
    });
    CHECK(fleet.boats().size() == 3);
    for (uint8_t id = 1; id <= 3; ++id) {
        INFO("boat " << int(id));
        REQUIRE(fleet.boats().count(id) == 1);
        const BoatState& b = fleet.boats().at(id);
        CHECK(b.rx_info.has_value());
        REQUIRE(b.self_status.has_value());
        CHECK(b.state.mode == static_cast<uint8_t>(Mode::MANUAL));
        CHECK_FALSE(b.state.is_armed());
        CHECK_FALSE(b.state.is_tripped());
        CHECK(b.state.reserved == 0);
        CHECK(b.self_status->faults == 0);
        CHECK(std::isfinite(b.self_status->heading_deg));
        CHECK(std::isfinite(b.self_status->gs_rssi));
        CHECK(std::isfinite(b.self_status->gs_snr));
    }
    CHECK(fleet.base_station().status.has_value());
    CHECK(fleet.counters().decode_errors == 0);
    REQUIRE(fleet_up);

    // TDMA: one SelfStatus per boat and one land frame per 100 ms cycle.
    // BaseStatus (1 Hz) counts the land frames; the PC sends nothing yet, so
    // they are the base station's own Heartbeats.
    {
        const base::BaseStatus first = *fleet.base_station().status;
        const uint64_t self1 = fleet.boats().at(1).total_seq().received;
        // Two BaseStatus later (~2 s; their timing jitters by a few ms).
        REQUIRE(pump_until(link, fleet, 3000ms, [&] {
            return fleet.base_station().status->uptime_ms >= first.uptime_ms + 1500;
        }));
        const base::BaseStatus last = *fleet.base_station().status;
        const double secs = (last.uptime_ms - first.uptime_ms) / 1000.0;
        const double land_hz = (last.tx_count - first.tx_count) / secs;
        const double self_hz = static_cast<double>(fleet.boats().at(1).total_seq().received - self1) / secs;
        INFO("land frames " << land_hz << " Hz, boat 1 SelfStatus " << self_hz << " Hz over " << secs << " s");
        CHECK(land_hz > 8.0);
        CHECK(land_hz < 12.0);
        CHECK(self_hz > 7.0);
        CHECK(self_hz < 13.0);
    }

    // Phase 2: arm boat 2 and drive it forward; boat 1 only drifts.
    Commander commander(link);
    REQUIRE(commander.set_mode(2, Mode::MANUAL, ArmedState::ARMED));
    const Pos start1 = position(fleet.boats().at(1));
    const Pos start2 = position(fleet.boats().at(2));
    const auto drive_end = Clock::now() + 1000ms;
    pump_until(link, fleet, 1000ms, [] { return false; }, [&] {
        if (Clock::now() < drive_end) link.send_msg(lora::Command{2, 1000, 0});
    });
    // Let the next SelfStatus (every 100 ms cycle) report the final position.
    pump_until(link, fleet, 300ms, [] { return false; });
    const double moved1 = distance(start1, position(fleet.boats().at(1)));
    const double moved2 = distance(start2, position(fleet.boats().at(2)));
    INFO("boat 1 moved " << moved1 << " m, boat 2 moved " << moved2 << " m");
    CHECK(moved2 > 0.4);
    CHECK(moved2 > moved1 + 0.3);
    CHECK(fleet.boats().at(2).state.is_armed());

    // Phase 3: Disable ALL trips every gate; Reenable ALL clears them.
    const auto disabled_at = Clock::now();
    REQUIRE(commander.disable(boat::ids::BROADCAST_ID));
    CHECK(pump_until(link, fleet, 1000ms, [&] { return all_gates(fleet, GateState::TRIPPED); }));
    // Commander repeats Disable (3 x 100 ms) and Reenable does not cancel the
    // repeats, so wait them out or a late repeat would trip the gates again.
    const auto cfg = commander.config();
    const auto repeats_done = disabled_at + cfg.repeat_interval * cfg.disable_repeats + 50ms;
    pump_until(link, fleet, std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::max(Clock::duration::zero(), repeats_done - Clock::now())),
               [] { return false; });
    REQUIRE(commander.reenable(boat::ids::BROADCAST_ID));
    CHECK(pump_until(link, fleet, 1000ms, [&] { return all_gates(fleet, GateState::ENABLED); }));

    // Nothing lost (loss 0), and the base station never echoes land frames.
    for (const auto& [id, b] : fleet.boats()) CHECK(b.total_seq().lost == 0);
    CHECK(fleet.counters().land_frames == 0);
    CHECK(link.stats().rx_bad == 0);
    link.close();
#endif
}

TEST_CASE("simulator land slot: one frame per cycle, by priority", "[sim]") {
#ifndef ASEP_FAKE_BASE_PATH
    SKIP("asep_fake_base not built");
#else
    // Not --quiet: the simulator prints each land frame it transmits.
    SimProcess sim({"--seed", "3", "--loss", "0", "--corrupt", "0"});
    REQUIRE(sim.running());
    std::string line;
    REQUIRE(sim.read_line(line, 2000ms));
    REQUIRE(line.rfind("PORT ", 0) == 0);
    SerialPort port;
    std::string error;
    REQUIRE(port.open(line.substr(5), 115200, &error));

    // Lowest priority first, in ONE write so the simulator queues all four
    // before its next land slot.
    std::vector<uint8_t> burst;
    auto add = [&](const auto& msg, uint8_t seq) {
        wirelink::Frame f{};
        REQUIRE(wirelink::codec::pack(msg, f));
        const auto pkt = wirelink::framing::wrap(f, seq);
        REQUIRE(pkt);
        burst.insert(burst.end(), pkt->data.begin(), pkt->data.begin() + static_cast<long>(pkt->len));
    };
    add(lora::Command{1, 100, 0}, 0);
    add(lora::Command{1, 200, 0}, 1);
    add(lora::SetMode{2, static_cast<uint8_t>(Mode::MANUAL), 1}, 0);
    add(lora::Disable{3}, 0);
    REQUIRE(port.write_all(burst.data(), burst.size(), 1000, &error));

    const std::string first = next_tx_line(sim);
    const std::string second = next_tx_line(sim);
    const std::string third = next_tx_line(sim);
    INFO(first << "\n" << second << "\n" << third);
    CHECK(first.rfind("TX Disable rx=3 seq=0 heard=3", 0) == 0);
    CHECK(second.rfind("TX SetMode rx=2 mode=MANUAL armed=ARMED seq=0 heard=2", 0) == 0);
    // Only the latest Command goes out.
    CHECK(third.rfind("TX Command rx=1 lin=200mm/s ang=0mrad/s seq=1 heard=1", 0) == 0);
    // Then the base station is idle again: Heartbeats only (not printed).
    std::string more;
    CHECK_FALSE(sim.read_line(more, 400ms));
    port.close();
#endif
}

TEST_CASE("simulator --base-gps reports the base station position", "[sim]") {
#ifndef ASEP_FAKE_BASE_PATH
    SKIP("asep_fake_base not built");
#else
    SimProcess sim({"--seed", "2", "--quiet", "--base-gps", "--lat", "35.5", "--lon", "139.25"});
    REQUIRE(sim.running());
    std::string line;
    REQUIRE(sim.read_line(line, 2000ms));
    REQUIRE(line.rfind("PORT ", 0) == 0);

    LinkSession link;
    std::string error;
    REQUIRE(link.open(line.substr(5), 115200, &error));
    FleetModel fleet;
    // No fix during the first second, then a fix near the origin.
    REQUIRE(pump_until(link, fleet, 3000ms, [&] { return fleet.base_station().has_fix(); }));
    const base::BasePosition& p = *fleet.base_station().position;
    CHECK(std::fabs(boat::units::e7_to_deg(p.lat) - 35.5) < 5e-5);  // ~5 m
    CHECK(std::fabs(boat::units::e7_to_deg(p.lon) - 139.25) < 5e-5);
    CHECK(p.satellites >= 8);
    const auto chosen = choose_base_position(fleet.base_station(), Clock::now(), geo::LatLon{1, 1}, false,
                                             std::nullopt);
    CHECK(chosen.source == BasePositionChoice::Source::Gps);
    CHECK(fleet.counters().decode_errors == 0);
    link.close();
#endif
}
