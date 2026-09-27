// End-to-end: asep_fake_base on a pty <-> LinkSession + FleetModel + Commander.
// Skipped when the simulator was not built (ASEP_FAKE_BASE_PATH undefined).

#include <catch2/catch_test_macros.hpp>

#include <basestation/core/commander.h>
#include <basestation/core/fleet_model.h>
#include <basestation/core/link_session.h>

#include <boat_defs/ids.h>
#include <boat_defs/mode.h>
#include <boat_defs/units.h>

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
        if (!b.status || b.status->gate_state != static_cast<uint8_t>(g)) return false;
    }
    return true;
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

    // Phase 1: every boat reports SelfStatus + Status with RxInfo attached,
    // and the base station reports its own status.
    const bool fleet_up = pump_until(link, fleet, 1500ms, [&] {
        if (fleet.boats().size() != 3 || !fleet.base_station().status) return false;
        for (const auto& [id, b] : fleet.boats()) {
            if (!b.self_status || !b.status || !b.rx_info) return false;
        }
        return true;
    });
    CHECK(fleet.boats().size() == 3);
    for (uint8_t id = 1; id <= 3; ++id) {
        INFO("boat " << int(id));
        REQUIRE(fleet.boats().count(id) == 1);
        const BoatState& b = fleet.boats().at(id);
        CHECK(b.self_status.has_value());
        CHECK(b.status.has_value());
        CHECK(b.rx_info.has_value());
        if (b.status) {
            CHECK(b.status->mode == static_cast<uint8_t>(Mode::MANUAL));
            CHECK(b.status->armed == static_cast<uint8_t>(ArmedState::DISARMED));
            CHECK(b.status->gate_state == static_cast<uint8_t>(GateState::ENABLED));
        }
    }
    CHECK(fleet.base_station().status.has_value());
    CHECK(fleet.counters().decode_errors == 0);
    REQUIRE(fleet_up);

    // Phase 2: arm boat 2 and drive it forward; boat 1 only drifts.
    Commander commander(link);
    REQUIRE(commander.set_mode(2, Mode::MANUAL, ArmedState::ARMED));
    const Pos start1 = position(fleet.boats().at(1));
    const Pos start2 = position(fleet.boats().at(2));
    const auto drive_end = Clock::now() + 1000ms;
    pump_until(link, fleet, 1000ms, [] { return false; }, [&] {
        if (Clock::now() < drive_end) link.send_msg(lora::Command{2, 1000, 0});
    });
    // Let the next SelfStatus (3 Hz) report the final position.
    pump_until(link, fleet, 400ms, [] { return false; });
    const double moved1 = distance(start1, position(fleet.boats().at(1)));
    const double moved2 = distance(start2, position(fleet.boats().at(2)));
    INFO("boat 1 moved " << moved1 << " m, boat 2 moved " << moved2 << " m");
    CHECK(moved2 > 0.4);
    CHECK(moved2 > moved1 + 0.3);
    CHECK(fleet.boats().at(2).status->armed == static_cast<uint8_t>(ArmedState::ARMED));

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

    CHECK(link.stats().rx_bad == 0);
    link.close();
#endif
}
