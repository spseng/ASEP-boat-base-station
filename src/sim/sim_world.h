#pragma once

// Simulated fleet for asep_fake_base: boat kinematics, a scalar field and a
// crude radio model. No I/O here; fake_base.cpp owns the pty and framing.

#include <boat_defs/mode.h>

#include <cmath>
#include <cstdint>
#include <random>
#include <vector>

namespace sim {

struct Origin {
    double lat_deg = 42.3160;   // Jamaica Pond, Boston
    double lon_deg = -71.1205;
};

struct SimBoat {
    uint8_t id = 0;

    // Local frame around the origin, metres; heading degrees true (CW from N).
    double north_m = 0;
    double east_m = 0;
    double heading_deg = 0;
    double speed_mps = 0;     // along heading
    double yaw_rate_rps = 0;  // + = CCW (ROS convention), as commanded

    boat::mode::Mode mode = boat::mode::Mode::MANUAL;
    boat::mode::ArmedState armed = boat::mode::ArmedState::DISARMED;
    boat::mode::GateState gate = boat::mode::GateState::ENABLED;
    uint16_t fault_flags = boat::mode::fault::NONE;

    // Last teleop command heard from land.
    int16_t cmd_lin_mm = 0;
    int16_t cmd_ang_mrad = 0;
    double cmd_time_s = -1e9;

    // Uplink quality of the last land frame this boat heard (NaN = none yet).
    float gs_rssi = NAN;
    float gs_snr = NAN;

    // Last good GPS fix, frozen while GPS_FAILURE is set.
    double fix_north_m = 0;
    double fix_east_m = 0;
    double fix_time_s = 0;

    // Per-type tx sequence counters, like a real boat's radio task.
    uint8_t seq[256] = {};

    // TDMA-ish schedule.
    double next_slot_s = 0;
    double next_status_s = 0;
    bool status_now = false;  // state changed: send Status in the next slot

    double wander_phase = 0;
};

class World {
public:
    World(int boats, uint32_t seed);

    // Advances the physics to `now_s` (seconds since start).
    void step(double now_s);

    std::vector<SimBoat>& boats() { return boats_; }

    // Temperature-like scalar at a point, with sensor noise.
    float scalar_at(double north_m, double east_m);

    // Link quality between the origin (base station) and a point.
    void radio_at(double north_m, double east_m, float& rssi, float& snr);

    // Occasionally raises / clears a fault on one boat so the UI has
    // something to show. Returns the boat whose flags changed, or nullptr.
    SimBoat* maybe_toggle_fault(double now_s);

    std::mt19937& rng() { return rng_; }
    bool chance(double p);

private:
    void integrate(SimBoat& b, double dt, double now_s);

    std::vector<SimBoat> boats_;
    std::mt19937 rng_;
    double last_s_ = 0;
    double drift_n_ = 0;  // water current, m/s
    double drift_e_ = 0;
    double next_fault_s_ = 0;
    int faulted_ = -1;  // index into boats_
    double fault_clear_s_ = 0;
};

}  // namespace sim
