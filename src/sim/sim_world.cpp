#include "sim_world.h"

#include <algorithm>

namespace sim {

namespace {
    constexpr double PI = 3.14159265358979323846;
    constexpr double DEG = 180.0 / PI;

    double wrap360(double d) {
        d = std::fmod(d, 360.0);
        return d < 0 ? d + 360.0 : d;
    }
    double wrap180(double d) {
        d = wrap360(d);
        return d > 180.0 ? d - 360.0 : d;
    }

    // Fraction of the way a first-order system moves toward its target in dt.
    double lag(double dt, double tau) { return 1.0 - std::exp(-dt / tau); }

    // Hotspot of the scalar field (local metres).
    constexpr double HOT_N = 80.0;
    constexpr double HOT_E = -50.0;
    constexpr double HOT_SIGMA = 60.0;

    constexpr double CMD_TIMEOUT_S = 0.5;
    constexpr double WANDER_SPEED = 0.5;  // m/s
    constexpr double MAX_AUTO_YAW = 0.5;  // rad/s
}

World::World(int n, uint32_t seed) : rng_(seed) {
    std::uniform_real_distribution<double> head(0.0, 360.0);
    boats_.resize(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) {
        SimBoat& b = boats_[static_cast<size_t>(i)];
        b.id = static_cast<uint8_t>(i + 1);
        // Spread the fleet on a ring so the boats start apart.
        const double a = 2.0 * PI * i / n + 0.3;
        const double r = 25.0 + 5.0 * i;
        b.north_m = r * std::cos(a);
        b.east_m = r * std::sin(a);
        b.heading_deg = head(rng_);
        b.fix_north_m = b.north_m;
        b.fix_east_m = b.east_m;
        b.wander_phase = a;
    }
    // Faults start late enough not to disturb short automated tests.
    next_fault_s_ = 20.0 + std::uniform_real_distribution<double>(0.0, 20.0)(rng_);
}

bool World::chance(double p) {
    if (p <= 0) return false;
    if (p >= 1) return true;
    return std::uniform_real_distribution<double>(0.0, 1.0)(rng_) < p;
}

float World::scalar_at(double n, double e) {
    const double dn = n - HOT_N, de = e - HOT_E;
    const double v = 18.0 + 4.0 * std::exp(-(dn * dn + de * de) / (2 * HOT_SIGMA * HOT_SIGMA));
    return static_cast<float>(v + std::normal_distribution<double>(0.0, 0.05)(rng_));
}

void World::radio_at(double n, double e, float& rssi, float& snr) {
    // Log-distance path loss; SNR saturates near the receiver, as on SX127x.
    const double d = std::max(3.0, std::hypot(n, e));
    const double r = -35.0 - 22.0 * std::log10(d) + std::normal_distribution<double>(0.0, 1.5)(rng_);
    const double s = 9.5 - (-r - 70.0) * 0.35 + std::normal_distribution<double>(0.0, 0.8)(rng_);
    rssi = static_cast<float>(r);
    snr = static_cast<float>(std::clamp(s, -20.0, 12.0));
}

SimBoat* World::maybe_toggle_fault(double now_s) {
    if (faulted_ >= 0 && now_s >= fault_clear_s_) {
        SimBoat& b = boats_[static_cast<size_t>(faulted_)];
        b.fault_flags &= static_cast<uint16_t>(~boat::mode::fault::GPS_FAILURE);
        b.status_now = true;
        faulted_ = -1;
        return &b;
    }
    if (faulted_ < 0 && now_s >= next_fault_s_ && !boats_.empty()) {
        faulted_ = std::uniform_int_distribution<int>(0, static_cast<int>(boats_.size()) - 1)(rng_);
        SimBoat& b = boats_[static_cast<size_t>(faulted_)];
        b.fault_flags |= boat::mode::fault::GPS_FAILURE;
        b.status_now = true;
        fault_clear_s_ = now_s + std::uniform_real_distribution<double>(5.0, 12.0)(rng_);
        next_fault_s_ = fault_clear_s_ + std::uniform_real_distribution<double>(20.0, 40.0)(rng_);
        return &b;
    }
    return nullptr;
}

void World::step(double now_s) {
    // Cap dt so a stalled process does not teleport the boats.
    const double dt = std::min(0.1, now_s - last_s_);
    last_s_ = now_s;
    if (dt <= 0) return;

    // Slowly wandering water current, bounded to a few cm/s.
    std::normal_distribution<double> walk(0.0, 0.004 * std::sqrt(dt));
    drift_n_ = std::clamp(drift_n_ + walk(rng_), -0.04, 0.04);
    drift_e_ = std::clamp(drift_e_ + walk(rng_), -0.04, 0.04);

    for (SimBoat& b : boats_) integrate(b, dt, now_s);
}

void World::integrate(SimBoat& b, double dt, double now_s) {
    using boat::mode::ArmedState;
    using boat::mode::GateState;
    using boat::mode::Mode;

    double lin = 0, ang = 0;  // targets: m/s, rad/s (+CCW)
    double tau = 0.5;
    const bool can_drive = b.armed == ArmedState::ARMED && b.gate == GateState::ENABLED;

    if (b.mode == Mode::EMERGENCY_STOP) {
        tau = 0.3;
    } else if (!can_drive) {
        tau = 3.0;  // outputs off: the hull coasts to a stop
    } else if (b.mode == Mode::MANUAL) {
        if (now_s - b.cmd_time_s <= CMD_TIMEOUT_S) {
            lin = b.cmd_lin_mm / 1000.0;
            ang = b.cmd_ang_mrad / 1000.0;
        }
    } else {
        // AUTONOMOUS: chase a point ahead on a per-boat circle around the
        // origin. RETURN_TO_HOME: head for the origin and stop there.
        double tn = 0, te = 0, speed = 0;
        if (b.mode == Mode::AUTONOMOUS) {
            const double r = 30.0 + 10.0 * b.id;
            const double a = std::atan2(b.east_m, b.north_m) + (b.id % 2 ? 0.3 : -0.3);
            tn = r * std::cos(a);
            te = r * std::sin(a);
            speed = WANDER_SPEED;
        } else {
            const double d = std::hypot(b.north_m, b.east_m);
            speed = d < 3.0 ? 0.0 : std::min(0.8, 0.2 * d);
        }
        const double want = std::atan2(te - b.east_m, tn - b.north_m) * DEG;
        const double err = wrap180(want - b.heading_deg);  // + = target to the right
        ang = std::clamp(-err / DEG * 1.0, -MAX_AUTO_YAW, MAX_AUTO_YAW);
        lin = speed * std::max(0.2, std::cos(err / DEG));
    }

    b.speed_mps += (lin - b.speed_mps) * lag(dt, tau);
    b.yaw_rate_rps += (ang - b.yaw_rate_rps) * lag(dt, std::min(tau, 0.5));

    // +yaw is CCW, heading is CW from north.
    b.heading_deg -= b.yaw_rate_rps * DEG * dt;
    if (std::fabs(b.speed_mps) < 0.05) {
        // Idle hulls swing a little in the wind.
        b.heading_deg += std::normal_distribution<double>(0.0, 2.0 * std::sqrt(dt))(rng_);
    }
    b.heading_deg = wrap360(b.heading_deg);

    const double h = b.heading_deg / DEG;
    b.north_m += (b.speed_mps * std::cos(h) + drift_n_) * dt;
    b.east_m += (b.speed_mps * std::sin(h) + drift_e_) * dt;

    if (!(b.fault_flags & boat::mode::fault::GPS_FAILURE)) {
        b.fix_north_m = b.north_m;
        b.fix_east_m = b.east_m;
        b.fix_time_s = now_s;
    }
}

}  // namespace sim
