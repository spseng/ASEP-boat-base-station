#include <basestation/core/fleet_model.h>

#include <basestation/core/geo.h>
#include <basestation/proto/names.h>

#include <boat_defs/ids.h>
#include <boat_defs/mode.h>

#include <wirelink/wire.h>

#include <string>

namespace basestation {

namespace {

// Derived course needs this much displacement to be meaningful (GPS noise).
constexpr double COURSE_MIN_BASELINE_M = 2.0;
// How far back the trail is searched for that baseline; bounds the per-frame
// cost when a boat jitters in place.
constexpr size_t COURSE_MAX_LOOKBACK = 64;

std::string boat_name(uint8_t id) { return "Boat " + std::to_string(id); }

bool is_boat_id(uint8_t id) {
    return id >= boat::ids::BOAT_ID_MIN && id <= boat::ids::BOAT_ID_MAX;
}

template <class T>
void prune_samples(std::deque<T>& d, double cutoff) {
    while (!d.empty() && d.front().t < cutoff) d.pop_front();
}

}  // namespace

// Plotting (0, 0) would drag a trail across the globe.
bool plausible_position(const geo::LatLon& p) {
    if (p.lat_deg == 0.0 && p.lon_deg == 0.0) return false;
    return p.lat_deg >= -90.0 && p.lat_deg <= 90.0 && p.lon_deg >= -180.0 && p.lon_deg <= 180.0;
}

// ---------------------------------------------------------------------------
// Base station position
// ---------------------------------------------------------------------------

bool BaseStationState::has_fix() const {
    return position && position->fix_quality != 0 &&
           plausible_position(geo::from_e7(position->lat, position->lon));
}

BasePositionChoice choose_base_position(const BaseStationState& base, Clock::time_point now,
                                        const std::optional<geo::LatLon>& manual, bool pin_manual,
                                        const std::optional<geo::LatLon>& cached_gps,
                                        double max_gps_age_s) {
    using Source = BasePositionChoice::Source;
    const bool manual_ok = manual && plausible_position(*manual);
    if (manual_ok && pin_manual) return {Source::Manual, *manual};
    if (base.has_fix() && std::chrono::duration<double>(now - base.position_time).count() <= max_gps_age_s)
        return {Source::Gps, geo::from_e7(base.position->lat, base.position->lon)};
    if (manual_ok) return {Source::Manual, *manual};
    if (cached_gps && plausible_position(*cached_gps)) return {Source::CachedGps, *cached_gps};
    return {};
}

const char* to_string(BasePositionChoice::Source s) {
    switch (s) {
    case BasePositionChoice::Source::None: return "not set";
    case BasePositionChoice::Source::Manual: return "manual";
    case BasePositionChoice::Source::Gps: return "base GPS";
    case BasePositionChoice::Source::CachedGps: return "last base GPS fix (cached)";
    }
    return "?";
}

// ---------------------------------------------------------------------------
// SeqStats
// ---------------------------------------------------------------------------

void SeqStats::update(uint8_t seq) {
    if (!initialised) {
        initialised = true;
        last = seq;
        ++received;
        return;
    }
    const uint8_t delta = static_cast<uint8_t>(seq - last);
    // seq is only 8 bits, so "far backwards" and "far forwards" alias. A
    // short step back (or a repeat) is a duplicate/reorder; everything else
    // is a forward jump.
    if (delta == 0 || delta > 256 - REORDER_WINDOW) {
        ++duplicates;
        return;
    }
    // Senders start every type at seq 0, so landing on 0 after a jump that
    // cannot be an ordinary wrap (last was not near 255) is taken to be a
    // restart rather than 200-odd lost frames.
    if (seq == 0 && delta > REORDER_WINDOW) {
        ++restarts;
    } else {
        lost += static_cast<uint64_t>(delta - 1);
    }
    last = seq;
    ++received;
}

double SeqStats::loss_ratio() const {
    const uint64_t total = received + lost;
    return total == 0 ? 0.0 : static_cast<double>(lost) / static_cast<double>(total);
}

// ---------------------------------------------------------------------------
// BoatState
// ---------------------------------------------------------------------------

SeqStats BoatState::total_seq() const {
    SeqStats t;
    for (const auto& [type, s] : seq) {
        (void)type;
        t.initialised = t.initialised || s.initialised;
        t.received += s.received;
        t.lost += s.lost;
        t.duplicates += s.duplicates;
        t.restarts += s.restarts;
    }
    return t;
}

bool BoatState::has_compass_heading() const {
    return self_status && std::isfinite(self_status->self.heading_deg);
}

float BoatState::heading_deg() const {
    return has_compass_heading() ? self_status->self.heading_deg : derived_course_deg;
}

// ---------------------------------------------------------------------------
// FleetModel
// ---------------------------------------------------------------------------

FleetModel::FleetModel(EventLog* events, Clock::time_point t0) : events_(events), t0_(t0) {}

void FleetModel::clear() {
    counters_ = {};
    boats_.clear();
    base_ = {};
    last_boat_frame_sender_.reset();
}

void FleetModel::reset_counters() {
    counters_ = {};
    last_boat_frame_sender_.reset();
}

BoatState& FleetModel::boat(uint8_t id, Clock::time_point t) {
    auto it = boats_.find(id);
    if (it != boats_.end()) return it->second;
    BoatState& b = boats_[id];
    b.id = id;
    b.first_heard = t;
    if (events_) events_->info(boat_name(id) + " first heard");
    return b;
}

void FleetModel::prune(BoatState& b, double now_s) {
    const double cutoff = now_s - limits_.history_seconds;
    prune_samples(b.trail, cutoff);
    while (b.trail.size() > limits_.max_trail_points) b.trail.pop_front();
    prune_samples(b.scalar_history, cutoff);
    prune_samples(b.rx_rssi_history, cutoff);
    prune_samples(b.rx_snr_history, cutoff);
    prune_samples(b.gs_rssi_history, cutoff);
}

void FleetModel::report_changes(const BoatState& b, const lora::SelfStatus& msg, const state::State& st) {
    if (!events_) return;
    const std::string name = boat_name(b.id);
    // With no previous SelfStatus, only report states that need attention
    // (tripped gate, active faults).
    const state::State* prev = b.self_status ? &b.state : nullptr;
    if (!prev || prev->gate != st.gate) {
        if (st.is_tripped())
            events_->warn(name + " gate TRIPPED");
        else if (prev)
            events_->info(name + " gate ENABLED");
    }
    if (prev && prev->mode != st.mode)
        events_->info(name + " mode " + names::mode_name(prev->mode) + " -> " + names::mode_name(st.mode));
    if (prev && prev->armed != st.armed) events_->info(name + (st.is_armed() ? " ARMED" : " DISARMED"));
    const uint8_t old_faults = b.self_status ? b.self_status->faults : 0;
    const auto set = static_cast<uint8_t>(msg.faults & ~old_faults);
    const auto cleared = static_cast<uint8_t>(old_faults & ~msg.faults);
    if (set) events_->warn(name + " fault: " + names::fault_list(set));
    if (cleared) events_->info(name + " fault cleared: " + names::fault_list(cleared));
}

void FleetModel::attach_rx_info(const RxFrame& rx, const base::RxInfo& info) {
    if (!last_boat_frame_sender_) {
        ++counters_.orphan_rx_info;
        return;
    }
    auto it = boats_.find(*last_boat_frame_sender_);
    if (it == boats_.end()) {
        ++counters_.orphan_rx_info;
        return;
    }
    BoatState& b = it->second;
    const double now_s = seconds(rx.t);
    b.rx_info = info;
    if (std::isfinite(info.rssi)) b.rx_rssi_history.push_back(Sample{now_s, info.rssi});
    if (std::isfinite(info.snr)) b.rx_snr_history.push_back(Sample{now_s, info.snr});
    prune(b, now_s);
}

void FleetModel::ingest(const RxFrame& rx) {
    ++counters_.frames;
    const wirelink::Frame& f = rx.frame;
    const double now_s = seconds(rx.t);

    // Only the frame immediately before an RxInfo may own it, so the
    // association is consumed by whatever frame comes next.
    const std::optional<uint8_t> prev_boat_sender = last_boat_frame_sender_;
    last_boat_frame_sender_.reset();

    if (base::is_local_type(f.type)) {
        switch (static_cast<base::MsgType>(f.type)) {
        case base::MsgType::RxInfo: {
            base::RxInfo info{};
            if (!wirelink::codec::unpack(f, info)) {
                ++counters_.decode_errors;
                return;
            }
            last_boat_frame_sender_ = prev_boat_sender;
            attach_rx_info(rx, info);
            last_boat_frame_sender_.reset();
            return;
        }
        case base::MsgType::BaseStatus: {
            base::BaseStatus st{};
            if (!wirelink::codec::unpack(f, st)) {
                ++counters_.decode_errors;
                return;
            }
            if (base_.status && st.uptime_ms < base_.status->uptime_ms) {
                ++base_.reboots;
                if (events_) events_->warn("Base station rebooted (uptime went backwards)");
            }
            base_.status = st;
            base_.status_time = rx.t;
            return;
        }
        case base::MsgType::BasePosition: {
            base::BasePosition pos{};
            if (!wirelink::codec::unpack(f, pos)) {
                ++counters_.decode_errors;
                return;
            }
            const bool had_fix = base_.has_fix();
            base_.position = pos;
            base_.position_time = rx.t;
            if (events_ && had_fix != base_.has_fix()) {
                if (base_.has_fix())
                    events_->info("Base station GPS fix (" + std::to_string(pos.satellites) + " satellites)");
                else
                    events_->warn("Base station GPS lost its fix");
            }
            return;
        }
        }
        ++counters_.unknown_types;
        return;
    }

    switch (static_cast<lora::MsgType>(f.type)) {
    case lora::MsgType::Disable:
    case lora::MsgType::Command:
    case lora::MsgType::Heartbeat:
    case lora::MsgType::Reenable:
    case lora::MsgType::SetMode:
        ++counters_.land_echoes;
        return;

    case lora::MsgType::SelfStatus: {
        lora::SelfStatus msg{};
        if (!wirelink::codec::unpack(f, msg)) {
            ++counters_.decode_errors;
            return;
        }
        if (!is_boat_id(msg.self.id)) {
            ++counters_.invalid_ids;
            return;
        }
        BoatState& b = boat(msg.self.id, rx.t);
        b.last_heard = rx.t;
        ++b.frames;
        b.seq[f.type].update(f.seq);
        const state::State st = state::decode(msg.state);
        report_changes(b, msg, st);
        b.self_status = msg;
        b.self_status_time = rx.t;
        b.state = st;
        if (std::isfinite(msg.self.scalar)) b.scalar_history.push_back(Sample{now_s, msg.self.scalar});
        if (std::isfinite(msg.gs_rssi)) b.gs_rssi_history.push_back(Sample{now_s, msg.gs_rssi});

        const geo::LatLon p = geo::from_e7(msg.self.lat, msg.self.lon);
        if (plausible_position(p)) {
            if (b.trail.empty()) {
                b.trail.push_back(TrailPoint{now_s, p.lat_deg, p.lon_deg});
            } else {
                const TrailPoint& lastp = b.trail.back();
                const geo::LocalFrame lf({lastp.lat_deg, lastp.lon_deg});
                if (geo::distance_m({}, lf.to_local(p)) >= limits_.trail_min_step_m)
                    b.trail.push_back(TrailPoint{now_s, p.lat_deg, p.lon_deg});
            }
            // Course over ground: from the most recent trail point far enough
            // away that GPS noise does not dominate, to the current fix.
            const geo::LocalFrame here(p);
            b.derived_course_deg = NAN;
            size_t looked = 0;
            for (auto it = b.trail.rbegin(); it != b.trail.rend() && looked < COURSE_MAX_LOOKBACK;
                 ++it, ++looked) {
                const geo::NorthEast q = here.to_local({it->lat_deg, it->lon_deg});
                if (geo::distance_m(q, {}) >= COURSE_MIN_BASELINE_M) {
                    b.derived_course_deg = static_cast<float>(geo::course_deg(q, {}));
                    break;
                }
            }
        }
        prune(b, now_s);
        last_boat_frame_sender_ = msg.self.id;
        return;
    }
    }
    ++counters_.unknown_types;
}

}  // namespace basestation
