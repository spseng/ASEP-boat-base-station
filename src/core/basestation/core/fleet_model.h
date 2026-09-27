#pragma once

// Everything the base station knows about the fleet, built from received
// frames. Single-threaded: owned and updated by the UI thread.

#include <basestation/core/common.h>
#include <basestation/core/event_log.h>
#include <basestation/core/geo.h>
#include <basestation/proto/base.h>
#include <basestation/proto/lora.h>

#include <cmath>
#include <cstdint>
#include <deque>
#include <map>
#include <optional>

namespace basestation {

// Sequence-number accounting for one (sender, message type) stream.
// seq is a uint8 per type per sender, so direction is ambiguous; with
// d = uint8(seq - last):
//   1 .. 256-REORDER_WINDOW : forward, d-1 frames lost
//   0 or > 256-REORDER_WINDOW : duplicate / small reorder, ignored
// Senders start every type at 0, so landing on seq 0 after a jump larger
// than REORDER_WINDOW counts as a restart instead of as lost frames.
struct SeqStats {
    static constexpr int REORDER_WINDOW = 16;

    bool initialised = false;
    uint8_t last = 0;
    uint64_t received = 0;
    uint64_t lost = 0;
    uint64_t duplicates = 0;
    uint64_t restarts = 0;

    void update(uint8_t seq);
    // lost / (received + lost), 0 when nothing received.
    double loss_ratio() const;
};

struct Sample {
    double t;  // seconds since FleetModel start
    float v;
};

struct TrailPoint {
    double t;  // seconds since FleetModel start
    double lat_deg;
    double lon_deg;
};

struct BoatState {
    uint8_t id = 0;
    Clock::time_point first_heard{};
    Clock::time_point last_heard{};  // any frame from this boat

    // From lora::SelfStatus.
    std::optional<lora::SelfStatus> self_status;
    Clock::time_point self_status_time{};

    // From lora::Status.
    std::optional<lora::Status> status;
    Clock::time_point status_time{};

    // From base::RxInfo attached to this boat's frames (base station's view).
    std::optional<base::RxInfo> rx_info;

    // Course over ground derived from the trail (NaN until the boat has moved
    // more than a couple of metres). Used when Status.heading_deg is NaN.
    float derived_course_deg = NAN;

    std::map<uint8_t, SeqStats> seq;  // key: lora::MsgType
    uint64_t frames = 0;

    std::deque<TrailPoint> trail;
    std::deque<Sample> scalar_history;
    std::deque<Sample> rx_rssi_history;  // base station's RSSI of this boat
    std::deque<Sample> rx_snr_history;
    std::deque<Sample> gs_rssi_history;  // this boat's RSSI of the base station

    // Aggregate loss over every type from this boat.
    SeqStats total_seq() const;
    // Best available heading: Status.heading_deg, else derived course, else NaN.
    float heading_deg() const;
};

struct BaseStationState {
    std::optional<base::BaseStatus> status;
    Clock::time_point status_time{};
    uint32_t reboots = 0;  // uptime_ms went backwards

    // From base::BasePosition (only if the base ESP32 has a GPS).
    std::optional<base::BasePosition> position;
    Clock::time_point position_time{};

    // The last BasePosition had a fix and a plausible position.
    bool has_fix() const;
};

// Where the base station is, and who said so. The GPS (BasePosition with a
// fix) wins over the position the operator set by hand, unless the operator
// pins the manual one; a GPS fix older than max_gps_age_s no longer counts.
// A GPS fix cached from an earlier session is the last resort.
struct BasePositionChoice {
    enum class Source { None, Manual, Gps, CachedGps };
    Source source = Source::None;
    geo::LatLon pos{};
    bool known() const { return source != Source::None; }
};

BasePositionChoice choose_base_position(const BaseStationState& base, Clock::time_point now,
                                        const std::optional<geo::LatLon>& manual, bool pin_manual,
                                        const std::optional<geo::LatLon>& cached_gps,
                                        double max_gps_age_s = 10.0);
const char* to_string(BasePositionChoice::Source s);

// (0, 0) is what a GPS without a fix reports; anything else in range is
// accepted.
bool plausible_position(const geo::LatLon& p);

class FleetModel {
public:
    struct Counters {
        uint64_t frames = 0;
        uint64_t decode_errors = 0;   // known type, payload failed to unpack
        uint64_t unknown_types = 0;   // type not in lora:: or base::
        uint64_t land_frames = 0;     // land->boat types heard (another station, or echo)
        uint64_t orphan_rx_info = 0;  // RxInfo not directly after a boat frame
        uint64_t invalid_ids = 0;     // boat id outside BOAT_ID_MIN..MAX
    };

    struct Limits {
        double history_seconds = 900.0;  // plots and trails
        size_t max_trail_points = 20000;
        double trail_min_step_m = 0.2;   // skip trail points closer than this
    };

    explicit FleetModel(EventLog* events = nullptr, Clock::time_point t0 = Clock::now());

    // Decodes and applies one frame. Emits events for notable transitions:
    // first frame from a boat, gate tripped/cleared, mode or armed change,
    // fault bits set/cleared, base-station reboot.
    void ingest(const RxFrame& rx);

    const std::map<uint8_t, BoatState>& boats() const { return boats_; }
    const BaseStationState& base_station() const { return base_; }
    const Counters& counters() const { return counters_; }

    Limits& limits() { return limits_; }
    const Limits& limits() const { return limits_; }

    Clock::time_point t0() const { return t0_; }
    double seconds(Clock::time_point t) const {
        return std::chrono::duration<double>(t - t0_).count();
    }

    void clear();  // forget everything (keeps t0 and limits)

private:
    void attach_rx_info(const RxFrame& rx, const base::RxInfo& info);
    BoatState& boat(uint8_t id, Clock::time_point t);
    void prune(BoatState& b, double now_s);

    EventLog* events_;
    Clock::time_point t0_;
    Limits limits_{};
    Counters counters_{};
    std::map<uint8_t, BoatState> boats_;
    BaseStationState base_{};
    std::optional<uint8_t> last_boat_frame_sender_;  // for RxInfo association
};

}  // namespace basestation
