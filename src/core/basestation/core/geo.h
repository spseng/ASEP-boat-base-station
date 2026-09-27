#pragma once

// Local-level (north/east) frame around an origin, equirectangular
// approximation. Error is well under a metre at the few-hundred-metre scale
// the fleet operates at.

#include <cstdint>

namespace basestation::geo {

constexpr double EARTH_RADIUS_M = 6371008.8;  // mean radius

struct LatLon {
    double lat_deg = 0;
    double lon_deg = 0;
};

struct NorthEast {
    double north_m = 0;
    double east_m = 0;
};

LatLon from_e7(int32_t lat_e7, int32_t lon_e7);

class LocalFrame {
public:
    LocalFrame() = default;
    explicit LocalFrame(LatLon origin);

    LatLon origin() const { return origin_; }
    NorthEast to_local(LatLon p) const;
    LatLon to_geo(NorthEast p) const;

private:
    LatLon origin_{};
    double cos_lat0_ = 1.0;
};

// Course from a to b, degrees true in [0, 360).
double course_deg(NorthEast a, NorthEast b);

// Great-circle-free planar distance in the local frame, metres.
double distance_m(NorthEast a, NorthEast b);

}  // namespace basestation::geo
