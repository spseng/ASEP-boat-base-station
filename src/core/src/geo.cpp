#include <basestation/core/geo.h>

#include <boat_defs/units.h>

#include <cmath>

namespace basestation::geo {

namespace {
constexpr double PI = 3.14159265358979323846;
constexpr double DEG_TO_RAD = PI / 180.0;
constexpr double RAD_TO_DEG = 180.0 / PI;
}  // namespace

LatLon from_e7(int32_t lat_e7, int32_t lon_e7) {
    return LatLon{boat::units::e7_to_deg(lat_e7), boat::units::e7_to_deg(lon_e7)};
}

LocalFrame::LocalFrame(LatLon origin)
    : origin_(origin), cos_lat0_(std::cos(origin.lat_deg * DEG_TO_RAD)) {}

NorthEast LocalFrame::to_local(LatLon p) const {
    double dlon = p.lon_deg - origin_.lon_deg;
    // Take the short way round across the antimeridian.
    if (dlon > 180.0) dlon -= 360.0;
    if (dlon < -180.0) dlon += 360.0;
    return NorthEast{(p.lat_deg - origin_.lat_deg) * DEG_TO_RAD * EARTH_RADIUS_M,
                     dlon * DEG_TO_RAD * EARTH_RADIUS_M * cos_lat0_};
}

LatLon LocalFrame::to_geo(NorthEast p) const {
    const double lat = origin_.lat_deg + p.north_m / EARTH_RADIUS_M * RAD_TO_DEG;
    // cos_lat0_ is ~0 only at the poles, where no boat will be.
    const double lon = origin_.lon_deg + p.east_m / (EARTH_RADIUS_M * cos_lat0_) * RAD_TO_DEG;
    return LatLon{lat, lon};
}

double course_deg(NorthEast a, NorthEast b) {
    // atan2(east, north): 0 = north, 90 = east (clockwise, degrees true).
    double c = std::atan2(b.east_m - a.east_m, b.north_m - a.north_m) * RAD_TO_DEG;
    if (c < 0) c += 360.0;
    if (c >= 360.0) c -= 360.0;
    return c;
}

double distance_m(NorthEast a, NorthEast b) {
    return std::hypot(b.north_m - a.north_m, b.east_m - a.east_m);
}

}  // namespace basestation::geo
