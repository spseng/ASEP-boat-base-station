#pragma once

// The boats' site file: the ROS parameter YAML in ASEP-boat
// (ros/bringup/config/cites/<site>.yaml) that fixes the local frame every
// boat works in. navsat_transform's datum is the origin of /state/odom, and
// lookout's origin_lat/origin_lon must be the same point, or boats compute
// peer positions in different frames. Anchoring the Fleet view at the same
// point makes its East/North read the same as the boats' /state/odom.

#include <basestation/core/geo.h>

#include <optional>
#include <string>
#include <vector>

namespace basestation::site {

struct SiteConfig {
    std::optional<geo::LatLon> datum;           // navsat_transform: datum [lat, lon, yaw]
    std::optional<geo::LatLon> lookout_origin;  // lookout: origin_lat, origin_lon
    std::vector<std::string> warnings;

    // The boats' frame origin: the datum, else lookout's origin.
    std::optional<geo::LatLon> origin() const { return datum ? datum : lookout_origin; }
};

// Datum and lookout origin further apart than this are reported.
constexpr double ORIGIN_MISMATCH_M = 0.05;

// Parses YAML text. Fails (nullopt, *error) on malformed YAML, malformed
// values, or when neither origin is present.
std::optional<SiteConfig> parse(const std::string& yaml, std::string* error);
std::optional<SiteConfig> load(const std::string& path, std::string* error);

}  // namespace basestation::site
