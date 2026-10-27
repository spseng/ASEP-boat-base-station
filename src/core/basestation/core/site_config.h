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

// The simulated scalar field (scalar_source: field_x, field_y, field_amp,
// field_sigma): a sum of Gaussians in the boats' frame, metres east / north
// of the origin.
struct Gaussian {
    double east_m = 0;
    double north_m = 0;
    double amp = 0;
    double sigma_m = 1;
};

struct ScalarField {
    std::vector<Gaussian> gaussians;
    double value(double east_m, double north_m) const;
};

struct SiteConfig {
    std::optional<geo::LatLon> datum;           // navsat_transform: datum [lat, lon, yaw]
    std::optional<geo::LatLon> lookout_origin;  // lookout: origin_lat, origin_lon
    std::optional<ScalarField> field;           // absent, or left out with a warning
    std::vector<std::string> warnings;

    // The boats' frame origin: the datum, else lookout's origin.
    std::optional<geo::LatLon> origin() const { return datum ? datum : lookout_origin; }
};

// Datum and lookout origin further apart than this are reported.
constexpr double ORIGIN_MISMATCH_M = 0.05;

// Parses YAML text. Fails (nullopt, *error) on malformed YAML, malformed
// origin values, or when neither origin is present. A malformed field is
// left out with a warning instead.
std::optional<SiteConfig> parse(const std::string& yaml, std::string* error);
std::optional<SiteConfig> load(const std::string& path, std::string* error);

}  // namespace basestation::site
