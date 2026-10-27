#include <basestation/core/site_config.h>

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>

namespace basestation::site {

namespace {

bool valid_lat_lon(double lat, double lon) {
    return std::isfinite(lat) && std::isfinite(lon) && lat >= -90 && lat <= 90 && lon >= -180 && lon <= 180;
}

// node[name]["ros__parameters"], or an undefined node.
YAML::Node params_of(const YAML::Node& root, const char* node_name) {
    if (!root.IsMap()) return {};
    const YAML::Node n = root[node_name];
    if (!n || !n.IsMap()) return {};
    const YAML::Node p = n["ros__parameters"];
    return p && p.IsMap() ? p : YAML::Node{};
}

}  // namespace

std::optional<SiteConfig> parse(const std::string& yaml, std::string* error) {
    auto fail = [&](const std::string& msg) -> std::optional<SiteConfig> {
        if (error) *error = msg;
        return std::nullopt;
    };

    YAML::Node root;
    try {
        root = YAML::Load(yaml);
    } catch (const YAML::Exception& e) {
        return fail(std::string("YAML: ") + e.what());
    }

    SiteConfig cfg;
    try {
        if (const YAML::Node p = params_of(root, "navsat_transform"); p && p["datum"]) {
            const YAML::Node d = p["datum"];
            if (!d.IsSequence() || d.size() != 3) return fail("navsat_transform.datum must be [lat, lon, yaw]");
            const double lat = d[0].as<double>();
            const double lon = d[1].as<double>();
            if (!valid_lat_lon(lat, lon)) return fail("navsat_transform.datum is not a valid latitude/longitude");
            cfg.datum = geo::LatLon{lat, lon};
        }
        if (const YAML::Node p = params_of(root, "lookout"); p && (p["origin_lat"] || p["origin_lon"])) {
            if (!p["origin_lat"] || !p["origin_lon"]) return fail("lookout needs both origin_lat and origin_lon");
            const double lat = p["origin_lat"].as<double>();
            const double lon = p["origin_lon"].as<double>();
            if (!valid_lat_lon(lat, lon)) return fail("lookout origin is not a valid latitude/longitude");
            cfg.lookout_origin = geo::LatLon{lat, lon};
        }
    } catch (const YAML::Exception& e) {
        return fail(std::string("bad value: ") + e.what());
    }

    if (!cfg.datum && !cfg.lookout_origin) return fail("no navsat_transform.datum or lookout origin_lat/origin_lon");
    if (cfg.datum && cfg.lookout_origin) {
        const double d = geo::distance_m({}, geo::LocalFrame(*cfg.datum).to_local(*cfg.lookout_origin));
        if (d > ORIGIN_MISMATCH_M) {
            char buf[160];
            std::snprintf(buf, sizeof buf,
                          "lookout origin is %.2f m from the navsat datum: boats would place peers in a "
                          "different frame from their own position",
                          d);
            cfg.warnings.emplace_back(buf);
        }
    }
    return cfg;
}

std::optional<SiteConfig> load(const std::string& path, std::string* error) {
    std::ifstream in(path);
    if (!in) {
        if (error) *error = "cannot read " + path;
        return std::nullopt;
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    return parse(ss.str(), error);
}

}  // namespace basestation::site
