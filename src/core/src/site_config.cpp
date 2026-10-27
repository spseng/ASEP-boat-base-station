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

const char* const FIELD_KEYS[] = {"field_x", "field_y", "field_amp", "field_sigma"};

// One of scalar_source's field arrays. ROS types a YAML list by its values:
// [10, 20] is an integer array, which a double-array parameter rejects, and
// a list mixing 10 and 10.5 is rejected outright. So every value must be
// written as a float.
std::optional<std::vector<double>> field_array(const YAML::Node& p, const char* key, std::string* why) {
    const YAML::Node n = p[key];
    if (!n.IsSequence()) {
        *why = std::string(key) + " is not a list";
        return std::nullopt;
    }
    std::vector<double> out;
    for (const YAML::Node& v : n) {
        const std::string& text = v.IsScalar() ? v.Scalar() : std::string();
        double d = 0;
        try {
            d = v.as<double>();
        } catch (const YAML::Exception&) {
            *why = std::string(key) + " has a value that is not a number: '" + text + "'";
            return std::nullopt;
        }
        if (!std::isfinite(d)) {
            *why = std::string(key) + " has a value that is not finite";
            return std::nullopt;
        }
        if (text.find_first_of(".eE") == std::string::npos) {
            *why = std::string(key) + ": write every value with a decimal point ('" + text + "' -> '" + text +
                   ".0'); ROS reads whole numbers as integers";
            return std::nullopt;
        }
        out.push_back(d);
    }
    return out;
}

// scalar_source's field, if its arrays are there. Problems go to *why.
std::optional<ScalarField> parse_field(const YAML::Node& root, std::string* why) {
    const YAML::Node p = params_of(root, "scalar_source");
    if (!p) return std::nullopt;
    int present = 0;
    for (const char* k : FIELD_KEYS) present += p[k] ? 1 : 0;
    if (present == 0) return std::nullopt;
    if (present != 4) {
        *why = "scalar_source needs all of field_x, field_y, field_amp, field_sigma";
        return std::nullopt;
    }
    std::vector<double> a[4];
    for (int i = 0; i < 4; ++i) {
        auto v = field_array(p, FIELD_KEYS[i], why);
        if (!v) return std::nullopt;
        a[i] = std::move(*v);
    }
    if (a[1].size() != a[0].size() || a[2].size() != a[0].size() || a[3].size() != a[0].size()) {
        char buf[160];
        std::snprintf(buf, sizeof buf, "field arrays differ in length (x %zu, y %zu, amp %zu, sigma %zu)", a[0].size(),
                      a[1].size(), a[2].size(), a[3].size());
        *why = buf;
        return std::nullopt;
    }
    if (a[0].empty()) {
        *why = "the field arrays are empty";
        return std::nullopt;
    }
    ScalarField f;
    for (size_t i = 0; i < a[0].size(); ++i) {
        if (a[3][i] <= 0) {
            *why = "field_sigma must be positive";
            return std::nullopt;
        }
        f.gaussians.push_back({a[0][i], a[1][i], a[2][i], a[3][i]});
    }
    return f;
}

}  // namespace

double ScalarField::value(double east_m, double north_m) const {
    double v = 0;
    for (const Gaussian& g : gaussians) {
        const double de = east_m - g.east_m, dn = north_m - g.north_m;
        v += g.amp * std::exp(-(de * de + dn * dn) / (2.0 * g.sigma_m * g.sigma_m));
    }
    return v;
}

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

    std::string why;
    cfg.field = parse_field(root, &why);
    if (!why.empty()) cfg.warnings.push_back("scalar field not used: " + why);
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
