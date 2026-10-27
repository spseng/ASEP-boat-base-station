#include <basestation/core/site_config.h>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>

using namespace basestation;
using Catch::Approx;

TEST_CASE("site: datum and lookout origin are read from the ROS parameter file") {
    const std::string yaml = R"(
lookout:
  ros__parameters:
    origin_lat: 43.194761
    origin_lon: -71.578504

navsat_transform:
  ros__parameters:
    datum: [43.194761, -71.578504, 0.0]
)";
    std::string err;
    const auto cfg = site::parse(yaml, &err);
    REQUIRE(cfg);
    REQUIRE(cfg->datum);
    REQUIRE(cfg->lookout_origin);
    CHECK(cfg->datum->lat_deg == Approx(43.194761));
    CHECK(cfg->datum->lon_deg == Approx(-71.578504));
    CHECK(cfg->warnings.empty());
    REQUIRE(cfg->origin());
    CHECK(cfg->origin()->lat_deg == Approx(43.194761));
}

TEST_CASE("site: the datum wins over lookout's origin, and a mismatch is reported") {
    const std::string yaml = R"(
lookout:
  ros__parameters:
    origin_lat: 43.1950
    origin_lon: -71.5785
navsat_transform:
  ros__parameters:
    datum: [43.1940, -71.5785, 0.0]
)";
    const auto cfg = site::parse(yaml, nullptr);
    REQUIRE(cfg);
    CHECK(cfg->origin()->lat_deg == Approx(43.1940));
    REQUIRE(cfg->warnings.size() == 1);
    CHECK(cfg->warnings[0].find("111") != std::string::npos);  // ~111 m apart
}

TEST_CASE("site: either origin alone is enough") {
    const auto only_lookout = site::parse("lookout:\n  ros__parameters:\n    origin_lat: 1.5\n    origin_lon: 2.5\n", nullptr);
    REQUIRE(only_lookout);
    CHECK_FALSE(only_lookout->datum);
    CHECK(only_lookout->origin()->lon_deg == Approx(2.5));

    const auto only_datum = site::parse("navsat_transform:\n  ros__parameters:\n    datum: [1.5, 2.5, 0.0]\n", nullptr);
    REQUIRE(only_datum);
    CHECK_FALSE(only_datum->lookout_origin);
    CHECK(only_datum->origin()->lat_deg == Approx(1.5));
}

TEST_CASE("site: unusable files are rejected with a reason") {
    std::string err;
    CHECK_FALSE(site::parse("", &err));
    CHECK_FALSE(err.empty());
    CHECK_FALSE(site::parse("swarm_captain:\n  ros__parameters:\n    k_descent: 1.0\n", &err));
    CHECK_FALSE(site::parse("navsat_transform:\n  ros__parameters:\n    datum: [43.19, -71.57]\n", &err));
    CHECK(err.find("datum") != std::string::npos);
    CHECK_FALSE(site::parse("navsat_transform:\n  ros__parameters:\n    datum: [143.0, -71.57, 0.0]\n", &err));
    CHECK_FALSE(site::parse("lookout:\n  ros__parameters:\n    origin_lat: 43.19\n", &err));
    CHECK_FALSE(site::parse("lookout:\n  ros__parameters:\n    origin_lat: north\n    origin_lon: 1\n", &err));
    CHECK_FALSE(site::parse("navsat_transform: [unclosed\n", &err));
    CHECK_FALSE(site::load("/nonexistent/site.yaml", &err));
}

#ifdef ASEP_SITE_FILE
TEST_CASE("site: the boats' own site file loads with a consistent origin") {
    std::string err;
    const auto cfg = site::load(ASEP_SITE_FILE, &err);
    INFO(err);
    REQUIRE(cfg);
    CHECK(cfg->datum);
    CHECK(cfg->warnings.empty());
}
#endif

namespace {
const char* const ORIGIN = "navsat_transform:\n  ros__parameters:\n    datum: [43.19, -71.57, 0.0]\n";
}

TEST_CASE("site: scalar_source's field arrays give a sum of Gaussians") {
    const std::string yaml = std::string(ORIGIN) + R"(
scalar_source:
  ros__parameters:
    field_x:     [0.0, 30.0]
    field_y:     [0.0, -20.0]
    field_amp:   [-10.0, 5.0]
    field_sigma: [10.0, 1e1]
)";
    std::string err;
    const auto cfg = site::parse(yaml, &err);
    REQUIRE(cfg);
    CHECK(cfg->warnings.empty());
    REQUIRE(cfg->field);
    REQUIRE(cfg->field->gaussians.size() == 2);
    const site::ScalarField& f = *cfg->field;
    // Same expression as the simulator's Field::get_scalar.
    CHECK(f.value(0, 0) == Approx(-10.0 + 5.0 * std::exp(-(900.0 + 400.0) / 200.0)));
    CHECK(f.value(30, -20) == Approx(5.0 - 10.0 * std::exp(-1300.0 / 200.0)));
    CHECK(f.value(1000, 1000) == Approx(0.0).margin(1e-12));
}

TEST_CASE("site: no field arrays means no field and no warning") {
    const auto cfg = site::parse(ORIGIN, nullptr);
    REQUIRE(cfg);
    CHECK_FALSE(cfg->field);
    CHECK(cfg->warnings.empty());
}

TEST_CASE("site: a malformed field is left out with a warning, the origin still loads") {
    auto check_rejected = [](const std::string& params, const char* expect) {
        const std::string yaml = std::string(ORIGIN) + "scalar_source:\n  ros__parameters:\n" + params;
        const auto cfg = site::parse(yaml, nullptr);
        REQUIRE(cfg);
        CHECK(cfg->datum);
        CHECK_FALSE(cfg->field);
        REQUIRE(cfg->warnings.size() == 1);
        INFO(cfg->warnings[0]);
        CHECK(cfg->warnings[0].find(expect) != std::string::npos);
    };
    check_rejected("    field_x: [0.0]\n    field_y: [0.0]\n    field_amp: [1.0]\n", "needs all");
    check_rejected("    field_x: [0.0, 1.0]\n    field_y: [0.0]\n    field_amp: [1.0]\n    field_sigma: [1.0]\n",
                   "differ in length");
    // ROS would read [10, 20] as an integer array.
    check_rejected("    field_x: [10, 20.0]\n    field_y: [0.0, 0.0]\n    field_amp: [1.0, 1.0]\n"
                   "    field_sigma: [1.0, 1.0]\n",
                   "decimal point");
    check_rejected("    field_x: [0.0]\n    field_y: [0.0]\n    field_amp: [1.0]\n    field_sigma: [0.0]\n",
                   "positive");
    check_rejected("    field_x: [abc]\n    field_y: [0.0]\n    field_amp: [1.0]\n    field_sigma: [1.0]\n",
                   "not a number");
    check_rejected("    field_x: []\n    field_y: []\n    field_amp: []\n    field_sigma: []\n", "empty");
}
