#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <basestation/core/geo.h>

#include <cmath>

using namespace basestation::geo;
using Catch::Matchers::WithinAbs;

namespace {
constexpr double PI = 3.14159265358979323846;
}

TEST_CASE("geo: from_e7 converts 1e-7 degrees", "[model][geo]") {
    const LatLon p = from_e7(356812345, 1397671234);
    CHECK_THAT(p.lat_deg, WithinAbs(35.6812345, 1e-9));
    CHECK_THAT(p.lon_deg, WithinAbs(139.7671234, 1e-9));
    const LatLon n = from_e7(-338688000, -1512093000);
    CHECK_THAT(n.lat_deg, WithinAbs(-33.8688, 1e-9));
    CHECK_THAT(n.lon_deg, WithinAbs(-151.2093, 1e-9));
}

TEST_CASE("geo: known offsets", "[model][geo]") {
    const LatLon origin{35.0, 139.0};
    const LocalFrame lf(origin);

    // 0.001 deg of latitude is ~111.2 m everywhere.
    const NorthEast n = lf.to_local({35.001, 139.0});
    CHECK_THAT(n.north_m, WithinAbs(111.195, 0.01));
    CHECK_THAT(n.east_m, WithinAbs(0.0, 1e-9));

    // Longitude shrinks with cos(latitude).
    const NorthEast e = lf.to_local({35.0, 139.001});
    CHECK_THAT(e.north_m, WithinAbs(0.0, 1e-9));
    CHECK_THAT(e.east_m, WithinAbs(111.195 * std::cos(35.0 * PI / 180.0), 0.01));

    const NorthEast o = lf.to_local(origin);
    CHECK(o.north_m == 0.0);
    CHECK(o.east_m == 0.0);

    // At the equator a degree of longitude is as long as one of latitude.
    const LocalFrame eq({0.0, 0.0});
    CHECK_THAT(eq.to_local({0.0, 0.001}).east_m, WithinAbs(111.195, 0.01));
}

TEST_CASE("geo: to_local / to_geo round trip", "[model][geo]") {
    const LocalFrame lf({-33.86, 151.21});
    for (const NorthEast p : {NorthEast{0, 0}, NorthEast{123.4, -56.7}, NorthEast{-500, 800},
                              NorthEast{0.01, 0.02}}) {
        const LatLon g = lf.to_geo(p);
        const NorthEast back = lf.to_local(g);
        CHECK_THAT(back.north_m, WithinAbs(p.north_m, 1e-6));
        CHECK_THAT(back.east_m, WithinAbs(p.east_m, 1e-6));
    }
    CHECK(lf.origin().lat_deg == -33.86);
    CHECK(lf.origin().lon_deg == 151.21);
}

TEST_CASE("geo: course_deg is degrees true in [0, 360)", "[model][geo]") {
    const NorthEast o{};
    CHECK_THAT(course_deg(o, {10, 0}), WithinAbs(0.0, 1e-9));
    CHECK_THAT(course_deg(o, {0, 10}), WithinAbs(90.0, 1e-9));
    CHECK_THAT(course_deg(o, {-10, 0}), WithinAbs(180.0, 1e-9));
    CHECK_THAT(course_deg(o, {0, -10}), WithinAbs(270.0, 1e-9));
    CHECK_THAT(course_deg(o, {1, 1}), WithinAbs(45.0, 1e-9));
    CHECK_THAT(course_deg(o, {-1, 1}), WithinAbs(135.0, 1e-9));
    CHECK_THAT(course_deg(o, {-1, -1}), WithinAbs(225.0, 1e-9));
    CHECK_THAT(course_deg(o, {1, -1}), WithinAbs(315.0, 1e-9));
    // Relative to a, not the origin.
    CHECK_THAT(course_deg({5, 5}, {5, 10}), WithinAbs(90.0, 1e-9));
    // Just west of north stays below 360.
    const double c = course_deg(o, {1, -1e-12});
    CHECK(c >= 0.0);
    CHECK(c < 360.0);
}

TEST_CASE("geo: distance_m", "[model][geo]") {
    CHECK_THAT(distance_m({0, 0}, {3, 4}), WithinAbs(5.0, 1e-12));
    CHECK_THAT(distance_m({1, 1}, {1, 1}), WithinAbs(0.0, 1e-12));
    CHECK_THAT(distance_m({-3, 2}, {0, -2}), WithinAbs(5.0, 1e-12));
}
