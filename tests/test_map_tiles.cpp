#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <basestation/core/map_tiles.h>

#include <cmath>
#include <set>

using namespace basestation;
using namespace basestation::tiles;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

TEST_CASE("tile math: known lat/lon -> tile numbers", "[tiles]") {
    // Google Maps API docs: Chicago at zoom 3 is tile (2, 2), pixel (525, 761).
    CHECK(tile_at({41.85, -87.65}, 3) == TileId{3, 2, 2});
    CHECK(std::floor(lon_to_tile_x(-87.65, 3) * TILE_PX) == 525);
    CHECK(std::floor(lat_to_tile_y(41.85, 3) * TILE_PX) == 761);

    // (0, 0) is the corner of the four tiles at zoom 1.
    CHECK(tile_at({0, 0}, 0) == TileId{0, 0, 0});
    CHECK(tile_at({0, 0}, 1) == TileId{1, 1, 1});
    CHECK(tile_at({1e-9, -1e-9}, 1) == TileId{1, 0, 0});

    // OSM wiki formula (deg2num), cross-checked in Python.
    CHECK(tile_at({42.3160, -71.1205}, 16) == TileId{16, 19820, 24250});
    CHECK(tile_at({42.3160, -71.1205}, 19) == TileId{19, 158567, 194003});
    CHECK(tile_at({51.5074, -0.1278}, 10) == TileId{10, 511, 340});
    CHECK(tile_at({-33.8568, 151.2153}, 15) == TileId{15, 30147, 19662});
}

TEST_CASE("tile math: clamping at the edges of the world", "[tiles]") {
    CHECK(tile_at({89.9, 0}, 4).y == 0);
    CHECK(tile_at({-89.9, 0}, 4).y == 15);
    CHECK(tile_at({0, 180.0}, 4).x == 0);  // wraps to -180
    CHECK(tile_at({0, 179.999}, 4).x == 15);
    CHECK(tile_at({0, -180.0}, 4).x == 0);
    CHECK(tile_at({0, 0}, 99).z == MAX_ZOOM);
    CHECK(tile_at({0, 0}, -3).z == 0);
}

TEST_CASE("tile math: round trips", "[tiles]") {
    for (int z : {0, 5, 12, 17, 19, 22}) {
        for (const geo::LatLon p : {geo::LatLon{42.316, -71.1205}, geo::LatLon{-33.86, 151.21},
                                    geo::LatLon{64.1, -21.9}, geo::LatLon{0.001, 0.001}}) {
            INFO("z=" << z << " lat=" << p.lat_deg << " lon=" << p.lon_deg);
            CHECK_THAT(tile_y_to_lat(lat_to_tile_y(p.lat_deg, z), z), WithinAbs(p.lat_deg, 1e-9));
            CHECK_THAT(tile_x_to_lon(lon_to_tile_x(p.lon_deg, z), z), WithinAbs(p.lon_deg, 1e-9));
            // The point lies inside the bounds of its own tile.
            const GeoBox b = tile_bounds(tile_at(p, z));
            CHECK(b.south <= p.lat_deg);
            CHECK(p.lat_deg <= b.north);
            CHECK(b.west <= p.lon_deg);
            CHECK(p.lon_deg <= b.east);
        }
    }
}

TEST_CASE("tile math: bounds", "[tiles]") {
    const GeoBox world = tile_bounds({0, 0, 0});
    CHECK_THAT(world.west, WithinAbs(-180.0, 1e-12));
    CHECK_THAT(world.east, WithinAbs(180.0, 1e-12));
    CHECK_THAT(world.north, WithinAbs(MAX_LAT_DEG, 1e-9));
    CHECK_THAT(world.south, WithinAbs(-MAX_LAT_DEG, 1e-9));

    // Values from the OSM wiki num2deg formula.
    const GeoBox b = tile_bounds({16, 19826, 24242});
    CHECK_THAT(b.north, WithinAbs(42.35042512243458, 1e-10));
    CHECK_THAT(b.west, WithinAbs(-71.092529296875, 1e-10));
    CHECK_THAT(b.south, WithinAbs(42.34636533160187, 1e-10));
    CHECK_THAT(b.east, WithinAbs(-71.0870361328125, 1e-10));

    // Neighbours share edges exactly.
    CHECK(tile_bounds({16, 19827, 24242}).west == b.east);
    CHECK(tile_bounds({16, 19826, 24243}).north == b.south);
}

TEST_CASE("tile math: ancestors and their sub-rectangles", "[tiles]") {
    double u0 = -1, v0 = -1, u1 = -1, v1 = -1;
    const TileId a = ancestor({16, 19821, 24250}, 2, &u0, &v0, &u1, &v1);
    CHECK(a == TileId{14, 4955, 6062});
    // 19821 = 4955 * 4 + 1, 24250 = 6062 * 4 + 2.
    CHECK(u0 == 0.25);
    CHECK(u1 == 0.5);
    CHECK(v0 == 0.5);
    CHECK(v1 == 0.75);
    // The child's bounds match the ancestor's sub-rectangle.
    const GeoBox cb = tile_bounds({16, 19821, 24250});
    const GeoBox ab = tile_bounds(a);
    CHECK_THAT(ab.west + (ab.east - ab.west) * u0, WithinAbs(cb.west, 1e-9));
    CHECK(ancestor({3, 5, 6}, 0) == TileId{3, 5, 6});
    CHECK(ancestor({3, 5, 6}, 10) == TileId{0, 0, 0});  // clamped at zoom 0
}

TEST_CASE("tile math: resolution and zoom choice", "[tiles]") {
    // Zoom 0 at the equator: 40075 km / 256 px.
    CHECK_THAT(metres_per_pixel(0, 0), WithinRel(156543.034, 1e-6));
    CHECK_THAT(metres_per_pixel(42.316, 16), WithinRel(1.766276, 1e-5));
    CHECK(zoom_for_resolution(1.766, 42.316, 19) == 16);
    CHECK(zoom_for_resolution(1.766 / 2, 42.316, 19) == 17);
    CHECK(zoom_for_resolution(1.766 * 1.3, 42.316, 19) == 16);  // rounds, not floors
    CHECK(zoom_for_resolution(0.01, 42.316, 19) == 19);          // clamped to the source
    CHECK(zoom_for_resolution(1e9, 42.316, 19) == 0);
    CHECK(zoom_for_resolution(0, 42.316, 18) == 18);
    CHECK(zoom_for_resolution(NAN, 42.316, 18) == 18);
}

TEST_CASE("area: ranges, counts and prefetch plans", "[tiles]") {
    const geo::LatLon pond{42.3160, -71.1205};
    const GeoBox box = box_around(pond, 500);
    const geo::LocalFrame f(pond);
    CHECK_THAT(f.to_local({box.north, pond.lon_deg}).north_m, WithinAbs(500, 1e-6));
    CHECK_THAT(f.to_local({pond.lat_deg, box.west}).east_m, WithinAbs(-500, 1e-6));

    const TileRange r16 = tile_range(box, 16);
    CHECK(r16.x0 <= 19820);
    CHECK(19820 <= r16.x1);
    CHECK(r16.count() == uint64_t(r16.x1 - r16.x0 + 1) * uint64_t(r16.y1 - r16.y0 + 1));
    // One tile of 256 px at z16 is ~450 m here, so 1 km square spans 3-4.
    CHECK(r16.x1 - r16.x0 + 1 >= 2);
    CHECK(r16.x1 - r16.x0 + 1 <= 4);

    // A point is one tile per zoom.
    const GeoBox point{pond.lat_deg, pond.lon_deg, pond.lat_deg, pond.lon_deg};
    CHECK(count_tiles(point, 0, 19) == 20);
    // Inverted box is empty.
    CHECK(tile_range(GeoBox{1, 1, 0, 0}, 5).count() == 0);

    const uint64_t n = count_tiles(box, 12, 18);
    const PrefetchPlan plan = plan_prefetch(box, 12, 18);
    REQUIRE(plan.ok);
    CHECK(plan.count == n);
    REQUIRE(plan.tiles.size() == n);
    CHECK(plan.tiles.front().z == 12);
    CHECK(plan.tiles.back().z == 18);
    CHECK(std::set<TileId>(plan.tiles.begin(), plan.tiles.end()).size() == n);  // no duplicates
    for (const TileId& t : plan.tiles) {
        const TileRange r = tile_range(box, t.z);
        CHECK((t.x >= r.x0 && t.x <= r.x1 && t.y >= r.y0 && t.y <= r.y1));
    }
}

TEST_CASE("area: prefetch refuses above the cap without enumerating", "[tiles]") {
    const GeoBox city = box_around({42.36, -71.06}, 20000);
    const PrefetchPlan p = plan_prefetch(city, 10, 19);
    CHECK_FALSE(p.ok);
    CHECK(p.tiles.empty());
    CHECK(p.count > DEFAULT_PREFETCH_CAP);
    CHECK(p.error.find("limit of 3000") != std::string::npos);

    const PrefetchPlan exact = plan_prefetch(box_around({42.316, -71.12}, 300), 15, 17, 1000000);
    REQUIRE(exact.ok);
    CHECK(plan_prefetch(box_around({42.316, -71.12}, 300), 15, 17, exact.count).ok);
    CHECK_FALSE(plan_prefetch(box_around({42.316, -71.12}, 300), 15, 17, exact.count - 1).ok);

    CHECK_FALSE(plan_prefetch(city, 5, 3).ok);
    CHECK_FALSE(plan_prefetch(city, -1, 3).ok);
    CHECK_FALSE(plan_prefetch(city, 3, MAX_ZOOM + 1).ok);
}

TEST_CASE("sources: built-ins, custom templates and URLs", "[tiles]") {
    const auto& src = builtin_sources();
    REQUIRE(src.size() >= 2);
    std::set<std::string> ids;
    for (const TileSource& s : src) {
        INFO(s.id);
        CHECK(check_url_template(s.url_template).empty());
        CHECK_FALSE(s.attribution.empty());
        CHECK(s.max_zoom <= MAX_ZOOM);
        CHECK(ids.insert(s.id).second);
        CHECK(find_builtin_source(s.id) == &s);
    }
    CHECK(find_builtin_source("nope") == nullptr);

    // Esri puts y before x.
    const TileSource* esri = find_builtin_source("esri-world-imagery");
    REQUIRE(esri);
    CHECK(tile_url(esri->url_template, {16, 19820, 24250}) ==
          "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/16/24250/19820");
    const TileSource* osm = find_builtin_source("osm");
    REQUIRE(osm);
    CHECK(tile_url(osm->url_template, {3, 2, 5}) == "https://tile.openstreetmap.org/3/2/5.png");

    const TileSource c = custom_source("http://127.0.0.1:8000/{z}/{x}/{y}.jpeg?key=1");
    CHECK(c.id.rfind("custom-", 0) == 0);
    CHECK(c.id.size() == 15);
    CHECK(c.ext == "jpg");
    CHECK(c.attribution == "Tiles: 127.0.0.1:8000");
    CHECK(custom_source("http://a/{z}/{x}/{y}").ext == "img");
    CHECK(custom_source("http://a/{z}/{x}/{y}.png").id != custom_source("http://b/{z}/{x}/{y}.png").id);
    CHECK(custom_source("http://a/{z}/{x}/{y}.png").id == custom_source("http://a/{z}/{x}/{y}.png").id);
    CHECK(tile_url("http://h/{z}/{x}/{y}/{x}", {1, 2, 3}) == "http://h/1/2/3/2");

    CHECK_FALSE(check_url_template("ftp://h/{z}/{x}/{y}").empty());
    CHECK_FALSE(check_url_template("http://h/{z}/{x}").empty());
    CHECK(check_url_template("https://h/{z}/{y}/{x}").empty());
}

TEST_CASE("cache: path layout and image sniffing", "[tiles]") {
    CHECK(cache_path("/tmp/tiles", "osm", {16, 19820, 24250}, "png") == "/tmp/tiles/osm/16/19820/24250.png");
    CHECK(cache_path("/tmp/tiles/", "osm", {0, 0, 0}, "jpg") == "/tmp/tiles/osm/0/0/0.jpg");

    const uint8_t png[] = {0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A, 0};
    const uint8_t jpg[] = {0xFF, 0xD8, 0xFF, 0xE0};
    const uint8_t html[] = {'<', 'h', 't', 'm', 'l', '>', ' ', ' ', ' '};
    CHECK(looks_like_image(png, sizeof png));
    CHECK(looks_like_image(jpg, sizeof jpg));
    CHECK_FALSE(looks_like_image(html, sizeof html));
    CHECK_FALSE(looks_like_image(png, 4));
    CHECK_FALSE(looks_like_image(nullptr, 0));
}
