#pragma once

// Slippy-map tiles: Web-Mercator tile math, tile sources, the on-disk cache
// layout and area-prefetch planning. Pure functions only; the threads that
// load and download tiles live in tile_service.h.
//
// Tile numbering is the usual XYZ scheme (OSM, Esri, Google): at zoom z the
// world is 2^z x 2^z tiles of 256 px, x grows east from the antimeridian and
// y grows south from +85.0511 deg.

#include <basestation/core/geo.h>

#include <cstdint>
#include <string>
#include <tuple>
#include <vector>

namespace basestation::tiles {

constexpr int MAX_ZOOM = 22;
constexpr int TILE_PX = 256;
// Web Mercator is undefined at the poles; tiles stop here.
constexpr double MAX_LAT_DEG = 85.0511287798066;
// Equatorial radius used by Web Mercator (not the mean radius in geo.h).
constexpr double MERCATOR_RADIUS_M = 6378137.0;

struct TileId {
    int z = 0;
    int x = 0;
    int y = 0;
};
inline bool operator==(const TileId& a, const TileId& b) { return a.z == b.z && a.x == b.x && a.y == b.y; }
inline bool operator!=(const TileId& a, const TileId& b) { return !(a == b); }
inline bool operator<(const TileId& a, const TileId& b) {
    return std::tie(a.z, a.x, a.y) < std::tie(b.z, b.x, b.y);
}

// A tile of one source; the unit of caching and loading.
struct TileKey {
    std::string source_id;
    TileId tile;
};
inline bool operator==(const TileKey& a, const TileKey& b) {
    return a.tile == b.tile && a.source_id == b.source_id;
}
inline bool operator<(const TileKey& a, const TileKey& b) {
    return std::tie(a.source_id, a.tile) < std::tie(b.source_id, b.tile);
}

// Geographic box in degrees. west <= east (boxes never cross the antimeridian
// here: a lake does not).
struct GeoBox {
    double south = 0;
    double west = 0;
    double north = 0;
    double east = 0;
};

// ---- Web-Mercator tile math ---------------------------------------------

// Fractional tile coordinates. Latitude is clamped to +-MAX_LAT_DEG and
// longitude wrapped into [-180, 180).
double lon_to_tile_x(double lon_deg, int z);
double lat_to_tile_y(double lat_deg, int z);
// Inverse: the longitude / latitude of a (fractional) tile edge.
double tile_x_to_lon(double x, int z);
double tile_y_to_lat(double y, int z);

// The tile containing `p` at zoom z (clamped to the valid range).
TileId tile_at(geo::LatLon p, int z);
// The tile's edges in degrees.
GeoBox tile_bounds(TileId t);
// The tile `levels` zooms up that contains `t`, and where `t` sits inside it
// as a fraction of its width / height: [u0, u1] x [v0, v1] (v down, like
// texture coordinates).
TileId ancestor(TileId t, int levels, double* u0 = nullptr, double* v0 = nullptr, double* u1 = nullptr,
                double* v1 = nullptr);

// Ground size of one tile pixel at latitude lat, metres.
double metres_per_pixel(double lat_deg, int z);
// The zoom whose pixels best match `m_per_px` screen metres per pixel,
// clamped to [0, max_zoom].
int zoom_for_resolution(double m_per_px, double lat_deg, int max_zoom);

// ---- Areas and prefetch planning -------------------------------------------

// Inclusive tile index range at one zoom.
struct TileRange {
    int z = 0;
    int x0 = 0, x1 = -1;
    int y0 = 0, y1 = -1;
    uint64_t count() const {
        return x1 < x0 || y1 < y0 ? 0 : uint64_t(x1 - x0 + 1) * uint64_t(y1 - y0 + 1);
    }
};

TileRange tile_range(const GeoBox& box, int z);
// Square box of +-radius_m around a centre (equirectangular, fine for km).
GeoBox box_around(geo::LatLon centre, double radius_m);
// Tiles covering `box` at every zoom in [zmin, zmax], without enumerating.
uint64_t count_tiles(const GeoBox& box, int zmin, int zmax);

struct PrefetchPlan {
    uint64_t count = 0;          // always set, even when refused
    bool ok = false;             // false: `error` says why, `tiles` is empty
    std::string error;
    std::vector<TileId> tiles;   // low zooms first
};

// Default cap on one prefetch job: plenty for one lake at full zoom, and
// small enough that nobody bulk-downloads a city by accident.
constexpr uint64_t DEFAULT_PREFETCH_CAP = 3000;

PrefetchPlan plan_prefetch(const GeoBox& box, int zmin, int zmax, uint64_t cap = DEFAULT_PREFETCH_CAP);

// ---- Tile sources ------------------------------------------------------------

struct TileSource {
    std::string id;           // cache directory name, [a-z0-9-]
    std::string name;         // shown in the UI
    std::string url_template; // {z} {x} {y} are substituted
    std::string ext;          // cache file extension ("png", "jpg")
    std::string attribution;  // must be shown whenever the tiles are
    int max_zoom = 19;
    double avg_tile_kb = 20;  // for download size estimates
    std::string usage_note;   // shown next to the prefetch controls
};

// The built-in sources, in UI order. Adding one is one entry in
// map_tiles.cpp.
const std::vector<TileSource>& builtin_sources();
// nullptr if no built-in source has that id.
const TileSource* find_builtin_source(const std::string& id);

// Id of the "Custom" choice in the UI.
constexpr const char* CUSTOM_SOURCE_ID = "custom";
// A source for a user-entered URL template. Its cache id includes a hash of
// the template, so two custom servers never share cache files.
TileSource custom_source(const std::string& url_template);
// Empty if the template is usable, else what is wrong with it.
std::string check_url_template(const std::string& url_template);

// Substitutes {z}, {x}, {y} in the template.
std::string tile_url(const std::string& url_template, TileId t);

// ---- On-disk cache -------------------------------------------------------------

// <cache_dir>/<source_id>/<z>/<x>/<y>.<ext>
std::string cache_path(const std::string& cache_dir, const std::string& source_id, TileId t,
                       const std::string& ext);

// Cheap magic-number check (PNG, JPEG, GIF, WebP), so an HTML error page
// served with status 200 (captive portal, proxy) never enters the cache.
bool looks_like_image(const uint8_t* data, size_t len);

}  // namespace basestation::tiles
