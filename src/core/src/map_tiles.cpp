#include <basestation/core/map_tiles.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>

namespace basestation::tiles {

namespace {

constexpr double PI = 3.14159265358979323846;
constexpr double DEG_TO_RAD = PI / 180.0;
constexpr double RAD_TO_DEG = 180.0 / PI;

double world_tiles(int z) { return std::ldexp(1.0, z); }

int clamp_index(double v, int z) {
    const int n = 1 << z;
    if (!(v >= 0)) return 0;  // also NaN
    if (v >= n) return n - 1;
    return static_cast<int>(v);
}

std::string replace_all(std::string s, const std::string& from, const std::string& to) {
    for (size_t pos = s.find(from); pos != std::string::npos; pos = s.find(from, pos + to.size()))
        s.replace(pos, from.size(), to);
    return s;
}

uint32_t fnv1a(const std::string& s) {
    uint32_t h = 2166136261u;
    for (unsigned char c : s) {
        h ^= c;
        h *= 16777619u;
    }
    return h;
}

}  // namespace

// ---------------------------------------------------------------------------
// Web-Mercator tile math
// ---------------------------------------------------------------------------

double lon_to_tile_x(double lon_deg, int z) {
    double lon = std::fmod(lon_deg + 180.0, 360.0);
    if (lon < 0) lon += 360.0;
    return lon / 360.0 * world_tiles(z);
}

double lat_to_tile_y(double lat_deg, int z) {
    const double lat = std::clamp(lat_deg, -MAX_LAT_DEG, MAX_LAT_DEG) * DEG_TO_RAD;
    return (1.0 - std::asinh(std::tan(lat)) / PI) / 2.0 * world_tiles(z);
}

double tile_x_to_lon(double x, int z) { return x / world_tiles(z) * 360.0 - 180.0; }

double tile_y_to_lat(double y, int z) {
    return std::atan(std::sinh(PI * (1.0 - 2.0 * y / world_tiles(z)))) * RAD_TO_DEG;
}

TileId tile_at(geo::LatLon p, int z) {
    z = std::clamp(z, 0, MAX_ZOOM);
    return TileId{z, clamp_index(lon_to_tile_x(p.lon_deg, z), z), clamp_index(lat_to_tile_y(p.lat_deg, z), z)};
}

GeoBox tile_bounds(TileId t) {
    GeoBox b;
    b.west = tile_x_to_lon(t.x, t.z);
    b.east = tile_x_to_lon(t.x + 1, t.z);
    b.north = tile_y_to_lat(t.y, t.z);
    b.south = tile_y_to_lat(t.y + 1, t.z);
    return b;
}

TileId ancestor(TileId t, int levels, double* u0, double* v0, double* u1, double* v1) {
    levels = std::clamp(levels, 0, t.z);
    const TileId a{t.z - levels, t.x >> levels, t.y >> levels};
    const double size = static_cast<double>(1 << levels);
    const double ox = t.x - (a.x << levels);
    const double oy = t.y - (a.y << levels);
    if (u0) *u0 = ox / size;
    if (u1) *u1 = (ox + 1) / size;
    if (v0) *v0 = oy / size;
    if (v1) *v1 = (oy + 1) / size;
    return a;
}

double metres_per_pixel(double lat_deg, int z) {
    const double lat = std::clamp(lat_deg, -MAX_LAT_DEG, MAX_LAT_DEG) * DEG_TO_RAD;
    return 2.0 * PI * MERCATOR_RADIUS_M * std::cos(lat) / (TILE_PX * world_tiles(z));
}

int zoom_for_resolution(double m_per_px, double lat_deg, int max_zoom) {
    max_zoom = std::clamp(max_zoom, 0, MAX_ZOOM);
    if (!(m_per_px > 0) || !std::isfinite(m_per_px)) return max_zoom;
    // Zoom 0 resolution / wanted resolution = 2^z. Rounding (rather than
    // ceil) magnifies tiles by at most ~1.4x and fetches 4x fewer tiles.
    const double z = std::log2(metres_per_pixel(lat_deg, 0) / m_per_px);
    if (!std::isfinite(z)) return max_zoom;
    return std::clamp(static_cast<int>(std::floor(z + 0.5)), 0, max_zoom);
}

// ---------------------------------------------------------------------------
// Areas and prefetch planning
// ---------------------------------------------------------------------------

TileRange tile_range(const GeoBox& box, int z) {
    TileRange r;
    r.z = std::clamp(z, 0, MAX_ZOOM);
    if (!(box.south <= box.north) || !(box.west <= box.east)) return r;  // empty (also NaN)
    // East edge exactly on 180 must not wrap to tile 0.
    const double east = std::min(box.east, 180.0 - 1e-9);
    const double west = std::max(box.west, -180.0);
    r.x0 = clamp_index(lon_to_tile_x(west, r.z), r.z);
    r.x1 = clamp_index(lon_to_tile_x(east, r.z), r.z);
    r.y0 = clamp_index(lat_to_tile_y(box.north, r.z), r.z);
    r.y1 = clamp_index(lat_to_tile_y(box.south, r.z), r.z);
    return r;
}

GeoBox box_around(geo::LatLon centre, double radius_m) {
    const geo::LocalFrame f(centre);
    const geo::LatLon sw = f.to_geo({-radius_m, -radius_m});
    const geo::LatLon ne = f.to_geo({radius_m, radius_m});
    return GeoBox{sw.lat_deg, sw.lon_deg, ne.lat_deg, ne.lon_deg};
}

uint64_t count_tiles(const GeoBox& box, int zmin, int zmax) {
    zmin = std::max(zmin, 0);
    zmax = std::min(zmax, MAX_ZOOM);
    uint64_t n = 0;
    for (int z = zmin; z <= zmax; ++z) n += tile_range(box, z).count();
    return n;
}

PrefetchPlan plan_prefetch(const GeoBox& box, int zmin, int zmax, uint64_t cap) {
    PrefetchPlan plan;
    if (zmin < 0 || zmax > MAX_ZOOM || zmin > zmax) {
        plan.error = "Invalid zoom range " + std::to_string(zmin) + ".." + std::to_string(zmax);
        return plan;
    }
    // Counted arithmetically first, so a huge request is refused without
    // allocating millions of entries.
    plan.count = count_tiles(box, zmin, zmax);
    if (plan.count == 0) {
        plan.error = "The area is empty";
        return plan;
    }
    if (plan.count > cap) {
        plan.error = std::to_string(plan.count) + " tiles is more than the limit of " + std::to_string(cap) +
                     ". Make the area smaller or lower the maximum zoom.";
        return plan;
    }
    plan.tiles.reserve(static_cast<size_t>(plan.count));
    for (int z = zmin; z <= zmax; ++z) {
        const TileRange r = tile_range(box, z);
        for (int x = r.x0; x <= r.x1; ++x)
            for (int y = r.y0; y <= r.y1; ++y) plan.tiles.push_back(TileId{z, x, y});
    }
    plan.ok = true;
    return plan;
}

// ---------------------------------------------------------------------------
// Tile sources
// ---------------------------------------------------------------------------

const std::vector<TileSource>& builtin_sources() {
    // One entry per source: {id, name, URL template, ext, attribution,
    // max zoom, average tile size (KB, for estimates), usage note}.
    static const std::vector<TileSource> sources = {
        {"esri-world-imagery", "Esri World Imagery (satellite)",
         "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}", "jpg",
         "Imagery \xC2\xA9 Esri, Maxar, Earthstar Geographics", 19, 25,
         "Esri's terms of use apply. Download only the area you will use."},
        {"osm", "OpenStreetMap", "https://tile.openstreetmap.org/{z}/{x}/{y}.png", "png",
         "\xC2\xA9 OpenStreetMap contributors", 19, 15,
         "OpenStreetMap's tile usage policy discourages bulk downloading: keep the area to one lake and "
         "the zoom range modest, or use your own tile server."},
    };
    return sources;
}

const TileSource* find_builtin_source(const std::string& id) {
    for (const TileSource& s : builtin_sources())
        if (s.id == id) return &s;
    return nullptr;
}

TileSource custom_source(const std::string& url_template) {
    TileSource s;
    char hash[16];
    std::snprintf(hash, sizeof hash, "%08x", fnv1a(url_template));
    s.id = std::string(CUSTOM_SOURCE_ID) + "-" + hash;
    s.name = "Custom";
    s.url_template = url_template;
    // Extension from the last path segment, if it looks like one. It only
    // names the cache file; images are decoded by content.
    s.ext = "img";
    const size_t q = url_template.find('?');
    const std::string path = url_template.substr(0, q);
    const size_t slash = path.rfind('/');
    const size_t dot = path.rfind('.');
    if (dot != std::string::npos && (slash == std::string::npos || dot > slash)) {
        std::string ext = path.substr(dot + 1);
        std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return char(std::tolower(c)); });
        if (ext == "jpeg") ext = "jpg";
        if (ext == "png" || ext == "jpg") s.ext = ext;
    }
    // Attribute to the host; the operator knows what they are serving.
    const size_t scheme = url_template.find("://");
    const size_t host_begin = scheme == std::string::npos ? 0 : scheme + 3;
    const size_t host_end = url_template.find('/', host_begin);
    s.attribution = "Tiles: " + url_template.substr(host_begin, host_end - host_begin);
    s.max_zoom = 19;
    s.avg_tile_kb = 20;
    s.usage_note = "Make sure the server's terms allow downloading.";
    return s;
}

std::string check_url_template(const std::string& t) {
    if (t.rfind("http://", 0) != 0 && t.rfind("https://", 0) != 0) return "URL must start with http:// or https://";
    for (const char* p : {"{z}", "{x}", "{y}"})
        if (t.find(p) == std::string::npos) return std::string("URL must contain ") + p;
    return {};
}

std::string tile_url(const std::string& url_template, TileId t) {
    std::string s = replace_all(url_template, "{z}", std::to_string(t.z));
    s = replace_all(s, "{x}", std::to_string(t.x));
    return replace_all(s, "{y}", std::to_string(t.y));
}

// ---------------------------------------------------------------------------
// Cache
// ---------------------------------------------------------------------------

std::string cache_path(const std::string& cache_dir, const std::string& source_id, TileId t,
                       const std::string& ext) {
    std::string p = cache_dir;
    if (!p.empty() && p.back() != '/') p += '/';
    p += source_id + '/' + std::to_string(t.z) + '/' + std::to_string(t.x) + '/' + std::to_string(t.y) + '.' + ext;
    return p;
}

bool looks_like_image(const uint8_t* d, size_t n) {
    static const uint8_t PNG[8] = {0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A};
    if (n >= 8 && std::equal(PNG, PNG + 8, d)) return true;
    return n >= 3 && d[0] == 0xFF && d[1] == 0xD8 && d[2] == 0xFF;  // JPEG SOI
}

}  // namespace basestation::tiles
