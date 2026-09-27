#include "map_layer.h"

#include <SDL3/SDL_render.h>
#include <implot.h>
#include <stb_image.h>

#include <climits>
#include <cmath>
#include <cstdint>

namespace basestation::app {

namespace {

// Tiles are never larger than this; anything bigger is not a map tile.
constexpr int MAX_TILE_PX = 2048;
// How many zoom levels up to look for a cached tile to stand in for a
// missing one, and how many of those to ask the loader for.
constexpr int FALLBACK_LEVELS = 8;
constexpr int FALLBACK_REQUEST_LEVELS = 4;
// Safety net for very large windows at a low zoom.
constexpr uint64_t MAX_TILES_IN_VIEW = 400;
// Keep the current zoom until the ideal one is this far away (0.5 = none).
constexpr double ZOOM_HYSTERESIS = 0.75;

// Runs on the TileService loader thread; stb_image is thread-safe for this.
bool decode_tile(const std::vector<uint8_t>& bytes, tiles::TileImage& out) {
    if (bytes.size() > static_cast<size_t>(INT_MAX)) return false;
    int w = 0, h = 0, n = 0;
    stbi_uc* px = stbi_load_from_memory(bytes.data(), static_cast<int>(bytes.size()), &w, &h, &n, 4);
    if (!px) return false;
    const bool ok = w > 0 && h > 0 && w <= MAX_TILE_PX && h <= MAX_TILE_PX;
    if (ok) {
        out.width = w;
        out.height = h;
        out.rgba.assign(px, px + static_cast<size_t>(w) * static_cast<size_t>(h) * 4);
    }
    stbi_image_free(px);
    return ok;
}

ImTextureRef tex_ref(SDL_Texture* t) {
    // The SDL_Renderer backend's ImTextureID is the SDL_Texture pointer.
    return ImTextureRef(static_cast<ImTextureID>(reinterpret_cast<intptr_t>(t)));
}

}  // namespace

MapLayer::MapLayer(SDL_Renderer* renderer, const std::string& cache_dir, const std::string& user_agent,
                   bool offline)
    : renderer_(renderer) {
    tiles::TileService::Config cfg;
    cfg.cache_dir = cache_dir;
    cfg.user_agent = user_agent;
    service_ = std::make_unique<tiles::TileService>(cfg, decode_tile);
    service_->set_offline(offline);
}

MapLayer::~MapLayer() {
    service_.reset();  // stop the threads first
    clear_textures();
}

void MapLayer::clear_textures() {
    for (auto& [key, t] : textures_) SDL_DestroyTexture(t.tex);
    textures_.clear();
}

void MapLayer::update() {
    ++frame_;
    for (tiles::TileImage& img : service_->take_ready(UPLOADS_PER_FRAME)) {
        SDL_Texture* tex = SDL_CreateTexture(renderer_, SDL_PIXELFORMAT_RGBA32, SDL_TEXTUREACCESS_STATIC,
                                             img.width, img.height);
        if (!tex) continue;
        SDL_UpdateTexture(tex, nullptr, img.rgba.data(), img.width * 4);
        SDL_SetTextureBlendMode(tex, SDL_BLENDMODE_BLEND);
        SDL_SetTextureScaleMode(tex, SDL_SCALEMODE_LINEAR);
        Texture& slot = textures_[img.key];
        if (slot.tex) SDL_DestroyTexture(slot.tex);
        slot = Texture{tex, frame_};
    }
    // Least recently used first. Nothing drawn this frame yet, and the last
    // frame has been presented, so any texture may go.
    while (textures_.size() > MAX_TEXTURES) {
        auto oldest = textures_.begin();
        for (auto it = textures_.begin(); it != textures_.end(); ++it)
            if (it->second.last_used < oldest->second.last_used) oldest = it;
        SDL_DestroyTexture(oldest->second.tex);
        textures_.erase(oldest);
    }
}

bool MapLayer::draw_tile(const tiles::TileSource& src, const geo::LocalFrame& frame, tiles::TileId t,
                         const ImVec4& tint) {
    for (int up = 0; up <= FALLBACK_LEVELS && up <= t.z; ++up) {
        double u0 = 0, v0 = 0, u1 = 1, v1 = 1;
        const tiles::TileId a = tiles::ancestor(t, up, &u0, &v0, &u1, &v1);
        auto it = textures_.find(tiles::TileKey{src.id, a});
        if (it == textures_.end()) {
            // Lower zooms only from the cache: a fallback never costs a download.
            if (up > 0 && up <= FALLBACK_REQUEST_LEVELS) service_->request(src, a, frame_, true);
            continue;
        }
        it->second.last_used = frame_;
        // The tile's own corners in the view's local frame; for a fallback,
        // the matching part of the ancestor's image. Equirectangular per
        // tile: the error is far below a pixel at lake scale.
        const tiles::GeoBox b = tiles::tile_bounds(t);
        const geo::NorthEast sw = frame.to_local({b.south, b.west});
        const geo::NorthEast ne = frame.to_local({b.north, b.east});
        ImPlotSpec spec;
        spec.Flags = ImPlotItemFlags_NoFit;
        ImPlot::PlotImage("##tile", tex_ref(it->second.tex), ImPlotPoint(sw.east_m, sw.north_m),
                          ImPlotPoint(ne.east_m, ne.north_m), ImVec2(static_cast<float>(u0), static_cast<float>(v0)),
                          ImVec2(static_cast<float>(u1), static_cast<float>(v1)), tint, spec);
        return up == 0;
    }
    return false;
}

MapLayer::DrawResult MapLayer::draw(const tiles::TileSource& src, const geo::LocalFrame& frame, float opacity) {
    DrawResult res;
    const ImPlotRect lim = ImPlot::GetPlotLimits();
    const ImVec2 size = ImPlot::GetPlotSize();
    if (size.x < 1 || size.y < 1) return res;

    const geo::LatLon centre = frame.to_geo({lim.Y.Min + lim.Y.Size() / 2, lim.X.Min + lim.X.Size() / 2});
    const double m_per_px = lim.X.Size() / size.x;
    int z = tiles::zoom_for_resolution(m_per_px, centre.lat_deg, src.max_zoom);
    // Hysteresis: auto-fit changes the scale a little every frame, and near
    // a zoom boundary that would flip between two levels (and fetch both).
    const double exact = std::log2(tiles::metres_per_pixel(centre.lat_deg, 0) / m_per_px);
    if (last_zoom_ >= 0 && last_zoom_ <= src.max_zoom && std::fabs(exact - last_zoom_) < ZOOM_HYSTERESIS)
        z = last_zoom_;
    last_zoom_ = z;
    const geo::LatLon sw = frame.to_geo({lim.Y.Min, lim.X.Min});
    const geo::LatLon ne = frame.to_geo({lim.Y.Max, lim.X.Max});
    const tiles::GeoBox box{sw.lat_deg, sw.lon_deg, ne.lat_deg, ne.lon_deg};
    tiles::TileRange r = tiles::tile_range(box, z);
    while (r.count() > MAX_TILES_IN_VIEW && z > 0) r = tiles::tile_range(box, --z);

    const ImVec4 tint(1, 1, 1, opacity);
    res.zoom = z;
    for (int x = r.x0; x <= r.x1; ++x) {
        for (int y = r.y0; y <= r.y1; ++y) {
            const tiles::TileId t{z, x, y};
            ++res.tiles;
            if (textures_.count(tiles::TileKey{src.id, t}) == 0) service_->request(src, t, frame_);
            if (!draw_tile(src, frame, t, tint)) ++res.missing;
        }
    }
    return res;
}

}  // namespace basestation::app
