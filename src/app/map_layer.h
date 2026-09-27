#pragma once

// Map tiles under the Fleet view: owns the TileService (background loading
// and downloading) and the SDL textures made from decoded tiles. Textures are
// created on the UI thread only (SDL_Renderer is not thread-safe) and kept
// in an LRU cache.

#include <basestation/core/geo.h>
#include <basestation/core/map_tiles.h>
#include <basestation/core/tile_service.h>

#include <imgui.h>

#include <map>
#include <memory>
#include <string>

struct SDL_Renderer;
struct SDL_Texture;

namespace basestation::app {

class MapLayer {
public:
    static constexpr size_t MAX_TEXTURES = 256;
    static constexpr int UPLOADS_PER_FRAME = 16;  // bounds the per-frame cost of new tiles

    MapLayer(SDL_Renderer* renderer, const std::string& cache_dir, const std::string& user_agent, bool offline);
    ~MapLayer();
    MapLayer(const MapLayer&) = delete;
    MapLayer& operator=(const MapLayer&) = delete;

    // Call once per frame before drawing: turns decoded tiles into textures
    // and evicts the least recently used ones over MAX_TEXTURES.
    void update();

    struct DrawResult {
        int zoom = -1;     // zoom level used, -1 if nothing was drawn
        int tiles = 0;     // tiles in view
        int missing = 0;   // shown from a lower zoom or not at all
    };
    // Draws the tiles covering the current ImPlot view (call between
    // BeginPlot and EndPlot, before other items so the map is underneath).
    DrawResult draw(const tiles::TileSource& src, const geo::LocalFrame& frame, float opacity);

    tiles::TileService& service() { return *service_; }
    const tiles::TileService& service() const { return *service_; }
    size_t texture_count() const { return textures_.size(); }
    void clear_textures();

private:
    struct Texture {
        SDL_Texture* tex = nullptr;
        uint64_t last_used = 0;
    };
    // Draws `t`, or the part of its nearest cached ancestor that covers it.
    bool draw_tile(const tiles::TileSource& src, const geo::LocalFrame& frame, tiles::TileId t, const ImVec4& tint);

    SDL_Renderer* renderer_;
    std::unique_ptr<tiles::TileService> service_;
    std::map<tiles::TileKey, Texture> textures_;
    uint64_t frame_ = 0;
    int last_zoom_ = -1;
};

}  // namespace basestation::app
