#include "field_layer.h"

#include <implot.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace basestation::app {

namespace {

// Image size along the longer side: about 1 px per metre for a 250 m field.
constexpr int IMAGE_PX = 256;
// The image covers each Gaussian out to this many sigmas...
constexpr double EXTENT_SIGMAS = 3.0;
// ...and never more than this from the origin (a stray huge sigma).
constexpr double MAX_HALF_EXTENT_M = 2000.0;

// Blue - grey - red, like matplotlib's coolwarm (the simulator's plots).
void colour(double t, uint8_t* px) {
    static const double lo[3] = {59, 76, 192}, mid[3] = {221, 221, 221}, hi[3] = {180, 4, 38};
    t = std::clamp(t, 0.0, 1.0);
    const double* a = t < 0.5 ? lo : mid;
    const double* b = t < 0.5 ? mid : hi;
    const double u = t < 0.5 ? t * 2 : (t - 0.5) * 2;
    for (int i = 0; i < 3; ++i) px[i] = static_cast<uint8_t>(std::lround(a[i] + (b[i] - a[i]) * u));
    px[3] = 255;
}

ImTextureRef tex_ref(SDL_Texture* t) {
    // The SDL_Renderer backend's ImTextureID is the SDL_Texture pointer.
    return ImTextureRef(static_cast<ImTextureID>(reinterpret_cast<intptr_t>(t)));
}

}  // namespace

FieldLayer::~FieldLayer() {
    if (tex_) SDL_DestroyTexture(tex_);
}

void FieldLayer::set(const site::ScalarField* field, geo::LatLon origin) {
    if (tex_) SDL_DestroyTexture(tex_);
    tex_ = nullptr;
    if (!field || field->gaussians.empty()) return;
    field_ = *field;
    frame_ = geo::LocalFrame(origin);

    e0_ = n0_ = MAX_HALF_EXTENT_M;
    e1_ = n1_ = -MAX_HALF_EXTENT_M;
    for (const site::Gaussian& g : field_.gaussians) {
        const double r = EXTENT_SIGMAS * g.sigma_m;
        e0_ = std::min(e0_, g.east_m - r);
        e1_ = std::max(e1_, g.east_m + r);
        n0_ = std::min(n0_, g.north_m - r);
        n1_ = std::max(n1_, g.north_m + r);
    }
    e0_ = std::max(e0_, -MAX_HALF_EXTENT_M);
    e1_ = std::min(e1_, MAX_HALF_EXTENT_M);
    n0_ = std::max(n0_, -MAX_HALF_EXTENT_M);
    n1_ = std::min(n1_, MAX_HALF_EXTENT_M);
    if (!(e1_ > e0_ && n1_ > n0_)) return;

    // Square pixels.
    const double m_per_px = std::max(e1_ - e0_, n1_ - n0_) / IMAGE_PX;
    const int w = std::max(2, static_cast<int>(std::ceil((e1_ - e0_) / m_per_px)));
    const int h = std::max(2, static_cast<int>(std::ceil((n1_ - n0_) / m_per_px)));
    e1_ = e0_ + w * m_per_px;
    n1_ = n0_ + h * m_per_px;

    // Values at pixel centres; row 0 is the north edge (PlotImage's uv0).
    std::vector<double> v(static_cast<size_t>(w) * static_cast<size_t>(h));
    min_ = INFINITY;
    max_ = -INFINITY;
    double min_e = 0, min_n = 0;
    for (int r = 0; r < h; ++r) {
        const double n = n1_ - (r + 0.5) * m_per_px;
        for (int c = 0; c < w; ++c) {
            const double e = e0_ + (c + 0.5) * m_per_px;
            const double x = field_.value(e, n);
            v[static_cast<size_t>(r) * w + c] = x;
            max_ = std::max(max_, x);
            if (x < min_) {
                min_ = x;
                min_e = e;
                min_n = n;
            }
        }
    }
    min_pos_ = frame_.to_geo({min_n, min_e});

    std::vector<uint8_t> rgba(v.size() * 4);
    const double span = max_ > min_ ? max_ - min_ : 1.0;
    for (size_t i = 0; i < v.size(); ++i) colour((v[i] - min_) / span, &rgba[i * 4]);

    tex_ = SDL_CreateTexture(renderer_, SDL_PIXELFORMAT_RGBA32, SDL_TEXTUREACCESS_STATIC, w, h);
    if (!tex_) return;
    SDL_UpdateTexture(tex_, nullptr, rgba.data(), w * 4);
    SDL_SetTextureBlendMode(tex_, SDL_BLENDMODE_BLEND);
}

void FieldLayer::draw(const geo::LocalFrame& view, float opacity) const {
    if (!tex_) return;
    // Both frames are north-up; only the offset between origins matters.
    const geo::NorthEast sw = view.to_local(frame_.to_geo({n0_, e0_}));
    const geo::NorthEast ne = view.to_local(frame_.to_geo({n1_, e1_}));
    ImPlotSpec spec;
    spec.Flags = ImPlotItemFlags_NoFit;
    ImPlot::PlotImage("##field", tex_ref(tex_), ImPlotPoint(sw.east_m, sw.north_m), ImPlotPoint(ne.east_m, ne.north_m),
                      ImVec2(0, 0), ImVec2(1, 1), ImVec4(1, 1, 1, opacity), spec);
}

double FieldLayer::value_at(geo::LatLon p) const {
    const geo::NorthEast ne = frame_.to_local(p);
    return field_.value(ne.east_m, ne.north_m);
}

}  // namespace basestation::app
