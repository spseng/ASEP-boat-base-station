#pragma once

// The simulated scalar field drawn under the Fleet view: one image of the
// field around its Gaussians, coloured blue (low) to red (high), rebuilt
// only when the site file is (re)loaded.

#include <basestation/core/geo.h>
#include <basestation/core/site_config.h>

#include <SDL3/SDL_render.h>

#include <optional>

namespace basestation::app {

class FieldLayer {
public:
    explicit FieldLayer(SDL_Renderer* renderer) : renderer_(renderer) {}
    ~FieldLayer();
    FieldLayer(const FieldLayer&) = delete;
    FieldLayer& operator=(const FieldLayer&) = delete;

    // Builds the image of `field`, defined around `origin` (metres east /
    // north of it). nullptr clears.
    void set(const site::ScalarField* field, geo::LatLon origin);
    bool ready() const { return tex_ != nullptr; }

    // Inside BeginPlot, in the view's local frame.
    void draw(const geo::LocalFrame& view, float opacity) const;

    double value_at(geo::LatLon p) const;
    double min_value() const { return min_; }
    double max_value() const { return max_; }
    geo::LatLon min_position() const { return min_pos_; }  // lowest point of the image grid

private:
    SDL_Renderer* renderer_;
    SDL_Texture* tex_ = nullptr;
    site::ScalarField field_;
    geo::LocalFrame frame_;  // the field's own frame
    double e0_ = 0, e1_ = 0, n0_ = 0, n1_ = 0;  // image extent in frame_
    double min_ = 0, max_ = 0;
    geo::LatLon min_pos_{};
};

}  // namespace basestation::app
