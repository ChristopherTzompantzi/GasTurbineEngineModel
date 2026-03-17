#include "InletMap.h"
#include <iostream>

/*
 * InletMap.cpp
 * ------------
 * Implements InletMap — 1D eta_r vs MN lookup for flight inlet.
 *
 * INTERPOLATION SEQUENCE (lookup):
 * 1. Validate map is loaded
 * 2. Clamp MN to map range
 * 3. Find bounding points by MN
 * 4. Linear interpolation → eta_r
 * 5. Return eta_r
 *
 * EXTRAPOLATION:
 *   MN below map minimum → silently clamped to lowest point
 *   MN above map maximum → silently clamped to highest point
 */

// =========================================================================
// Constructor
// =========================================================================

InletMap::InletMap(const std::string& filepath)
{
    const MapReader::MapFileData data = MapReader::read(filepath);

    if (data.type.empty())
    {
        std::cerr << "[InletMap] ERROR: failed to load '"
                  << filepath << "'\n";
        return;
    }
    if (data.type != "INLET")
    {
        std::cerr << "[InletMap] ERROR: file '" << filepath
                  << "' has TYPE=" << data.type
                  << " — expected INLET\n";
        return;
    }
    if (data.map_1d.size() < 2)
    {
        std::cerr << "[InletMap] ERROR: '" << filepath
                  << "' has fewer than 2 points\n";
        return;
    }

    design_pt_ = data.design_pt;
    points_    = data.map_1d;
    loaded_    = true;
}

// =========================================================================
// interpolate
//
// Linear interpolation of eta_r at the given MN.
// Silently clamps to map bounds if MN is out of range.
// =========================================================================

double InletMap::interpolate(double MN) const noexcept
{
    if (MN <= points_.front().x) return points_.front().y;
    if (MN >= points_.back().x)  return points_.back().y;

    for (std::size_t i = 0; i + 1 < points_.size(); ++i)
    {
        if (MN >= points_[i].x && MN <= points_[i+1].x)
        {
            const double t = (MN - points_[i].x)
                           / (points_[i+1].x - points_[i].x);
            return points_[i].y + t * (points_[i+1].y - points_[i].y);
        }
    }

    return points_.back().y;
}

// =========================================================================
// lookup
//
// Returns eta_r at the given flight Mach number.
// Delegates to interpolate() after validating map is loaded.
// =========================================================================

double InletMap::lookup(double MN) const
{
    if (!loaded_)
    {
        std::cerr << "[InletMap] ERROR: lookup called on unloaded map\n";
        return 0.0;
    }
    return interpolate(MN);
}