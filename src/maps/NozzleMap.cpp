#include "NozzleMap.h"
#include <iostream>

/*
 * NozzleMap.cpp
 * -------------
 * Implements NozzleMap — 1D Cfg vs NPR lookup for core and fan nozzles.
 *
 * INTERPOLATION SEQUENCE (lookup):
 * 1. Validate map is loaded
 * 2. Clamp NPR to map range
 * 3. Find bounding points by NPR
 * 4. Linear interpolation → Cfg
 * 5. Return Cfg
 *
 * EXTRAPOLATION:
 *   NPR below map minimum → silently clamped to lowest point
 *   NPR above map maximum → silently clamped to highest point
 */

NozzleMap::NozzleMap(const std::string& filepath)
{
    const MapReader::MapFileData data = MapReader::read(filepath);

    if (data.type.empty())
    {
        std::cerr << "[NozzleMap] ERROR: failed to load '"
                  << filepath << "'\n";
        return;
    }
    if (data.type != "NOZZLE")
    {
        std::cerr << "[NozzleMap] ERROR: file '" << filepath
                  << "' has TYPE=" << data.type
                  << " — expected NOZZLE\n";
        return;
    }
    if (data.map_1d.size() < 2)
    {
        std::cerr << "[NozzleMap] ERROR: '" << filepath
                  << "' has fewer than 2 points\n";
        return;
    }

    design_pt_ = data.design_pt;
    points_    = data.map_1d;
    loaded_    = true;
}

double NozzleMap::interpolate(double NPR) const noexcept
{
    if (NPR <= points_.front().x) return points_.front().y;
    if (NPR >= points_.back().x)  return points_.back().y;

    for (std::size_t i = 0; i + 1 < points_.size(); ++i)
    {
        if (NPR >= points_[i].x && NPR <= points_[i+1].x)
        {
            const double t = (NPR - points_[i].x)
                           / (points_[i+1].x - points_[i].x);
            return points_[i].y + t * (points_[i+1].y - points_[i].y);
        }
    }

    return points_.back().y;
}

double NozzleMap::lookup(double NPR) const
{
    if (!loaded_)
    {
        std::cerr << "[NozzleMap] ERROR: lookup called on unloaded map\n";
        return 0.0;
    }
    return interpolate(NPR);
}