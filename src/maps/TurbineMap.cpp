#include "TurbineMap.h"
#include <iostream>

/*
 * TurbineMap.cpp
 * --------------
 * Implements TurbineMap — loads and interpolates 2D turbine maps.
 *
 * INTERPOLATION SEQUENCE (lookup):
 * 1. Validate map is loaded
 * 2. Find bounding speed lines by Nc%
 * 3. Interpolate along each bounding speed line by DhT → (PR_lo, eff_lo)
 *                                                      → (PR_hi, eff_hi)
 * 4. Interpolate between speed line results by Nc% → final (PR, eff)
 * 5. Return TurbineMapResult
 *
 * EXTRAPOLATION:
 *   Nc% below lowest speed line  → silently clamped to lowest speed line
 *   Nc% above highest speed line → silently clamped to highest speed line
 *   DhT outside speed line range → silently clamped to nearest endpoint
 */

// =========================================================================
// Constructor
// =========================================================================

TurbineMap::TurbineMap(const std::string& filepath)
{
    const MapReader::MapFileData data = MapReader::read(filepath);

    if (data.type.empty())
    {
        std::cerr << "[TurbineMap] ERROR: failed to load '"
                  << filepath << "'\n";
        return;
    }
    if (data.type != "TURBINE")
    {
        std::cerr << "[TurbineMap] ERROR: file '" << filepath
                  << "' has TYPE=" << data.type
                  << " — expected TURBINE\n";
        return;
    }
    if (data.speed_lines.size() < 2)
    {
        std::cerr << "[TurbineMap] ERROR: '" << filepath
                  << "' has fewer than 2 speed lines\n";
        return;
    }

    design_pt_   = data.design_pt;
    speed_lines_ = data.speed_lines;
    loaded_      = true;
}

// =========================================================================
// interpolateSpeedLine
//
// Linear interpolation along one speed line at the given DhT.
// Silently clamps to speed line bounds if DhT is out of range.
// =========================================================================

std::pair<double,double>
TurbineMap::interpolateSpeedLine(const MapReader::MapSpeedLine& sl,
                                  double                         DhT) const noexcept
{
    const auto& pts = sl.points;

    if (DhT <= pts.front().x)
        return {pts.front().y1, pts.front().y2};
    if (DhT >= pts.back().x)
        return {pts.back().y1, pts.back().y2};

    for (std::size_t i = 0; i + 1 < pts.size(); ++i)
    {
        if (DhT >= pts[i].x && DhT <= pts[i+1].x)
        {
            const double t   = (DhT - pts[i].x) / (pts[i+1].x - pts[i].x);
            const double PR  = pts[i].y1 + t * (pts[i+1].y1 - pts[i].y1);
            const double eff = pts[i].y2 + t * (pts[i+1].y2 - pts[i].y2);
            return {PR, eff};
        }
    }

    return {pts.back().y1, pts.back().y2};
}

// =========================================================================
// lookup
//
// Returns PR and eff at the given (DhT, Nc%) operating point.
// Uses bilinear interpolation between speed lines.
// =========================================================================

TurbineMapResult TurbineMap::lookup(double DhT, double Nc_pct) const
{
    TurbineMapResult result;

    if (!loaded_)
    {
        std::cerr << "[TurbineMap] ERROR: lookup called on unloaded map\n";
        return result;
    }

    const auto& sls = speed_lines_;

    // Step 1 — Clamp Nc% to map range
    if (Nc_pct <= sls.front().Nc)
    {
        const auto [PR, eff] = interpolateSpeedLine(sls.front(), DhT);
        result.PR  = PR;
        result.eff = eff;
        return result;
    }
    if (Nc_pct >= sls.back().Nc)
    {
        const auto [PR, eff] = interpolateSpeedLine(sls.back(), DhT);
        result.PR  = PR;
        result.eff = eff;
        return result;
    }

    // Step 2 — Find bounding speed lines
    std::size_t lo = 0;
    for (std::size_t i = 0; i + 1 < sls.size(); ++i)
    {
        if (Nc_pct >= sls[i].Nc && Nc_pct <= sls[i+1].Nc)
        {
            lo = i;
            break;
        }
    }
    const std::size_t hi = lo + 1;

    // Step 3 — Interpolate along each bounding speed line by DhT
    const auto [PR_lo, eff_lo] = interpolateSpeedLine(sls[lo], DhT);
    const auto [PR_hi, eff_hi] = interpolateSpeedLine(sls[hi], DhT);

    // Step 4 — Interpolate between speed lines by Nc%
    const double t = (Nc_pct - sls[lo].Nc) / (sls[hi].Nc - sls[lo].Nc);
    result.PR  = PR_lo  + t * (PR_hi  - PR_lo);
    result.eff = eff_lo + t * (eff_hi - eff_lo);

    return result;
}