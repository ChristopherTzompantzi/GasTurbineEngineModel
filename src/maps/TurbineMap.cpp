#include "TurbineMap.h"
#include <iostream>

/*
 * TurbineMap.cpp
 * --------------
 * Implements TurbineMap — loads and interpolates 2D turbine maps
 * with Kurzke-Riegler map scaling.
 *
 * INTERPOLATION SEQUENCE (lookup):
 * 1. Validate map is loaded
 * 2. Transform query to reference map space:
 *      DhT_ref = DhT_actual / scale_Wc   (DhT uses same axis scale as Wc)
 *      Nc_ref  = Nc_actual  / scale_Nc
 * 3. Clamp Nc_ref to map range
 * 4. Find bounding speed lines by Nc_ref
 * 5. Interpolate along each bounding speed line by DhT_ref → (PR_ref, eff_ref)
 * 6. Interpolate between speed lines by Nc_ref → (PR_ref, eff_ref)
 * 7. Transform result to actual engine space:
 *      PR_actual  = 1.0 + (PR_ref  - 1.0) * scale_PR
 *      eff_actual = eff_ref * scale_Eff
 * 8. Return TurbineMapResult
 *
 * NOTE ON DhT AXIS SCALING:
 *   DhT is dimensionless but is still scaled by scale_Wc because it plays
 *   the same role as Wc on the compressor map — it is the x-axis parameter
 *   that shifts with engine size. At scale_Wc=1.0 no change occurs.
 *
 * REFERENCE:
 *   Kurzke, J. and Riegler, C., ASME 2000-GT-0006.
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
    scales_      = data.scales;     // Phase 5: Kurzke-Riegler scale factors
    speed_lines_ = data.speed_lines;
    loaded_      = true;
}

// =========================================================================
// interpolateSpeedLine
//
// Linear interpolation along one speed line in reference map space.
// DhT_ref is already divided by scale_Wc before this is called.
// =========================================================================

std::pair<double,double>
TurbineMap::interpolateSpeedLine(const MapReader::MapSpeedLine& sl,
                                  double                         DhT_ref) const noexcept
{
    const auto& pts = sl.points;

    if (DhT_ref <= pts.front().x)
        return {pts.front().y1, pts.front().y2};
    if (DhT_ref >= pts.back().x)
        return {pts.back().y1, pts.back().y2};

    for (std::size_t i = 0; i + 1 < pts.size(); ++i)
    {
        if (DhT_ref >= pts[i].x && DhT_ref <= pts[i+1].x)
        {
            const double t   = (DhT_ref - pts[i].x) / (pts[i+1].x - pts[i].x);
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
// Returns PR and eff at the given (DhT, Nc) operating point.
// Applies Kurzke-Riegler scale factors transparently.
// =========================================================================

TurbineMapResult TurbineMap::lookup(double DhT, double Nc) const
{
    TurbineMapResult result;

    if (!loaded_)
    {
        std::cerr << "[TurbineMap] ERROR: lookup called on unloaded map\n";
        return result;
    }

    const auto& sls = speed_lines_;

    // Step 2 — Transform query to reference map space
    const double DhT_ref = DhT / scales_.scale_Wc;
    const double Nc_ref  = Nc  / scales_.scale_Nc;

    // Step 3 — Clamp Nc_ref and interpolate
    double PR_ref  = 0.0;
    double eff_ref = 0.0;

    if (Nc_ref <= sls.front().Nc)
    {
        const auto [PR, eff] = interpolateSpeedLine(sls.front(), DhT_ref);
        PR_ref  = PR;
        eff_ref = eff;
    }
    else if (Nc_ref >= sls.back().Nc)
    {
        const auto [PR, eff] = interpolateSpeedLine(sls.back(), DhT_ref);
        PR_ref  = PR;
        eff_ref = eff;
    }
    else
    {
        // Step 4 — Find bounding speed lines
        std::size_t lo = 0;
        for (std::size_t i = 0; i + 1 < sls.size(); ++i)
        {
            if (Nc_ref >= sls[i].Nc && Nc_ref <= sls[i+1].Nc)
            {
                lo = i;
                break;
            }
        }
        const std::size_t hi = lo + 1;

        // Step 5 — Interpolate along each bounding speed line
        const auto [PR_lo, eff_lo] = interpolateSpeedLine(sls[lo], DhT_ref);
        const auto [PR_hi, eff_hi] = interpolateSpeedLine(sls[hi], DhT_ref);

        // Step 6 — Interpolate between speed lines
        const double t = (Nc_ref - sls[lo].Nc) / (sls[hi].Nc - sls[lo].Nc);
        PR_ref  = PR_lo  + t * (PR_hi  - PR_lo);
        eff_ref = eff_lo + t * (eff_hi - eff_lo);
    }

    // Step 7 — Transform result to actual engine space
    result.PR  = 1.0 + (PR_ref  - 1.0) * scales_.scale_PR;
    result.eff = eff_ref * scales_.scale_Eff;

    return result;
}