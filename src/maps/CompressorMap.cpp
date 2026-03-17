#include "CompressorMap.h"
#include <iostream>

/*
 * CompressorMap.cpp
 * -----------------
 * Implements CompressorMap — loads and interpolates 2D compressor/fan maps.
 *
 * INTERPOLATION SEQUENCE (lookup):
 * 1. Validate map is loaded
 * 2. Find bounding speed lines by Nc%
 * 3. Interpolate along each bounding speed line by Wc → (PR_lo, eff_lo)
 *                                                     → (PR_hi, eff_hi)
 * 4. Interpolate between speed line results by Nc% → final (PR, eff)
 * 5. Interpolate VSV schedule by Nc% → vsv_angle
 * 6. Return CompressorMapResult
 *
 * EXTRAPOLATION:
 *   Nc% below lowest speed line  → silently clamped to lowest speed line
 *   Nc% above highest speed line → silently clamped to highest speed line
 *   Wc outside speed line range  → silently clamped to nearest endpoint
 */

// =========================================================================
// Constructor
// =========================================================================

CompressorMap::CompressorMap(const std::string& filepath)
{
    // Load and parse map file via shared MapReader
    const MapReader::MapFileData data = MapReader::read(filepath);

    // Check parse succeeded and type is correct
    if (data.type.empty())
    {
        std::cerr << "[CompressorMap] ERROR: failed to load '"
                  << filepath << "'\n";
        return;
    }
    if (data.type != "COMPRESSOR")
    {
        std::cerr << "[CompressorMap] ERROR: file '" << filepath
                  << "' has TYPE=" << data.type
                  << " — expected COMPRESSOR\n";
        return;
    }
    if (data.speed_lines.size() < 2)
    {
        std::cerr << "[CompressorMap] ERROR: '" << filepath
                  << "' has fewer than 2 speed lines\n";
        return;
    }

    // Store parsed data
    design_pt_   = data.design_pt;
    speed_lines_ = data.speed_lines;
    vsv_schedule_ = data.vsv_schedule;
    loaded_      = true;
}

// =========================================================================
// interpolateSpeedLine
//
// Linear interpolation along one speed line at the given Wc.
// Returns {PR, eff}. Clamps to speed line bounds if Wc out of range.
// =========================================================================

std::pair<double,double>
CompressorMap::interpolateSpeedLine(const MapReader::MapSpeedLine& sl,
                                     double                         Wc) const noexcept
{
    const auto& pts = sl.points;

    // Clamp to speed line bounds
    if (Wc <= pts.front().x)
        return {pts.front().y1, pts.front().y2};
    if (Wc >= pts.back().x)
        return {pts.back().y1, pts.back().y2};

    // Find bounding points
    for (std::size_t i = 0; i + 1 < pts.size(); ++i)
    {
        if (Wc >= pts[i].x && Wc <= pts[i+1].x)
        {
            // Linear interpolation fraction
            const double t = (Wc - pts[i].x) / (pts[i+1].x - pts[i].x);
            const double PR  = pts[i].y1 + t * (pts[i+1].y1 - pts[i].y1);
            const double eff = pts[i].y2 + t * (pts[i+1].y2 - pts[i].y2);
            return {PR, eff};
        }
    }

    // Should not reach here — return last point as fallback
    return {pts.back().y1, pts.back().y2};
}

// =========================================================================
// interpolateVsv
//
// Linear interpolation of VSV angle from schedule at given Nc%.
// Returns 0.0 if no VSV schedule present.
// =========================================================================

double CompressorMap::interpolateVsv(double Nc_pct) const noexcept
{
    if (vsv_schedule_.empty()) return 0.0;

    // Clamp to schedule bounds
    if (Nc_pct <= vsv_schedule_.front().Nc)
        return vsv_schedule_.front().angle;
    if (Nc_pct >= vsv_schedule_.back().Nc)
        return vsv_schedule_.back().angle;

    // Find bounding entries
    for (std::size_t i = 0; i + 1 < vsv_schedule_.size(); ++i)
    {
        if (Nc_pct >= vsv_schedule_[i].Nc && Nc_pct <= vsv_schedule_[i+1].Nc)
        {
            const double t = (Nc_pct - vsv_schedule_[i].Nc)
                           / (vsv_schedule_[i+1].Nc - vsv_schedule_[i].Nc);
            return vsv_schedule_[i].angle
                 + t * (vsv_schedule_[i+1].angle - vsv_schedule_[i].angle);
        }
    }

    return vsv_schedule_.back().angle;
}

// =========================================================================
// lookup
//
// Returns PR, eff, and VSV angle at the given (Wc, Nc%) operating point.
// Uses bilinear interpolation between speed lines.
// =========================================================================

CompressorMapResult CompressorMap::lookup(double Wc, double Nc_pct) const
{
    CompressorMapResult result;

    if (!loaded_)
    {
        std::cerr << "[CompressorMap] ERROR: lookup called on unloaded map\n";
        return result;
    }

    const auto& sls = speed_lines_;

    // Step 1 — Clamp Nc% to map range
    if (Nc_pct <= sls.front().Nc)
    {
        // Below lowest speed line — use lowest
        const auto [PR, eff] = interpolateSpeedLine(sls.front(), Wc);
        result.PR        = PR;
        result.eff       = eff;
        result.vsv_angle = interpolateVsv(Nc_pct);
        return result;
    }
    if (Nc_pct >= sls.back().Nc)
    {
        // Above highest speed line — use highest
        const auto [PR, eff] = interpolateSpeedLine(sls.back(), Wc);
        result.PR        = PR;
        result.eff       = eff;
        result.vsv_angle = interpolateVsv(Nc_pct);
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

    // Step 3 — Interpolate along each bounding speed line by Wc
    const auto [PR_lo, eff_lo] = interpolateSpeedLine(sls[lo], Wc);
    const auto [PR_hi, eff_hi] = interpolateSpeedLine(sls[hi], Wc);

    // Step 4 — Interpolate between speed lines by Nc%
    const double t   = (Nc_pct - sls[lo].Nc) / (sls[hi].Nc - sls[lo].Nc);
    result.PR        = PR_lo  + t * (PR_hi  - PR_lo);
    result.eff       = eff_lo + t * (eff_hi - eff_lo);

    // Step 5 — Interpolate VSV angle
    result.vsv_angle = interpolateVsv(Nc_pct);

    return result;
}