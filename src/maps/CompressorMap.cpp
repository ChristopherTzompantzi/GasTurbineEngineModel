#include "CompressorMap.h"
#include <iostream>

/*
 * CompressorMap.cpp
 * -----------------
 * Implements CompressorMap — loads and interpolates 2D compressor/fan maps
 * with Kurzke-Riegler map scaling and surge line support.
 *
 * INTERPOLATION SEQUENCE (lookup):
 * 1. Validate map is loaded
 * 2. Transform query to reference map space:
 *      Wc_ref = Wc_actual / scale_Wc
 *      Nc_ref = Nc_actual / scale_Nc
 * 3. Clamp Nc_ref to map range — extrapolation uses boundary speed line
 * 4. Find bounding speed lines by Nc_ref
 * 5. Interpolate along each bounding speed line by Wc_ref → (PR_ref, eff_ref)
 * 6. Interpolate between speed lines by Nc_ref → (PR_ref, eff_ref)
 * 7. Transform result to actual engine space:
 *      PR_actual  = 1.0 + (PR_ref  - 1.0) * scale_PR
 *      eff_actual = eff_ref * scale_Eff
 * 8. Interpolate VSV/IGV angle vs Nc_actual (no scaling on angles)
 * 9. Return CompressorMapResult
 *
 * SURGE LINE SEQUENCE (surgePR):
 * 1. Transform Wc_actual to reference space: Wc_ref = Wc / scale_Wc
 * 2. Interpolate surge line at Wc_ref → PR_surge_ref
 * 3. Transform to actual space: PR_surge = 1.0 + (PR_surge_ref - 1.0) * scale_PR
 *
 * NOTE ON SCALE FACTORS:
 *   All scale factors default to 1.0 when not present in map file.
 *   At scale=1.0 the behaviour is identical to Phase 3/4 — no change.
 *   The (PR-1) formulation preserves PR=1 at zero compression regardless
 *   of scale_PR value.
 *
 * REFERENCE:
 *   Kurzke, J. and Riegler, C., ASME 2000-GT-0006.
 */

// =========================================================================
// Constructor
// =========================================================================

CompressorMap::CompressorMap(const std::string& filepath)
{
    const MapReader::MapFileData data = MapReader::read(filepath);

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

    design_pt_    = data.design_pt;
    scales_       = data.scales;        // Phase 5: Kurzke-Riegler scale factors
    speed_lines_  = data.speed_lines;
    vsv_schedule_ = data.vsv_schedule;
    surge_line_   = data.surge_line;    // Phase 5: surge boundary
    loaded_       = true;
}

// =========================================================================
// interpolateSpeedLine
//
// Linear interpolation along one speed line in reference map space.
// Wc_ref is already divided by scale_Wc before this is called.
// =========================================================================

std::pair<double,double>
CompressorMap::interpolateSpeedLine(const MapReader::MapSpeedLine& sl,
                                     double                         Wc_ref) const noexcept
{
    const auto& pts = sl.points;

    if (Wc_ref <= pts.front().x)
        return {pts.front().y1, pts.front().y2};
    if (Wc_ref >= pts.back().x)
        return {pts.back().y1, pts.back().y2};

    for (std::size_t i = 0; i + 1 < pts.size(); ++i)
    {
        if (Wc_ref >= pts[i].x && Wc_ref <= pts[i+1].x)
        {
            const double t   = (Wc_ref - pts[i].x) / (pts[i+1].x - pts[i].x);
            const double PR  = pts[i].y1 + t * (pts[i+1].y1 - pts[i].y1);
            const double eff = pts[i].y2 + t * (pts[i+1].y2 - pts[i].y2);
            return {PR, eff};
        }
    }

    return {pts.back().y1, pts.back().y2};
}

// =========================================================================
// interpolateVsv
//
// Linear interpolation of VSV/IGV angle from schedule at given Nc.
// Operates in actual engine space — Nc in RPM corrected.
// VSV angles are not scaled (they are geometric, not aerodynamic).
// Returns 0.0 if no schedule present.
// =========================================================================

double CompressorMap::interpolateVsv(double Nc) const noexcept
{
    if (vsv_schedule_.empty()) return 0.0;

    if (Nc <= vsv_schedule_.front().Nc)
        return vsv_schedule_.front().angle;
    if (Nc >= vsv_schedule_.back().Nc)
        return vsv_schedule_.back().angle;

    for (std::size_t i = 0; i + 1 < vsv_schedule_.size(); ++i)
    {
        if (Nc >= vsv_schedule_[i].Nc && Nc <= vsv_schedule_[i+1].Nc)
        {
            const double t = (Nc - vsv_schedule_[i].Nc)
                           / (vsv_schedule_[i+1].Nc - vsv_schedule_[i].Nc);
            return vsv_schedule_[i].angle
                 + t * (vsv_schedule_[i+1].angle - vsv_schedule_[i].angle);
        }
    }

    return vsv_schedule_.back().angle;
}

// =========================================================================
// interpolateSurgeLine
//
// Linear interpolation of surge PR vs Wc in reference map space.
// Returns 0.0 if no surge line present.
// =========================================================================

double CompressorMap::interpolateSurgeLine(double Wc_ref) const noexcept
{
    if (surge_line_.empty()) return 0.0;

    if (Wc_ref <= surge_line_.front().x)
        return surge_line_.front().y;
    if (Wc_ref >= surge_line_.back().x)
        return surge_line_.back().y;

    for (std::size_t i = 0; i + 1 < surge_line_.size(); ++i)
    {
        if (Wc_ref >= surge_line_[i].x && Wc_ref <= surge_line_[i+1].x)
        {
            const double t = (Wc_ref      - surge_line_[i].x)
                           / (surge_line_[i+1].x - surge_line_[i].x);
            return surge_line_[i].y
                 + t * (surge_line_[i+1].y - surge_line_[i].y);
        }
    }

    return surge_line_.back().y;
}

// =========================================================================
// surgePR
//
// Returns surge boundary PR at the given actual Wc.
// Applies scale factors: transform Wc to reference space, interpolate,
// then transform PR result back to actual engine space.
// Returns 0.0 if no surge line loaded.
// =========================================================================

double CompressorMap::surgePR(double Wc) const noexcept
{
    if (surge_line_.empty()) return 0.0;

    // Transform query to reference space
    const double Wc_ref      = Wc / scales_.scale_Wc;

    // Interpolate surge line in reference space
    const double PR_surge_ref = interpolateSurgeLine(Wc_ref);

    // Transform result to actual engine space
    return 1.0 + (PR_surge_ref - 1.0) * scales_.scale_PR;
}

// =========================================================================
// surgeMargin
//
// SM = (PR_surge(Wc) - PR_operating) / PR_surge(Wc) * 100  [%]
// Returns 0.0 if no surge line loaded.
// =========================================================================

double CompressorMap::surgeMargin(double Wc, double PR_operating) const noexcept
{
    const double PR_surge = surgePR(Wc);
    if (PR_surge <= 0.0) return 0.0;
    return (PR_surge - PR_operating) / PR_surge * 100.0;
}

// =========================================================================
// lookup
//
// Returns PR, eff, and VSV/IGV angle at the given (Wc, Nc) operating point.
// Applies Kurzke-Riegler scale factors transparently.
// =========================================================================

CompressorMapResult CompressorMap::lookup(double Wc, double Nc) const
{
    CompressorMapResult result;

    if (!loaded_)
    {
        std::cerr << "[CompressorMap] ERROR: lookup called on unloaded map\n";
        return result;
    }

    const auto& sls = speed_lines_;

    // Step 2 — Transform query to reference map space
    const double Wc_ref = Wc / scales_.scale_Wc;
    const double Nc_ref = Nc / scales_.scale_Nc;

    // Step 3 — Clamp Nc_ref and interpolate
    double PR_ref  = 0.0;
    double eff_ref = 0.0;

    if (Nc_ref <= sls.front().Nc)
    {
        const auto [PR, eff] = interpolateSpeedLine(sls.front(), Wc_ref);
        PR_ref  = PR;
        eff_ref = eff;
    }
    else if (Nc_ref >= sls.back().Nc)
    {
        const auto [PR, eff] = interpolateSpeedLine(sls.back(), Wc_ref);
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
        const auto [PR_lo, eff_lo] = interpolateSpeedLine(sls[lo], Wc_ref);
        const auto [PR_hi, eff_hi] = interpolateSpeedLine(sls[hi], Wc_ref);

        // Step 6 — Interpolate between speed lines
        const double t = (Nc_ref - sls[lo].Nc) / (sls[hi].Nc - sls[lo].Nc);
        PR_ref  = PR_lo  + t * (PR_hi  - PR_lo);
        eff_ref = eff_lo + t * (eff_hi - eff_lo);
    }

    // Step 7 — Transform result to actual engine space
    // PR uses (PR-1) formulation — preserves PR=1 at zero compression
    result.PR  = 1.0 + (PR_ref  - 1.0) * scales_.scale_PR;
    result.eff = eff_ref * scales_.scale_Eff;

    // Step 8 — Interpolate VSV/IGV angle in actual engine space
    result.vsv_angle = interpolateVsv(Nc);

    return result;
}