#ifndef TURBINEMAP_H
#define TURBINEMAP_H

#include "MapReader.h"
#include <string>
#include <vector>

/*
 * TurbineMap.h
 * ------------
 * Declares the TurbineMap class — loads and interpolates 2D turbine
 * performance maps from .map files.
 *
 * USED BY:
 *   Turbine — HP Turbine and LP Turbine (same class, different map files)
 *
 * MAP AXES:
 *   x-axis: DhT [-] — dimensionless energy function
 *            DhT = Cp*(Tt_in - Tt_exit) / Tt_in
 *            NOTE: DhT [-] is NOT dHt [J/kg] used in Turbine.cpp
 *   y-axis: Nc  [RPM corrected]
 *
 * INTERPOLATION — bilinear:
 * 1. Apply scale factors to query (DhT, Nc) → reference map space
 * 2. Find bounding speed lines by Nc_ref
 * 3. Linear interpolation along each speed line by DhT_ref → (PR_ref, eff_ref)
 * 4. Linear interpolation between speed lines by Nc_ref → (PR_ref, eff_ref)
 * 5. Apply scale factors to result → (PR_actual, eff_actual)
 *
 * MAP SCALING (Phase 5 — Kurzke-Riegler method):
 *   Scale factors stored in map file header as SCALE_PR, SCALE_EFF,
 *   SCALE_WC, SCALE_NC. All default to 1.0 when absent.
 *
 *   Query transformation (actual → reference map space):
 *     DhT_ref = DhT_actual / scale_Wc   (DhT uses same scale as Wc axis)
 *     Nc_ref  = Nc_actual  / scale_Nc
 *
 *   Result transformation (reference → actual engine space):
 *     PR_actual  = 1.0 + (PR_ref  - 1.0) * scale_PR
 *     eff_actual = eff_ref * scale_Eff
 *
 * EXTRAPOLATION:
 *   Nc  outside map range        → silently clamped to nearest speed line
 *   DhT outside speed line range → silently clamped to nearest point
 *
 * FUTURE WORK (Phase 5):
 *   Turbine tip clearance control — efficiency correction factor
 *   as a function of power setting or Nc.
 *
 * REFERENCES:
 *   Kurzke, J. and Riegler, C., ASME 2000-GT-0006. [Scaling method]
 *   Walsh & Fletcher, "Gas Turbine Performance", Ch. 7-8. [Reference maps]
 *
 * UNITS:
 *   DhT  [-]          dimensionless energy function
 *   Nc   [RPM corrected]
 *   PR   [-]          total pressure ratio
 *   eff  [-]          isentropic efficiency
 *
 * NPSS ALIGNMENT:
 *   Mirrors NPSS TurbineMap element — data file driven,
 *   bilinear interpolation on (DhT, Nc) axes.
 */

// =========================================================================
// TurbineMapResult — returned by TurbineMap::lookup()
// =========================================================================

struct TurbineMapResult {
    double PR  = 1.0;   // pressure ratio [-] — scaled to actual engine
    double eff = 0.0;   // isentropic efficiency [-] — scaled to actual engine
};

// =========================================================================
// TurbineMap
// =========================================================================

class TurbineMap {
public:

    /*
     * Constructor — loads and parses the map file.
     * Call isLoaded() after construction to verify success.
     *
     * @param filepath  Path to the .map file (TYPE must be TURBINE)
     */
    explicit TurbineMap(const std::string& filepath);

    /*
     * isLoaded — returns true if the map file was parsed successfully.
     */
    bool isLoaded() const noexcept { return loaded_; }

    /*
     * lookup — returns PR and eff at the given operating point.
     * Scale factors applied transparently — inputs and outputs are in
     * actual engine units, not reference map units.
     *
     * @param DhT  Dimensionless energy function [-]
     *             DhT = Cp*(Tt_in - Tt_exit) / Tt_in
     * @param Nc   Corrected speed [RPM corrected]
     * @return     TurbineMapResult with scaled PR and eff
     */
    TurbineMapResult lookup(double DhT, double Nc) const;

    /*
     * designPoint — returns the design point from the map file header.
     */
    const MapReader::MapDesignPoint& designPoint() const noexcept
    {
        return design_pt_;
    }

    /*
     * scaleFactors — returns the Kurzke-Riegler scale factors.
     * All 1.0 when no SCALE_* fields present in map file.
     */
    const MapReader::MapScaleFactors& scaleFactors() const noexcept
    {
        return scales_;
    }

private:

    // =====================================================================
    // PRIVATE MEMBERS
    // =====================================================================

    bool                                  loaded_     = false;
    MapReader::MapDesignPoint             design_pt_;           // design point from header
    MapReader::MapScaleFactors            scales_;              // Kurzke-Riegler scale factors
    std::vector<MapReader::MapSpeedLine>  speed_lines_;         // 2D map data

    // =====================================================================
    // PRIVATE METHODS
    // =====================================================================

    /*
     * interpolateSpeedLine — linear interpolation along one speed line.
     * Operates in reference map space (before scale factors applied).
     * Silently clamps to speed line bounds if DhT_ref out of range.
     *
     * @param sl       Speed line to interpolate on
     * @param DhT_ref  Dimensionless energy function in reference space [-]
     * @return         {PR_ref, eff_ref}
     */
    std::pair<double,double>
    interpolateSpeedLine(const MapReader::MapSpeedLine& sl,
                         double                         DhT_ref) const noexcept;
};

#endif // TURBINEMAP_H