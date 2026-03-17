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
 *   y-axis: Nc  [%] — percent corrected speed
 *
 * INTERPOLATION — bilinear:
 * 1. Find bounding speed lines by Nc%
 * 2. Linear interpolation along each speed line by DhT → (PR, eff)
 * 3. Linear interpolation between speed lines by Nc% → final (PR, eff)
 *
 * EXTRAPOLATION:
 *   Nc%  outside map range        → silently clamped to nearest speed line
 *   DhT  outside speed line range → silently clamped to nearest point
 *
 * NO VSV SCHEDULE:
 *   Turbines do not use variable stator vanes in this model.
 *
 * FUTURE WORK (Phase 5):
 *   Turbine tip clearance control — efficiency correction factor
 *   as a function of power setting or Nc%.
 *
 * UNITS:
 *   DhT  [-]   dimensionless energy function
 *   Nc   [%]   percent of design corrected speed
 *   PR   [-]   total pressure ratio
 *   eff  [-]   isentropic efficiency
 *
 * NPSS ALIGNMENT:
 *   Mirrors NPSS TurbineMap element — data file driven,
 *   bilinear interpolation on (DhT, Nc%) axes.
 */

// =========================================================================
// TurbineMapResult — returned by TurbineMap::lookup()
// =========================================================================

struct TurbineMapResult {
    double PR  = 1.0;   // pressure ratio [-]
    double eff = 0.0;   // isentropic efficiency [-]
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
     * @param filepath  Path to the .map file
     */
    explicit TurbineMap(const std::string& filepath);

    /*
     * isLoaded — returns true if the map file was parsed successfully.
     * Elements must check this before calling lookup().
     */
    bool isLoaded() const noexcept { return loaded_; }

    /*
     * lookup — returns PR and eff at the given operating point.
     *
     * @param DhT     Dimensionless energy function [-]
     *                DhT = Cp*(Tt_in - Tt_exit) / Tt_in
     * @param Nc_pct  Corrected speed [% of design]
     * @return        TurbineMapResult with PR and eff
     */
    TurbineMapResult lookup(double DhT, double Nc_pct) const;

    /*
     * designPoint — returns the design point from the map file header.
     * Used by elements to verify consistency with cycle design point.
     */
    const MapReader::MapDesignPoint& designPoint() const noexcept
    {
        return design_pt_;
    }

private:

    // =====================================================================
    // PRIVATE MEMBERS
    // =====================================================================

    bool                                 loaded_     = false;   // true if map file parsed successfully
    MapReader::MapDesignPoint            design_pt_;            // design point reference from file header
    std::vector<MapReader::MapSpeedLine> speed_lines_;          // speed lines from SPEED_LINE blocks

    // =====================================================================
    // PRIVATE METHODS
    // =====================================================================

    /*
     * interpolateSpeedLine — linear interpolation along one speed line.
     * Returns (PR, eff) at the given DhT. Silently clamps if out of range.
     *
     * @param sl   Speed line to interpolate on
     * @param DhT  Dimensionless energy function [-]
     * @return     {PR, eff} pair
     */
    std::pair<double,double>
    interpolateSpeedLine(const MapReader::MapSpeedLine& sl,
                         double                         DhT) const noexcept;
};

#endif // TURBINEMAP_H