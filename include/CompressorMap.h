#ifndef COMPRESSORMAP_H
#define COMPRESSORMAP_H

#include "MapReader.h"
#include <string>
#include <vector>

/*
 * CompressorMap.h
 * ---------------
 * Declares the CompressorMap class — loads and interpolates 2D
 * compressor/fan performance maps from .map files.
 *
 * USED BY:
 *   Compressor — HP compressor with VSV schedule
 *   Fan        — bypass fan, no VSV (IGV support deferred to Phase 5)
 *
 * INTERPOLATION — bilinear:
 * 1. Find bounding speed lines by Nc%
 * 2. Linear interpolation along each speed line by Wc → (PR, eff)
 * 3. Linear interpolation between speed lines by Nc% → final (PR, eff)
 *
 * EXTRAPOLATION:
 *   Nc% outside map range → silently clamped to nearest speed line
 *   Wc  outside speed line range → silently clamped to nearest point
 *
 * VSV SCHEDULE:
 *   If present in the map file, vsv_angle is interpolated vs Nc%.
 *   Returns 0.0 for maps with no VSV schedule (e.g. fan).
 *   IGV support for fan deferred to Phase 5.
 *
 * UNITS:
 *   Wc      [kg/s corrected]
 *   Nc      [% of design corrected speed]
 *   PR      [-]
 *   eff     [-]
 *   vsv_angle [deg]  0.0 = design, negative = closing
 *
 * NPSS ALIGNMENT:
 *   Mirrors NPSS CompressorMap element — data file driven,
 *   bilinear interpolation, VSV schedule as separate table.
 */

// =========================================================================
// MapResult — returned by CompressorMap::lookup()
// =========================================================================

struct CompressorMapResult {
    double PR        = 1.0;   // pressure ratio [-]
    double eff       = 0.0;   // isentropic efficiency [-]
    double vsv_angle = 0.0;   // VSV angle [deg] — 0.0 if no schedule
};

// =========================================================================
// CompressorMap
// =========================================================================

class CompressorMap {
public:

    /*
     * Constructor — loads and parses the map file.
     * Call isLoaded() after construction to verify success.
     *
     * @param filepath  Path to the .map file
     */
    explicit CompressorMap(const std::string& filepath);

    /*
     * isLoaded — returns true if the map file was parsed successfully.
     * Elements must check this before calling lookup().
     */
    bool isLoaded() const noexcept { return loaded_; }

    /*
     * lookup — returns PR, eff, and VSV angle at the given operating point.
     *
     * @param Wc      Corrected mass flow [kg/s]
     * @param Nc_pct  Corrected speed [% of design]
     * @return        CompressorMapResult with PR, eff, vsv_angle
     */
    CompressorMapResult lookup(double Wc, double Nc_pct) const;

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

    bool                                 loaded_      = false;  // true if map file parsed successfully
    MapReader::MapDesignPoint            design_pt_;            // design point reference from file header
    std::vector<MapReader::MapSpeedLine> speed_lines_;          // speed lines from SPEED_LINE blocks
    std::vector<MapReader::VsvPoint>     vsv_schedule_;         // VSV schedule from VSV_SCHEDULE block

    // =====================================================================
    // PRIVATE METHODS
    // =====================================================================

    /*
     * interpolateSpeedLine — linear interpolation along one speed line.
     * Returns (PR, eff) at the given Wc on the given speed line.
     * Clamps to speed line bounds if Wc is out of range.
     *
     * @param sl   Speed line to interpolate on
     * @param Wc   Corrected mass flow [kg/s]
     * @return     {PR, eff} pair
     */
    std::pair<double,double>
    interpolateSpeedLine(const MapReader::MapSpeedLine& sl,
                         double                         Wc) const noexcept;

    /*
     * interpolateVsv — linear interpolation of VSV angle vs Nc%.
     * Returns 0.0 if no VSV schedule is present.
     *
     * @param Nc_pct  Corrected speed [%]
     * @return        VSV angle [deg]
     */
    double interpolateVsv(double Nc_pct) const noexcept;
};

#endif // COMPRESSORMAP_H