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
 *   Fan        — bypass fan with IGV schedule (Phase 5.2)
 *
 * INTERPOLATION — bilinear:
 * 1. Apply scale factors to query (Wc, Nc) → reference map space
 * 2. Find bounding speed lines by Nc
 * 3. Linear interpolation along each speed line by Wc → (PR_ref, eff_ref)
 * 4. Linear interpolation between speed lines by Nc → (PR_ref, eff_ref)
 * 5. Apply scale factors to result → (PR_actual, eff_actual)
 * 6. Interpolate VSV/IGV schedule by Nc → vsv_angle
 *
 * MAP SCALING (Phase 5 — Kurzke-Riegler method):
 *   Scale factors stored in map file header as SCALE_PR, SCALE_EFF,
 *   SCALE_WC, SCALE_NC. All default to 1.0 when absent.
 *
 *   Query transformation (actual → reference map space):
 *     Wc_query = Wc_actual / scale_Wc
 *     Nc_query = Nc_actual / scale_Nc
 *
 *   Result transformation (reference → actual engine space):
 *     PR_actual  = 1.0 + (PR_ref  - 1.0) * scale_PR
 *     eff_actual = eff_ref * scale_Eff
 *
 *   PR uses (PR-1) scaling to preserve PR=1 at zero compression.
 *
 * SURGE LINE (Phase 5):
 *   If SURGE_LINE block present in map file, surgePR(Wc) returns
 *   the surge boundary PR at the given corrected flow. Scale factors
 *   are applied to the surge line at lookup time.
 *
 *   Surge margin:
 *     SM = (PR_surge(Wc) - PR_op) / PR_surge(Wc) * 100  [%]
 *
 * EXTRAPOLATION:
 *   Nc outside map range → silently clamped to nearest speed line
 *   Wc outside speed line range → silently clamped to nearest point
 *
 * REFERENCES:
 *   Kurzke, J. and Riegler, C., ASME 2000-GT-0006. [Scaling method]
 *   Walsh & Fletcher, "Gas Turbine Performance", Ch. 7-8. [Reference maps]
 *   Mattingly, Appendix D. [Surge line data]
 *
 * UNITS:
 *   Wc        [kg/s corrected]
 *   Nc        [RPM corrected]
 *   PR        [-]
 *   eff       [-]
 *   vsv_angle [deg]  0.0 = design, negative = closing
 */

// =========================================================================
// CompressorMapResult — returned by lookup()
// =========================================================================

struct CompressorMapResult {
    double PR        = 1.0;   // pressure ratio [-] — scaled to actual engine
    double eff       = 0.0;   // isentropic efficiency [-] — scaled to actual engine
    double vsv_angle = 0.0;   // VSV/IGV angle [deg] — 0.0 if no schedule
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
     * @param filepath  Path to the .map file (TYPE must be COMPRESSOR)
     */
    explicit CompressorMap(const std::string& filepath);

    /*
     * isLoaded — returns true if the map file was parsed successfully.
     */
    bool isLoaded() const noexcept { return loaded_; }

    /*
     * lookup — returns PR, eff, and VSV/IGV angle at the given operating point.
     * Scale factors are applied transparently — inputs and outputs are in
     * actual engine units, not reference map units.
     *
     * @param Wc  Corrected mass flow [kg/s]
     * @param Nc  Corrected speed [RPM corrected]
     * @return    CompressorMapResult with scaled PR, eff, vsv_angle
     */
    CompressorMapResult lookup(double Wc, double Nc) const;

    /*
     * surgePR — returns the surge boundary pressure ratio at the given Wc.
     * Scale factors applied to surge line at lookup time.
     * Returns 0.0 if no surge line is present in the map file.
     *
     * @param Wc  Corrected mass flow [kg/s] — actual engine units
     * @return    PR_surge [-] — 0.0 if no surge line loaded
     */
    double surgePR(double Wc) const noexcept;

    /*
     * surgeMargin — returns surge margin [%] at the given operating point.
     * SM = (PR_surge(Wc) - PR_operating) / PR_surge(Wc) * 100
     * Returns 0.0 if no surge line present.
     *
     * @param Wc            Corrected mass flow [kg/s]
     * @param PR_operating  Current operating PR [-]
     * @return              Surge margin [%]
     */
    double surgeMargin(double Wc, double PR_operating) const noexcept;

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

    bool                                  loaded_       = false;
    MapReader::MapDesignPoint             design_pt_;            // design point from header
    MapReader::MapScaleFactors            scales_;               // Kurzke-Riegler scale factors
    std::vector<MapReader::MapSpeedLine>  speed_lines_;          // 2D map data
    std::vector<MapReader::VsvPoint>      vsv_schedule_;         // VSV/IGV schedule
    std::vector<MapReader::Map1DPoint>    surge_line_;           // surge boundary (Wc, PR_surge)

    // =====================================================================
    // PRIVATE METHODS
    // =====================================================================

    /*
     * interpolateSpeedLine — linear interpolation along one speed line.
     * Operates in reference map space (before scale factors applied).
     * Clamps to speed line bounds if Wc_ref out of range.
     *
     * @param sl      Speed line to interpolate on
     * @param Wc_ref  Corrected flow in reference map space [kg/s]
     * @return        {PR_ref, eff_ref}
     */
    std::pair<double,double>
    interpolateSpeedLine(const MapReader::MapSpeedLine& sl,
                         double                         Wc_ref) const noexcept;

    /*
     * interpolateVsv — linear interpolation of VSV/IGV angle vs Nc.
     * Operates in actual engine space (Nc in RPM corrected).
     * Returns 0.0 if no schedule present.
     *
     * @param Nc  Corrected speed [RPM corrected]
     * @return    VSV/IGV angle [deg]
     */
    double interpolateVsv(double Nc) const noexcept;

    /*
     * interpolateSurgeLine — linear interpolation of surge PR vs Wc.
     * Operates in reference map space. Scale applied by surgePR().
     * Returns 0.0 if surge line is empty.
     *
     * @param Wc_ref  Corrected flow in reference map space [kg/s]
     * @return        PR_surge_ref [-]
     */
    double interpolateSurgeLine(double Wc_ref) const noexcept;
};

#endif // COMPRESSORMAP_H