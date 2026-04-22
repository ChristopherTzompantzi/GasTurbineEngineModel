#ifndef COMBUSTORMAP_H
#define COMBUSTORMAP_H

#include "MapReader.h"
#include <string>
#include <vector>

/*
 * CombustorMap.h
 * --------------
 * Declares the CombustorMap class — loads and interpolates 1D combustor
 * performance maps from .map files.
 *
 * USED BY:
 *   Combustor — replaces fixed eff_b and dPqP constructor parameters
 *               with table lookups when a map file is loaded.
 *
 * TWO EMBEDDED 1D TABLES:
 *   eff_b vs FAR  — combustion efficiency as function of fuel-air ratio
 *     Physical basis: at very low FAR (lean) combustion is incomplete —
 *     eff_b drops. At very high FAR (rich) eff_b also drops. Peak eff_b
 *     occurs near stoichiometric FAR (~0.067 for Jet-A in air).
 *     Typical range: 0.970 – 0.9995 depending on power setting.
 *
 *   dPqP vs Wc_in — pressure loss as function of corrected inlet flow
 *     Physical basis: pressure loss scales with dynamic pressure at
 *     combustor inlet. Higher corrected flow → higher inlet velocity →
 *     higher pressure loss. Typical range: 0.03 – 0.06.
 *
 * FILE FORMAT:
 *   TYPE        COMBUSTOR
 *   NAME        component identifier
 *   SOURCE      data provenance
 *   EFF_B_TABLE
 *     FAR=X  eff_b=X
 *     ...
 *   END_EFF_B_TABLE
 *   DPQP_TABLE
 *     Wc=X  dPqP=X
 *     ...
 *   END_DPQP_TABLE
 *
 * INTERPOLATION — linear 1D:
 *   Clamps to table bounds if query is outside range.
 *
 * NPSS ALIGNMENT:
 *   Mirrors NPSS Burner S_eff and S_dPqP subelements.
 *   Both are optional 1D table lookups attached after construction.
 *
 * REFERENCES:
 *   Walsh & Fletcher, "Gas Turbine Performance", Ch.6 — combustor maps.
 *   Lefebvre & Ballal, "Gas Turbine Combustion", 3rd ed., Ch.4 — eff_b curves.
 *
 * UNITS:
 *   FAR   [-]   fuel-air ratio
 *   eff_b [-]   combustion efficiency
 *   Wc_in [kg/s corrected]  corrected inlet flow
 *   dPqP  [-]   fractional total pressure loss
 */

// =========================================================================
// CombustorMapResult — returned by CombustorMap::lookup()
// =========================================================================

struct CombustorMapResult {
    double eff_b = 0.99;   // combustion efficiency [-]
    double dPqP  = 0.04;   // fractional pressure loss [-]
};

// =========================================================================
// CombustorMap
// =========================================================================

class CombustorMap {
public:

    /*
     * Constructor — loads and parses the map file.
     * Call isLoaded() after construction to verify success.
     *
     * @param filepath  Path to the .map file (TYPE must be COMBUSTOR)
     */
    explicit CombustorMap(const std::string& filepath);

    /*
     * isLoaded — returns true if the map file was parsed successfully.
     */
    bool isLoaded() const noexcept { return loaded_; }

    /*
     * lookup — returns eff_b and dPqP at the given operating point.
     * Both tables are interpolated independently.
     * Clamps to table bounds if query is outside range.
     *
     * @param FAR    Fuel-air ratio [-]
     * @param Wc_in  Corrected inlet flow [kg/s]
     * @return       CombustorMapResult with eff_b and dPqP
     */
    CombustorMapResult lookup(double FAR, double Wc_in) const noexcept;

private:

    // =====================================================================
    // PRIVATE MEMBERS
    // =====================================================================

    bool                            loaded_ = false;
    std::vector<MapReader::Map1DPoint> eff_b_table_;  // FAR → eff_b
    std::vector<MapReader::Map1DPoint> dpqp_table_;   // Wc_in → dPqP

    // =====================================================================
    // PRIVATE METHODS
    // =====================================================================

    /*
     * interpolate1D — linear interpolation on a 1D table.
     * Clamps to table bounds if x is outside range.
     *
     * @param table  Sorted (x, y) pairs
     * @param x      Query value
     * @return       Interpolated y value
     */
    double interpolate1D(const std::vector<MapReader::Map1DPoint>& table,
                         double                                     x) const noexcept;
};

#endif // COMBUSTORMAP_H