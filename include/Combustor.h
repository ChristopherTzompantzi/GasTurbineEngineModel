#ifndef COMBUSTOR_H
#define COMBUSTOR_H

#include "Element.h"
#include "Thermo.h"
#include "CombustorMap.h"
#include <optional>

/*
 * Combustor.h
 * -----------
 * Declares the Combustor element — adds fuel energy to the flow,
 * raising total temperature and introducing FAR into the cycle.
 * Inherits from Element.
 *
 * WHAT THE COMBUSTOR DOES:
 * 1. Receives compressed air from Compressor exit
 * 2. Gets eff_b and dPqP — from CombustorMap if loaded, else constructor values
 * 3. Adds fuel — raises total temperature
 * 4. Applies fractional pressure loss (dPqP)
 * 5. Updates FAR to reflect added fuel mass
 * 6. Writes real gas Cp and gamma to flowOut via Thermo at exit conditions
 *
 * TWO OPERATING MODES (selectable via constructor):
 *   Tt4 mode — user specifies turbine inlet temperature (Tt4)
 *              model computes required FAR
 *              most common in engine design — Tt4 is a design constraint
 *
 *   FAR mode — user specifies fuel-air ratio (FAR)
 *              model computes resulting Tt_exit
 *              useful for off-design studies
 *
 * THERMODYNAMIC EQUATIONS (real gas via Thermo):
 *
 *   Pressure loss:
 *     Pt_exit = Pt_inlet × (1 - dPqP)
 *
 *   FAR mode — compute Tt_exit from FAR:
 *     H_inlet  = Thermo::getH(Tt_inlet, FAR_inlet)
 *     H_exit   = H_inlet + FAR × LHV × eff_b / (1 + FAR)
 *     Tt_exit  = Thermo::getT(H_exit, FAR_exit)
 *
 *   Tt4 mode — compute FAR from Tt4:
 *     Solved iteratively via enthalpy balance:
 *     H_exit(Tt4, FAR) = H_inlet + FAR × LHV × eff_b / (1 + FAR)
 *     FAR is iterated until enthalpy balance is satisfied
 *
 *   Exit properties:
 *     Cp    = Thermo::getCp   (Tt_exit, FAR_exit)   — written to flowOut
 *     gamma = Thermo::getGamma(Tt_exit, FAR_exit)   — written to flowOut
 *
 * COMBUSTOR MAP (Phase 5.4):
 *   When a CombustorMap is loaded via loadMap(), eff_b and dPqP are
 *   obtained from table lookups instead of constructor constants:
 *     eff_b = f(FAR)    — drops at lean and rich extremes
 *     dPqP  = f(Wc_in)  — increases with corrected inlet flow
 *   Mirrors NPSS Burner S_eff and S_dPqP subelements.
 *   Falls back to constructor values if map not loaded.
 *
 * NAMING CONVENTION — NPSS:
 *   Tt4    — turbine inlet total temperature [K]
 *   FAR    — fuel-air ratio [-]
 *   dPqP   — fractional total pressure loss [-]
 *   eff_b  — combustor thermal efficiency [-]
 *   LHV    — lower heating value of fuel [J/kg]
 *
 * LHV — LOWER HEATING VALUE:
 *   Jet-A default: 43,100,000 J/kg (43.1 MJ/kg)
 *
 * REFERENCES:
 *   Walsh & Fletcher, "Gas Turbine Performance", Ch.6.
 *   Lefebvre & Ballal, "Gas Turbine Combustion", 3rd ed., Ch.4.
 *
 * FUTURE WORK:
 *   Species tracking for combustion products.
 *   NOx and emissions modeling.
 */

enum class CombustorMode { Tt4, FAR };

class Combustor : public Element {
public:

    /*
     * Constructor — mode selects operating mode.
     * Pass the relevant parameter; the unused one is ignored during compute().
     * Call loadMap() after construction to enable map-based eff_b and dPqP.
     */
    Combustor(CombustorMode mode,
              double Tt4,
              double FAR_in,
              double eff_b  = 0.99,
              double dPqP   = 0.04,
              double LHV    = 43100000.0) noexcept;

    /*
     * loadMap — loads a CombustorMap from file.
     * Mirrors NPSS S_eff / S_dPqP subelement pattern.
     * Once loaded, compute() uses map lookup for eff_b and dPqP.
     * If load fails, falls back to constructor values with a warning.
     *
     * @param filepath  Path to the .map file (TYPE must be COMBUSTOR)
     */
    void loadMap(const std::string& filepath);

    /*
     * compute — performs combustor thermodynamics.
     * Implements Element::compute().
     */
    void compute() noexcept override;

    // Current combustion efficiency [-] — available after compute()
    // From map lookup if loaded, else constructor value
    double eff_b_out = 0.0;

    // Current pressure loss fraction [-] — available after compute()
    double dPqP_out  = 0.0;

private:

    CombustorMode                  mode_;    // Operating mode — Tt4 or FAR
    double                         Tt4_;     // Turbine inlet total temperature [K]
    double                         FAR_in_;  // Fuel-air ratio [-] — FAR mode
    double                         eff_b_;   // Fallback combustion efficiency [-]
    double                         dPqP_;    // Fallback pressure loss fraction [-]
    double                         LHV_;     // Lower heating value [J/kg]
    std::optional<CombustorMap>    map_;     // CombustorMap — present only if loadMap() called
};

#endif // COMBUSTOR_H