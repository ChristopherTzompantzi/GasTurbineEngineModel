#ifndef COMBUSTOR_H
#define COMBUSTOR_H

#include "Element.h"
#include "Thermo.h"

/*
 * Combustor.h
 * -----------
 * Declares the Combustor element — adds fuel energy to the flow,
 * raising total temperature and introducing FAR into the cycle.
 * Inherits from Element.
 *
 * WHAT THE COMBUSTOR DOES:
 * 1. Receives compressed air from Compressor exit
 * 2. Adds fuel — raises total temperature
 * 3. Applies fractional pressure loss (dPqP)
 * 4. Updates FAR to reflect added fuel mass
 * 5. Writes real gas Cp and gamma to flowOut via Thermo at exit conditions
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
 * NAMING CONVENTION — NPSS:
 *   Tt4    — turbine inlet total temperature [K]
 *   FAR    — fuel-air ratio [-]
 *   dPqP   — fractional total pressure loss [-]  (NPSS: dPqP)
 *   eff_b  — combustor thermal efficiency [-]
 *   LHV    — lower heating value of fuel [J/kg]
 *
 * LHV — LOWER HEATING VALUE:
 *   Energy released per kg of fuel burned.
 *   Jet-A default: 43,100,000 J/kg (43.1 MJ/kg)
 *   Called "lower" because water produced by combustion
 *   is assumed to leave as vapor — always true in gas turbines.
 *
 * FUTURE WORK:
 *   Species tracking for combustion products.
 *   NOx and emissions modeling.
 */

enum class CombustorMode { Tt4, FAR };

class Combustor : public Element {
public:
    // Constructor — mode selects operating mode.
    // Pass the relevant parameter; the unused one is ignored during compute().
    Combustor(CombustorMode mode,
              double Tt4,
              double FAR_in,
              double eff_b  = 0.99,
              double dPqP   = 0.04,
              double LHV    = 43100000.0) noexcept;

    // Implements Element::compute() — performs combustor thermodynamics
    void compute() noexcept override;

private:
    // Members always initialized via constructor — no in-class defaults needed.
    CombustorMode mode;   // Operating mode — Tt4 or FAR
    double Tt4;           // Turbine inlet total temperature [K] — used in Tt4 mode
    double FAR_in;        // Fuel-air ratio [-] — used in FAR mode
    double eff_b;         // Combustor thermal efficiency [-]
    double dPqP;          // Fractional total pressure loss [-]
    double LHV;           // Lower heating value of fuel [J/kg]
};

#endif // COMBUSTOR_H