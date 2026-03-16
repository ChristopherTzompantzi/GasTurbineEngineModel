#ifndef AFTERBURNER_H
#define AFTERBURNER_H

#include "Element.h"
#include "Thermo.h"

/*
 * Afterburner.h
 * -------------
 * Models the afterburner (reheat) of a turbojet or turbofan engine.
 * Adds fuel to the turbine exhaust to raise total temperature,
 * producing additional thrust at the cost of high fuel consumption.
 *
 * WHAT THE AFTERBURNER DOES:
 * 1. Receives turbine exit flow on flowIn
 * 2. Computes inlet enthalpy via Thermo::getH
 * 3. Iterates FAR_ab until enthalpy balance is satisfied at Tt7
 * 4. Applies pressure loss across the afterburner duct
 * 5. Updates FAR — additive on top of existing core FAR
 * 6. Updates mass flow — adds fuel mass
 * 7. Writes real gas Cp and gamma to flowOut via Thermo at exit conditions
 *
 * DIFFERENCE FROM COMBUSTOR:
 *   Location  : after turbine — Tt_in ~1100 K vs ~500 K for combustor
 *   FAR       : additive — gas already has FAR from core combustion
 *   dPqP      : higher — typically 5–8% vs 3–5% for combustor
 *   Nozzle    : requires variable area — Nozzle Ath must increase when lit
 *
 * THERMODYNAMIC EQUATIONS (real gas via Thermo):
 *   H_in      = Thermo::getH(Tt_in, FAR_core)     — inlet enthalpy
 *   FAR_ab    = iterated via enthalpy balance at Tt7:
 *               H_exit(Tt7, FAR_total) = H_in + FAR_ab × LHV × eff_ab / (1 + FAR_total)
 *   FAR_total = FAR_core + FAR_ab × (1 + FAR_core)
 *               — accounts for the fact that afterburner fuel burns
 *                 in a mixture that already contains combustion products
 *   Tt_exit   = Tt7                               — set directly
 *   Pt_exit   = Pt_in × (1 - dPqP)               — pressure loss
 *   W_exit    = W_in × (1 + FAR_total) / (1 + FAR_core)
 *               — mass flow increase from afterburner fuel only
 *   Cp        = Thermo::getCp(Tt7, FAR_total)     — written to flowOut
 *   gamma     = Thermo::getGamma(Tt7, FAR_total)  — written to flowOut
 *
 * VARIABLE NOZZLE AREA:
 * When the afterburner is lit, the nozzle throat area must increase
 * to pass the higher volume flow. The nozzle Ath is calculated from
 * the flow function — it adjusts automatically to the exit conditions.
 * Future work will add explicit variable geometry nozzle scheduling.
 *
 * SHAFT WORK:
 *   getWork() returns 0.0 — no shaft interaction
 *   (inherited default from Element)
 *
 * NAMING CONVENTION — NPSS:
 *   Tt7    — afterburner exit total temperature [K]
 *   FAR_ab — afterburner fuel-air ratio increment [-]
 *   eff_ab — afterburner combustion efficiency [-]
 *   dPqP   — total pressure loss fraction [-]
 *   LHV    — fuel lower heating value [J/kg]
 *
 * TYPICAL VALUES:
 *   Tt7    : 2100 – 2300 K
 *   eff_ab : 0.95 – 0.98
 *   dPqP   : 0.05 – 0.08
 *
 * FUTURE WORK:
 *   Variable LHV for different fuel types.
 *   Afterburner lit/unlit mode switch.
 *   NOx and smoke emissions modeling.
 */

class Afterburner : public Element
{
public:
    // Constructor
    // Tt7    — afterburner exit total temperature [K]
    // eff_ab — combustion efficiency [-]
    // dPqP   — total pressure loss fraction [-]
    // LHV    — fuel lower heating value [J/kg], default Jet-A
    Afterburner(double Tt7,
                double eff_ab = 0.97,
                double dPqP   = 0.06,
                double LHV    = 43100000.0) noexcept;

    // Implements Element::compute() — performs afterburner thermodynamics
    void compute() noexcept override;

    // FAR increment from afterburner fuel addition — available after compute()
    double FAR_ab = 0.0;

private:
    // Members always initialized via constructor — no in-class defaults needed.
    double Tt7;     // Afterburner exit total temperature [K]
    double eff_ab;  // Combustion efficiency [-]
    double dPqP;    // Total pressure loss fraction [-]
    double LHV;     // Fuel lower heating value [J/kg]
};

#endif // AFTERBURNER_H