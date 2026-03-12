#ifndef AFTERBURNER_H
#define AFTERBURNER_H

#include "Element.h"

/*
 * Afterburner.h
 * -------------
 * Models the afterburner (reheat) of a turbojet or turbofan engine.
 * Adds fuel to the turbine exhaust to raise total temperature,
 * producing additional thrust at the cost of high fuel consumption.
 *
 * WHAT THE AFTERBURNER DOES:
 * 1. Receives turbine exit flow on flowIn
 * 2. Adds fuel to raise total temperature to Tt7 (afterburner exit)
 * 3. Applies pressure loss across the afterburner duct
 * 4. Updates FAR — additive on top of existing core FAR
 * 5. Updates mass flow — adds fuel mass
 *
 * DIFFERENCE FROM COMBUSTOR:
 *   Location  : after turbine — Tt_in ~1100 K vs ~500 K for combustor
 *   FAR       : additive — gas already has FAR from core combustion
 *   dPqP      : higher — typically 5–8% vs 3–5% for combustor
 *   Nozzle    : requires variable area — Nozzle Ath must increase when lit
 *
 * THERMODYNAMIC EQUATIONS (Perfect Gas — Phase 1):
 *   FAR_ab    = Cp × (Tt7 - Tt_in) / (LHV × eff_ab - Cp × Tt7)
 *   FAR_total = FAR_core + FAR_ab × (1 + FAR_core)
 *               — accounts for the fact that afterburner fuel burns
 *                 in a mixture that already contains combustion products
 *   Tt_exit   = Tt7                          — set directly
 *   Pt_exit   = Pt_in × (1 - dPqP)          — pressure loss
 *   W_exit    = W_in × (1 + FAR_total) / (1 + FAR_core)
 *               — mass flow increase from afterburner fuel only
 *
 * VARIABLE NOZZLE AREA:
 * When the afterburner is lit, the nozzle throat area must increase
 * to pass the higher volume flow. In Phase 1b the nozzle Ath is
 * calculated from the flow function — it adjusts automatically.
 * Phase 2 will add explicit variable geometry nozzle scheduling.
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
 * PHASE 2:
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