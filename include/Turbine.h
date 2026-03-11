#ifndef TURBINE_H
#define TURBINE_H

#include "Element.h"

/*
 * Turbine.h
 * ---------
 * Declares the Turbine element — extracts work from the flow by expanding
 * hot combustion gases, driving the compressor shaft.
 * Inherits from Element.
 *
 * WHAT THE TURBINE DOES (Phase 1):
 * 1. Receives hot gas from Combustor exit
 * 2. Expands the gas — drops total pressure and temperature
 * 3. Extracts shaft work to drive the compressor
 * 4. Mass flow passes through unchanged
 *
 * THERMODYNAMIC EQUATIONS (Perfect Gas — Phase 1):
 *   Pt_exit       = Pt_inlet / PR_t
 *   Tt_exit_ideal = Tt_inlet × (1/PR_t)^((γ-1)/γ)
 *   Tt_exit       = Tt_inlet - eff_t × (Tt_inlet - Tt_exit_ideal)
 *   dHt           = Cp × (Tt_inlet - Tt_exit)   [J/kg]
 *
 * EFFICIENCY DIRECTION — WHY TURBINE MULTIPLIES, COMPRESSOR DIVIDES:
 * Compressor fights an adverse pressure gradient — losses mean MORE
 * work is needed than ideal → divide by eff.
 * Turbine works with a favorable pressure gradient — losses mean LESS
 * work is extracted than ideal → multiply by eff.
 *
 * SHAFT POWER BALANCE:
 * The turbine must supply exactly enough work to drive the compressor.
 * This is the key cycle constraint enforced by the solver:
 *   (1 + FAR) × dHt_turbine = dHt_compressor
 * The (1+FAR) accounts for the extra fuel mass the turbine processes.
 *
 * NAMING CONVENTION — NPSS:
 *   PR_t  — turbine pressure ratio [-]
 *   eff_t — turbine isentropic efficiency [-]
 *   dHt   — specific work extracted [J/kg]
 *
 * PHASE 2 CONSIDERATION:
 * PR_t and eff_t will be replaced by turbine map lookup.
 * Cooling flow modeling — HP turbine blade cooling air.
 * Polytropic efficiency as alternative input mode.
 */

class Turbine : public Element {
public:
    // Constructor — sets pressure ratio and isentropic efficiency
    Turbine(double PR_t, double eff_t) noexcept;

    // Implements Element::compute() — performs turbine thermodynamics
    void compute() noexcept override;

    // Specific work extracted [J/kg] — available after compute()
    // Used by solver for shaft power balance with compressor
    double dHt = 0.0;

private:
    // Members always initialized via constructor — no in-class defaults needed.
    double PR_t;    // Turbine pressure ratio [-]
    double eff_t;   // Turbine isentropic efficiency [-]
};

#endif // TURBINE_H