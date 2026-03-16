#ifndef COMPRESSOR_H
#define COMPRESSOR_H

#include "Element.h"
#include "Thermo.h"

/*
 * Compressor.h
 * ------------
 * Declares the Compressor element — raises total pressure and temperature
 * by doing work on the flow. Inherits from Element.
 *
 * WHAT THE COMPRESSOR DOES:
 * 1. Receives flow from Inlet at compressor face conditions
 * 2. Evaluates real gas gamma and Cp at inlet conditions via Thermo
 * 3. Raises total pressure by pressure ratio (PR)
 * 4. Raises total temperature accounting for isentropic efficiency (eff)
 * 5. Writes real gas Cp and gamma to flowOut via Thermo at exit conditions
 * 6. Mass flow passes through unchanged
 *
 * THERMODYNAMIC EQUATIONS (real gas via Thermo):
 *   Pt_exit        = Pt_inlet × PR
 *   gamma          = Thermo::getGamma(Tt_inlet, FAR)   — real gas at inlet
 *   Tt_exit_ideal  = Tt_inlet × PR^((γ-1)/γ)
 *   Tt_exit        = Tt_inlet + (Tt_exit_ideal - Tt_inlet) / eff
 *   Cp             = Thermo::getCp(Tt_exit, FAR)        — written to flowOut
 *   gamma_exit     = Thermo::getGamma(Tt_exit, FAR)     — written to flowOut
 *
 * ISENTROPIC EFFICIENCY (eff):
 * Also called adiabatic efficiency — these are equivalent for a compressor
 * since no heat transfer across the casing is assumed (adiabatic process).
 * eff = ideal work / actual work = (Tt_ideal - Tt_in) / (Tt_actual - Tt_in)
 * eff = 1.0 means perfect isentropic compression (no losses).
 * Typical range: 0.85 to 0.92 for modern compressors.
 *
 * FUTURE WORK — POLYTROPIC EFFICIENCY:
 * Polytropic efficiency (η_p) measures small-stage efficiency independently
 * of pressure ratio. At high PR (20+), isentropic efficiency is noticeably
 * lower than polytropic efficiency even for the same blade aerodynamics.
 * Future work will add polytropic efficiency as an alternative input mode,
 * converting internally to isentropic for the temperature calculation:
 *   η_is = (PR^((γ-1)/γ) - 1) / (PR^((γ-1)/(γ×η_p)) - 1)
 *
 * COMPRESSOR WORK:
 * Specific work [J/kg] = Cp × (Tt_exit - Tt_inlet)
 * Stored in dHt for turbine matching and cycle analysis.
 * The turbine must provide at least this much work to drive
 * the compressor — this is the shaft power balance equation.
 *
 * NAMING CONVENTION — NPSS:
 * PR  — pressure ratio
 * eff — isentropic efficiency
 * dHt — total enthalpy rise [J/kg] (specific work added to the flow)
 * Wc  — corrected mass flow (future work — reserved, do NOT use for other purposes)
 * Nc  — corrected speed (future work)
 *
 * FUTURE WORK (performance maps):
 * PR and eff will be replaced by compressor map lookup
 * based on corrected flow (Wc) and corrected speed (Nc).
 * Surge margin and stall detection will be added at that point.
 */

class Compressor : public Element {
public:
    // Constructor — sets pressure ratio and isentropic efficiency
    Compressor(double PR, double eff) noexcept;

    // Implements Element::compute() — performs compressor thermodynamics
    void compute() noexcept override;

    // Returns negative dHt — compressor consumes power from shaft
    // Sign convention matches Element base class: negative = power out of shaft
    double getWork() const noexcept override { return -dHt; }

    // Total enthalpy rise [J/kg] — available after compute()
    // dHt = Cp × (Tt_exit - Tt_inlet) — the specific work added to the flow.
    // Used by the solver for turbine work matching (turbine must supply at
    // least this much enthalpy drop to drive the compressor).
    //
    // NAMING NOTE — why dHt and not wc:
    // In NPSS, Wc (capital W) is firmly reserved for corrected mass flow.
    // Using wc (lowercase) for specific work would create a near-identical
    // identifier that means something completely different — a silent source
    // of confusion when Wc is added in future work. dHt (delta total enthalpy)
    // is the thermodynamically correct name and has zero collision risk.
    double dHt = 0.0;

private:
    // Members always initialized via constructor — no in-class defaults needed.
    double PR;   // Overall pressure ratio [-]
    double eff;  // Isentropic efficiency [-]
};

#endif // COMPRESSOR_H