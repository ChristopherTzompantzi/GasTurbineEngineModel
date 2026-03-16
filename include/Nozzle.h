#ifndef NOZZLE_H
#define NOZZLE_H

#include "Element.h"
#include "ISA.h"
#include "Thermo.h"

/*
 * Nozzle.h
 * --------
 * Declares the Nozzle element — converts remaining total pressure
 * into jet velocity and thrust, completing the Brayton cycle.
 * Inherits from Element.
 *
 * WHAT THE NOZZLE DOES:
 * 1. Receives expanded gas from Turbine exit
 * 2. Evaluates real gas gamma and Cp at inlet conditions via Thermo
 * 3. Determines choked or unchoked condition from NPR
 * 4. Expands gas to ambient (or critical) pressure
 * 5. Computes jet velocity using isentropic relations
 * 6. Computes gross thrust Fg
 *
 * CHOKED vs UNCHOKED:
 *   NPR_crit = ((γ+1)/2)^(γ/(γ-1)) ≈ 1.893 for γ=1.4
 *   NPR < NPR_crit → unchoked → Ps_exit = Ps_ambient
 *   NPR ≥ NPR_crit → choked  → Ps_exit = Pt_inlet / NPR_crit
 *
 * THERMODYNAMIC EQUATIONS (real gas via Thermo):
 *   NPR      = Pt_inlet / Ps_ambient
 *   gamma    = Thermo::getGamma(Tt_inlet, FAR)   — real gas at inlet
 *   Cp       = Thermo::getCp(Tt_inlet, FAR)       — real gas at inlet
 *   Vjet     = Cfg × sqrt(2 × Cp × Tt_inlet × (1 - (Ps_exit/Pt_inlet)^((γ-1)/γ)))
 *   Fg       = W × Vjet + (Ps_exit - Ps_ambient) × Ath
 *
 * THROAT AREA (Ath) — CALCULATED FROM FLOW CONDITIONS:
 *   Ath is derived from the continuity equation and Flow Function (FF).
 *   This avoids hardcoding a geometric parameter — the nozzle
 *   sizes itself based on actual flow conditions.
 *   FF  = sqrt(γ/R) × MN × (1 + (γ-1)/2 × MN²)^(-(γ+1)/(2×(γ-1)))
 *   Ath = W × sqrt(Tt_inlet) / (Pt_inlet × FF)
 *
 * FUTURE WORK — Ath AS CONSTRUCTOR PARAMETER:
 *   For convergent-divergent nozzles and detailed thrust accounting,
 *   Ath should be accepted as a fixed geometric input.
 *   This allows modeling of over/under-expanded nozzles and
 *   more accurate pressure thrust calculation at off-design conditions.
 *
 * NAMING CONVENTION — NPSS:
 *   NPR     — nozzle pressure ratio [-]
 *   NPR_crit— critical nozzle pressure ratio [-]
 *   Cfg     — nozzle velocity coefficient / discharge coefficient [-]
 *   Vjet    — jet exit velocity [m/s]
 *   Fg      — gross thrust [N]
 *   Ath     — nozzle throat area [m²]
 *   FF      — flow function [-]
 *
 * FUTURE WORK:
 *   Convergent-divergent nozzle modeling.
 *   Fixed Ath as geometric input for detailed thrust accounting.
 *   Nozzle performance map (Cfg vs NPR).
 *   Thrust coefficient (Ct) calculation.
 */

class Nozzle : public Element {
public:
    // Constructor — sets altitude and nozzle efficiency
    Nozzle(double altitude_m, double Cfg = 0.98) noexcept;

    // Implements Element::compute() — performs nozzle thermodynamics
    void compute() noexcept override;

    // Outputs available after compute()
    double Fg   = 0.0;   // Gross thrust [N]
    double Vjet = 0.0;   // Jet exit velocity [m/s]
    double Ath  = 0.0;   // Calculated throat area [m²]
    double NPR  = 0.0;   // Nozzle pressure ratio [-]

private:
    // Members always initialized via constructor — no in-class defaults needed.
    double altitude_m;   // Flight altitude [m] — for ISA ambient conditions
    double Cfg;          // Nozzle velocity coefficient [-]
};

#endif // NOZZLE_H