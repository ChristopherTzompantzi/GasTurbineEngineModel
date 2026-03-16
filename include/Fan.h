#ifndef FAN_H
#define FAN_H

#include "Element.h"
#include "Thermo.h"

/*
 * Fan.h
 * -----
 * Models the fan of a turbofan engine.
 * Aerodynamically identical to a compressor — raises total pressure
 * and temperature of the ENTIRE inlet mass flow (core + bypass).
 *
 * WHAT THE FAN DOES:
 * 1. Receives total inlet mass flow from Inlet
 * 2. Raises total pressure by fan pressure ratio (PR_f)
 * 3. Raises total temperature accounting for isentropic efficiency
 * 4. Passes full mass flow to Splitter — BPR is handled there
 *
 * THERMODYNAMIC EQUATIONS (identical to Compressor — real gas via Thermo):
 *   Pt_exit       = Pt_inlet × PR_f
 *   gamma         = Thermo::getGamma(Tt_inlet, FAR)   — real gas at inlet conditions
 *   Tt_exit_ideal = Tt_inlet × PR_f^((γ-1)/γ)
 *   Tt_exit       = Tt_inlet + (Tt_exit_ideal - Tt_inlet) / eff_f
 *   dHt           = Cp × (Tt_exit - Tt_inlet)         [J/kg]
 *   Cp            = Thermo::getCp(Tt_exit, FAR)        — written to flowOut
 *   gamma_exit    = Thermo::getGamma(Tt_exit, FAR)     — written to flowOut
 *
 * WHY FAN IS A SEPARATE CLASS FROM COMPRESSOR:
 * Architecturally the Fan sits on the LP shaft alongside the LP Turbine.
 * The Compressor sits on the HP shaft alongside the HP Turbine.
 * Keeping them as separate classes makes the shaft connections explicit
 * and readable in main.cpp, and matches NPSS element conventions.
 *
 * SHAFT CONNECTION:
 *   LP Shaft: Fan ←→ LP Turbine
 *   HP Shaft: Compressor ←→ HP Turbine
 *
 * SIGN CONVENTION:
 *   getWork() returns -dHt — fan consumes power from LP shaft
 *
 * TYPICAL VALUES:
 *   PR_f  : 1.4 – 1.8   (vs 10+ for core compressor)
 *   eff_f : 0.88 – 0.92
 *
 * FUTURE WORK (performance maps):
 *   Fan performance map — PR_f and eff_f as functions of corrected
 *   flow (Wc) and corrected speed (Nc).
 *   Fan diameter and tip speed constraints.
 */

class Fan : public Element
{
public:
    // Constructor — fan pressure ratio and isentropic efficiency
    Fan(double PR_f, double eff_f) noexcept;

    // Implements Element::compute() — performs fan thermodynamics
    void compute() noexcept override;

    // Returns negative dHt — fan consumes power from LP shaft
    // Sign convention matches Element base class: negative = power out of shaft
    double getWork() const noexcept override { return -dHt; }

    // Specific work input [J/kg] — available after compute()
    // dHt = Cp × (Tt_exit - Tt_inlet)
    double dHt = 0.0;

private:
    // Members always initialized via constructor — no in-class defaults needed.
    double PR_f;    // Fan pressure ratio [-]
    double eff_f;   // Fan isentropic efficiency [-]
};

#endif // FAN_H