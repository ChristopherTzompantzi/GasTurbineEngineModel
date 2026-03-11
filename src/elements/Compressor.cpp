#include "Compressor.h"
#include <cmath>

/*
 * Compressor.cpp
 * --------------
 * Implements the Compressor element thermodynamics.
 *
 * COMPUTE SEQUENCE:
 * 1. Read inlet conditions from flowIn (Pt, Tt, gamma, Cp)
 * 2. Compute exit total pressure     — Pt_exit = Pt_inlet × PR
 * 3. Compute ideal exit temperature  — isentropic: Tt_ideal = Tt_inlet × PR^((γ-1)/γ)
 * 4. Compute actual exit temperature — corrected for efficiency losses
 * 5. Compute dHt                     — total enthalpy rise [J/kg] = Cp × ΔTt
 *                                      (named dHt, not wc — Wc is reserved for
 *                                      corrected mass flow in Phase 2; see Compressor.h)
 * 6. Pass W, MN, FAR, gamma, Cp through unchanged
 */

Compressor::Compressor(double PR, double eff) noexcept
    : PR(PR)
    , eff(eff)
{}

void Compressor::compute() noexcept
{
    // Read inlet conditions
    double Pt_in  = flowIn.Pt;
    double Tt_in  = flowIn.Tt;
    double gamma  = flowIn.gamma;
    double Cp     = flowIn.Cp;

    // Step 1 — Exit total pressure
    // Pt_exit = Pt_inlet × PR
    flowOut.Pt = Pt_in * PR;

    // Step 2 — Ideal exit temperature (isentropic compression)
    // Tt_ideal = Tt_inlet × PR^((γ-1)/γ)
    double exponent   = (gamma - 1.0) / gamma;
    double Tt_ideal   = Tt_in * std::pow(PR, exponent);

    // Step 3 — Actual exit temperature (accounting for losses)
    // Tt_exit = Tt_inlet + (Tt_ideal - Tt_inlet) / eff
    flowOut.Tt = Tt_in + (Tt_ideal - Tt_in) / eff;

    // Step 4 — Total enthalpy rise [J/kg]
    // dHt = Cp × (Tt_exit - Tt_inlet)
    dHt = Cp * (flowOut.Tt - Tt_in);

    // Step 5 — Pass remaining properties through unchanged
    flowOut.W     = flowIn.W;
    flowOut.MN    = flowIn.MN;
    flowOut.FAR   = flowIn.FAR;
    flowOut.gamma = flowIn.gamma;
    flowOut.Cp    = flowIn.Cp;
}