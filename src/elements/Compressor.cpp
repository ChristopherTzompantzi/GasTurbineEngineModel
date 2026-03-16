#include "Compressor.h"
#include "Thermo.h"
#include <cmath>

/*
 * Compressor.cpp
 * --------------
 * Implements the Compressor element thermodynamics.
 *
 * COMPUTE SEQUENCE:
 * 1. Read inlet conditions from flowIn (Pt, Tt)
 * 2. Evaluate real gas gamma and Cp at inlet conditions via Thermo
 * 3. Compute exit total pressure     — Pt_exit = Pt_inlet × PR
 * 4. Compute ideal exit temperature  — isentropic: Tt_ideal = Tt_inlet × PR^((γ-1)/γ)
 * 5. Compute actual exit temperature — corrected for efficiency losses
 * 6. Compute dHt                     — total enthalpy rise [J/kg] = Cp × ΔTt
 *                                      (named dHt, not wc — Wc is reserved for
 *                                      corrected mass flow in future work; see Compressor.h)
 * 7. Pass W, MN, FAR through unchanged
 * 8. Write real gas Cp and gamma to flowOut via Thermo at exit conditions
 */

Compressor::Compressor(double PR, double eff) noexcept
    : PR(PR)
    , eff(eff)
{}

void Compressor::compute() noexcept
{
    // Step 1 — Read inlet conditions
    const double Pt_in = flowIn.Pt;
    const double Tt_in = flowIn.Tt;

    // Step 2 — Real gas properties at inlet conditions
    const double gamma = Thermo::getGamma(Tt_in, flowIn.FAR);
    const double Cp    = Thermo::getCp   (Tt_in, flowIn.FAR);

    // Step 3 — Exit total pressure
    // Pt_exit = Pt_inlet × PR
    flowOut.Pt = Pt_in * PR;

    // Step 4 — Ideal exit temperature (isentropic compression)
    // Tt_ideal = Tt_inlet × PR^((γ-1)/γ)
    double exponent   = (gamma - 1.0) / gamma;
    double Tt_ideal   = Tt_in * std::pow(PR, exponent);

    // Step 5 — Actual exit temperature (accounting for losses)
    // Tt_exit = Tt_inlet + (Tt_ideal - Tt_inlet) / eff
    flowOut.Tt = Tt_in + (Tt_ideal - Tt_in) / eff;

    // Step 6 — Total enthalpy rise [J/kg]
    // dHt = Cp × (Tt_exit - Tt_inlet)
    dHt = Cp * (flowOut.Tt - Tt_in);

    // Step 7 — Pass remaining properties through unchanged
    flowOut.W   = flowIn.W;
    flowOut.MN  = flowIn.MN;
    flowOut.FAR = flowIn.FAR;

    // Step 8 — Real gas Cp and gamma at exit conditions — evaluated at Tt_exit
    flowOut.Cp    = Thermo::getCp   (flowOut.Tt, flowOut.FAR);
    flowOut.gamma = Thermo::getGamma(flowOut.Tt, flowOut.FAR);
}