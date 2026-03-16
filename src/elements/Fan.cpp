#include "Fan.h"
#include "Thermo.h"
#include <cmath>

/*
 * Fan.cpp
 * -------
 * Implements Fan element thermodynamics.
 * Equations identical to Compressor — see Compressor.cpp for derivation.
 *
 * COMPUTE SEQUENCE:
 * 1. Read inlet total conditions from flowIn
 * 2. Evaluate real gas gamma and Cp at inlet conditions via Thermo
 * 3. Compute exit total pressure      — Pt_exit = Pt_inlet × PR_f
 * 4. Compute ideal exit temperature   — isentropic: Tt_ideal = Tt_in × PR_f^((γ-1)/γ)
 * 5. Compute actual exit temperature  — corrected for efficiency losses
 * 6. Compute specific work dHt        — Cp × (Tt_exit - Tt_in)
 * 7. Write results to flowOut
 * 8. Write real gas Cp and gamma to flowOut via Thermo at exit conditions
 *
 * Source: Mattingly, J.D., "Elements of Gas Turbine Propulsion",
 *         McGraw-Hill, 1996. Chapter 5 — component thermodynamics.
 */

Fan::Fan(double PR_f, double eff_f) noexcept
    : PR_f(PR_f)
    , eff_f(eff_f)
{}

void Fan::compute() noexcept
{
    // Read inlet conditions
    const double Pt_in = flowIn.Pt;
    const double Tt_in = flowIn.Tt;

    // Real gas properties at inlet conditions — FAR=0 upstream of combustor
    const double gamma = Thermo::getGamma(Tt_in, flowIn.FAR);
    const double Cp    = Thermo::getCp   (Tt_in, flowIn.FAR);

    // Exit total pressure
    // Pt_exit = Pt_inlet × PR_f
    double Pt_exit = Pt_in * PR_f;

    // Ideal (isentropic) exit temperature
    // Tt_ideal = Tt_in × PR_f^((γ-1)/γ)
    double Tt_ideal = Tt_in * std::pow(PR_f, (gamma - 1.0) / gamma);

    // Actual exit temperature — isentropic efficiency accounts for losses
    // Tt_exit = Tt_in + (Tt_ideal - Tt_in) / eff_f
    double Tt_exit = Tt_in + (Tt_ideal - Tt_in) / eff_f;

    // Specific work input to the flow
    // dHt = Cp × (Tt_exit - Tt_in)   [J/kg]
    dHt = Cp * (Tt_exit - Tt_in);

    // Write to flowOut — mass flow and FAR pass through unchanged
    flowOut.Pt  = Pt_exit;
    flowOut.Tt  = Tt_exit;
    flowOut.W   = flowIn.W;
    flowOut.MN  = flowIn.MN;
    flowOut.FAR = flowIn.FAR;

    // Real gas Cp and gamma at exit conditions — evaluated at Tt_exit
    flowOut.Cp    = Thermo::getCp   (flowOut.Tt, flowOut.FAR);
    flowOut.gamma = Thermo::getGamma(flowOut.Tt, flowOut.FAR);
}