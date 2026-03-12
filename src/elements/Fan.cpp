#include "Fan.h"
#include <cmath>

/*
 * Fan.cpp
 * -------
 * Implements Fan element thermodynamics.
 * Equations identical to Compressor — see Compressor.cpp for derivation.
 *
 * COMPUTE SEQUENCE:
 * 1. Read inlet total conditions from flowIn
 * 2. Compute exit total pressure      — Pt_exit = Pt_inlet × PR_f
 * 3. Compute ideal exit temperature   — isentropic: Tt_ideal = Tt_in × PR_f^((γ-1)/γ)
 * 4. Compute actual exit temperature  — corrected for efficiency losses
 * 5. Compute specific work dHt        — Cp × (Tt_exit - Tt_in)
 * 6. Write results to flowOut
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
    double Pt_in  = flowIn.Pt;
    double Tt_in  = flowIn.Tt;
    double gamma  = flowIn.gamma;
    double Cp     = flowIn.Cp;

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

    // Write to flowOut — mass flow, FAR, gamma, Cp pass through unchanged
    flowOut.Pt    = Pt_exit;
    flowOut.Tt    = Tt_exit;
    flowOut.W     = flowIn.W;
    flowOut.MN    = flowIn.MN;
    flowOut.FAR   = flowIn.FAR;
    flowOut.gamma = gamma;
    flowOut.Cp    = Cp;
}