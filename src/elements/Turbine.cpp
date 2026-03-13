#include "Turbine.h"
#include <cmath>

/*
 * Turbine.cpp
 * -----------
 * Implements the Turbine element thermodynamics.
 *
 * COMPUTE SEQUENCE:
 * 1. Read inlet conditions from flowIn (Pt, Tt, gamma, Cp)
 * 2. Compute exit total pressure      — Pt_exit = Pt_inlet / PR_t
 * 3. Compute ideal exit temperature   — isentropic expansion
 * 4. Compute actual exit temperature  — corrected for efficiency
 * 5. Compute dHt                      — specific work extracted [J/kg]
 * 6. Pass W, MN, FAR, gamma, Cp through unchanged
 *
 * NOTE ON EFFICIENCY DIRECTION:
 * Turbine efficiency MULTIPLIES the available temperature drop.
 * This is opposite to the compressor where efficiency DIVIDES.
 * Turbine losses reduce work extracted (you get less than ideal).
 * Compressor losses increase work required (you pay more than ideal).
 */

Turbine::Turbine(double PR_t, double eff_t) noexcept
    : PR_t(PR_t)
    , eff_t(eff_t)
{}

void Turbine::compute() noexcept
{
    // Read inlet conditions
    double Pt_in  = flowIn.Pt;
    double Tt_in  = flowIn.Tt;
    double gamma  = flowIn.gamma;
    double Cp     = flowIn.Cp;

    // Exit total pressure
    // Pt_exit = Pt_inlet / PR_t
    flowOut.Pt = Pt_in / PR_t;

    // Ideal exit temperature (isentropic expansion)
    // Tt_ideal = Tt_inlet × (1/PR_t)^((γ-1)/γ)
    double exponent   = (gamma - 1.0) / gamma;
    double Tt_ideal   = Tt_in * std::pow(1.0 / PR_t, exponent);

    // Actual exit temperature (accounting for losses)
    // Tt_exit = Tt_inlet - eff_t × (Tt_inlet - Tt_ideal)
    flowOut.Tt = Tt_in - eff_t * (Tt_in - Tt_ideal);

    // Specific work extracted [J/kg]
    // dHt = Cp × (Tt_inlet - Tt_exit)
    dHt = Cp * (Tt_in - flowOut.Tt);

    // Pass remaining properties through unchanged
    flowOut.W     = flowIn.W;
    flowOut.MN    = flowIn.MN;
    flowOut.FAR   = flowIn.FAR;
    flowOut.gamma = flowIn.gamma;
    flowOut.Cp    = flowIn.Cp;
}

void Turbine::setPR(double PR_t_new) noexcept
{
    PR_t = PR_t_new;
}