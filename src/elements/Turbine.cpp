#include "Turbine.h"
#include "Thermo.h"
#include <cmath>

/*
 * Turbine.cpp
 * -----------
 * Implements the Turbine element thermodynamics.
 *
 * COMPUTE SEQUENCE:
 * 1. Read inlet conditions from flowIn (Pt, Tt)
 * 2. Evaluate real gas gamma and Cp at inlet conditions via Thermo
 * 3. Compute exit total pressure      — Pt_exit = Pt_inlet / PR_t
 * 4. Compute ideal exit temperature   — isentropic expansion
 * 5. Compute actual exit temperature  — corrected for efficiency
 * 6. Compute dHt                      — specific work extracted [J/kg]
 * 7. Pass W, MN, FAR through unchanged
 * 8. Write real gas Cp and gamma to flowOut via Thermo at exit conditions
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
    // Step 1 — Read inlet conditions
    const double Pt_in = flowIn.Pt;
    const double Tt_in = flowIn.Tt;

    // Step 2 - Real gas properties at inlet conditions
    const double gamma = Thermo::getGamma(Tt_in, flowIn.FAR);
    const double Cp    = Thermo::getCp   (Tt_in, flowIn.FAR);

    // Step 3 — Exit total pressure
    // Pt_exit = Pt_inlet / PR_t
    flowOut.Pt = Pt_in / PR_t;

    // Step 4 — Ideal exit temperature (isentropic expansion)
    // Tt_ideal = Tt_inlet × (1/PR_t)^((γ-1)/γ)
    double exponent   = (gamma - 1.0) / gamma;
    double Tt_ideal   = Tt_in * std::pow(1.0 / PR_t, exponent);

    // Step 5 — Actual exit temperature (accounting for losses)
    // Tt_exit = Tt_inlet - eff_t × (Tt_inlet - Tt_ideal)
    flowOut.Tt = Tt_in - eff_t * (Tt_in - Tt_ideal);

    // Step 6 — Specific work extracted [J/kg]
    // dHt = Cp × (Tt_inlet - Tt_exit)
    dHt = Cp * (Tt_in - flowOut.Tt);

    // Step 7 — Pass remaining properties through unchanged
    flowOut.W   = flowIn.W;
    flowOut.MN  = flowIn.MN;
    flowOut.FAR = flowIn.FAR;

    // Step 8 — Real gas Cp and gamma at exit conditions — evaluated at Tt_exit
    flowOut.Cp    = Thermo::getCp   (flowOut.Tt, flowOut.FAR);
    flowOut.gamma = Thermo::getGamma(flowOut.Tt, flowOut.FAR);
}

void Turbine::setPR(double PR_t_new) noexcept
{
    PR_t = PR_t_new;
}