#include "Afterburner.h"
#include "Thermo.h"
#include <cmath>

/*
 * Afterburner.cpp
 * ---------------
 * Implements Afterburner element thermodynamics.
 * Similar to Combustor.cpp — see Combustor.cpp for base derivation.
 *
 * COMPUTE SEQUENCE:
 * 1. Read turbine exit conditions from flowIn (Pt, Tt, FAR_core)
 * 2. Compute inlet enthalpy via Thermo::getH(Tt_in, FAR_core)
 * 3. Iterate FAR_ab until enthalpy balance is satisfied at Tt7
 * 4. Compute total FAR — additive on existing core FAR
 * 5. Apply pressure loss
 * 6. Update mass flow with afterburner fuel addition
 * 7. Write results to flowOut
 * 8. Write real gas Cp and gamma to flowOut via Thermo at exit conditions
 *
 * FAR ACCOUNTING:
 * The afterburner adds fuel to a gas mixture that already contains
 * combustion products from the core. FAR_ab is solved via Newton
 * iteration on the enthalpy balance at Tt7 — same approach as
 * Combustor Tt4 mode:
 *
 *   H_exit(Tt7, FAR_total) = H_in + FAR_ab × LHV × eff_ab / (1 + FAR_total)
 *
 *   FAR_total = FAR_core + FAR_ab × (1 + FAR_core)
 *
 * The (1 + FAR_core) factor scales FAR_ab because the afterburner
 * fuel is injected per unit of mixed gas flow, not per unit of
 * original air flow. This preserves correct mass accounting.
 *
 * Source: Mattingly, J.D., "Elements of Gas Turbine Propulsion",
 *         McGraw-Hill, 1996. Chapter 5 — afterburner cycle analysis.
 */

Afterburner::Afterburner(double Tt7,
                         double eff_ab,
                         double dPqP,
                         double LHV) noexcept
    : Tt7(Tt7)
    , eff_ab(eff_ab)
    , dPqP(dPqP)
    , LHV(LHV)
{}

void Afterburner::compute() noexcept
{
    // Step 1 — Read inlet conditions
    const double Pt_in    = flowIn.Pt;
    const double Tt_in    = flowIn.Tt;
    const double W_in     = flowIn.W;
    const double FAR_core = flowIn.FAR;

    // Step 2 — Inlet enthalpy via Thermo
    const double H_in = Thermo::getH(Tt_in, FAR_core);

    // Step 3 — Iterate FAR_ab until enthalpy balance is satisfied at Tt7
    //
    // Energy balance:
    //   H_exit(Tt7, FAR_total) = H_in + FAR_ab × LHV × eff_ab / (1 + FAR_total)
    // where FAR_total = FAR_core + FAR_ab × (1 + FAR_core)
    //
    // Initial guess from perfect gas approximation
    const double Cp_guess = Thermo::getCp(Tt_in, FAR_core);
    FAR_ab = Cp_guess * (Tt7 - Tt_in) / (LHV * eff_ab - Cp_guess * Tt7);

    constexpr double tol      = 1.0e-6;   // FAR convergence tolerance
    constexpr int    max_iter = 50;

    for (int i = 0; i < max_iter; ++i) {
        const double FAR_total_i   = FAR_core + FAR_ab * (1.0 + FAR_core);
        const double H_exit_thermo = Thermo::getH(Tt7, FAR_total_i);
        const double H_exit_energy = H_in
                                   + FAR_ab * LHV * eff_ab
                                   / (1.0 + FAR_total_i);
        const double residual = H_exit_thermo - H_exit_energy;

        // Derivative dResidual/dFAR_ab — numerical finite difference
        constexpr double dFAR = 1.0e-6;
        const double FAR_total_p   = FAR_core + (FAR_ab + dFAR) * (1.0 + FAR_core);
        const double H_exit_thermo_p = Thermo::getH(Tt7, FAR_total_p);
        const double H_exit_energy_p = H_in
                                     + (FAR_ab + dFAR) * LHV * eff_ab
                                     / (1.0 + FAR_total_p);
        const double dRdFAR = (H_exit_thermo_p - H_exit_energy_p - residual) / dFAR;

        if (std::abs(dRdFAR) < 1.0e-12) break;
        const double dFAR_step = residual / dRdFAR;
        FAR_ab -= dFAR_step;

        if (std::abs(dFAR_step) < tol) break;
    }

    // Step 4 — Total FAR — additive on existing core FAR
    // FAR_total = FAR_core + FAR_ab × (1 + FAR_core)
    const double FAR_total = FAR_core + FAR_ab * (1.0 + FAR_core);

    // Step 5 — Exit total pressure — afterburner duct pressure loss
    const double Pt_exit = Pt_in * (1.0 - dPqP);

    // Step 6 — Exit mass flow — adds afterburner fuel mass
    // W_exit = W_in × (1 + FAR_total) / (1 + FAR_core)
    const double W_exit = W_in * (1.0 + FAR_total) / (1.0 + FAR_core);

    // Step 7 — Write results to flowOut
    flowOut.Pt  = Pt_exit;
    flowOut.Tt  = Tt7;
    flowOut.W   = W_exit;
    flowOut.MN  = flowIn.MN;
    flowOut.FAR = FAR_total;

    // Step 8 — Real gas Cp and gamma at exit conditions
    flowOut.Cp    = Thermo::getCp   (flowOut.Tt, flowOut.FAR);
    flowOut.gamma = Thermo::getGamma(flowOut.Tt, flowOut.FAR);
}