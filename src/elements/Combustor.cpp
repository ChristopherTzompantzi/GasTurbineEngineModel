#include "Combustor.h"
#include "Thermo.h"
#include <cmath>

/*
 * Combustor.cpp
 * -------------
 * Implements the Combustor element thermodynamics.
 *
 * COMPUTE SEQUENCE (both modes):
 * 1. Read inlet conditions from flowIn (Pt, Tt)
 * 2. Compute inlet enthalpy via Thermo::getH(Tt_in, FAR_in)
 * 3. Tt4 mode — iterate FAR until enthalpy balance is satisfied at Tt4
 *    FAR mode — compute H_exit from energy balance, then Tt_exit via Thermo::getT
 * 4. Apply fractional pressure loss:  Pt_exit = Pt_inlet × (1 - dPqP)
 * 5. Update flowOut with exit conditions (Pt, Tt, W, MN, FAR)
 * 6. Write real gas Cp and gamma to flowOut via Thermo at exit conditions
 */

Combustor::Combustor(CombustorMode mode,
                     double Tt4,
                     double FAR_in,
                     double eff_b,
                     double dPqP,
                     double LHV) noexcept
    : mode(mode)
    , Tt4(Tt4)
    , FAR_in(FAR_in)
    , eff_b(eff_b)
    , dPqP(dPqP)
    , LHV(LHV)
{}

void Combustor::compute() noexcept
{
    // Step 1 — Read inlet conditions
    const double Pt_in = flowIn.Pt;
    const double Tt_in = flowIn.Tt;
    const double FAR_inlet = flowIn.FAR;

    // Step 2 — Inlet specific enthalpy via Thermo
    const double H_in = Thermo::getH(Tt_in, FAR_inlet);

    double FAR_exit = 0.0;
    double Tt_out   = 0.0;

    if (mode == CombustorMode::Tt4) {
        // Step 3 — Tt4 mode: iterate FAR until enthalpy balance is satisfied
        //
        // Energy balance per unit mass of air+fuel mixture:
        //   H_exit(Tt4, FAR) = H_in + FAR × LHV × eff_b / (1 + FAR)
        //
        // Rearranged for FAR with Newton iteration:
        //   f(FAR) = H_exit(Tt4, FAR) - H_in - FAR × LHV × eff_b / (1 + FAR) = 0
        //
        // Initial guess from perfect gas approximation
        const double Cp_guess = Thermo::getCp(Tt_in, FAR_inlet);
        FAR_exit = (Cp_guess * (Tt4 - Tt_in)) / (LHV * eff_b - Cp_guess * Tt4);

        constexpr double tol      = 1.0e-6;   // FAR convergence tolerance
        constexpr int    max_iter = 50;

        for (int i = 0; i < max_iter; ++i) {
            const double H_exit_thermo = Thermo::getH(Tt4, FAR_exit);
            const double H_exit_energy = H_in
                                       + FAR_exit * LHV * eff_b
                                       / (1.0 + FAR_exit);
            const double residual = H_exit_thermo - H_exit_energy;

            // Derivative dResidual/dFAR — numerical finite difference
            constexpr double dFAR = 1.0e-6;
            const double H_exit_thermo_p = Thermo::getH(Tt4, FAR_exit + dFAR);
            const double H_exit_energy_p = H_in
                                         + (FAR_exit + dFAR) * LHV * eff_b
                                         / (1.0 + FAR_exit + dFAR);
            const double dRdFAR = (H_exit_thermo_p - H_exit_energy_p
                                 - residual) / dFAR;

            if (std::abs(dRdFAR) < 1.0e-12) break;   // singular — stop
            const double dFAR_step = residual / dRdFAR;
            FAR_exit -= dFAR_step;

            if (std::abs(dFAR_step) < tol) break;
        }

        Tt_out = Tt4;
    }
    else {
        // Step 3 — FAR mode: compute H_exit from energy balance
        // then invert to find Tt_exit via Thermo::getT
        //
        // H_exit = H_in + FAR × LHV × eff_b / (1 + FAR)
        FAR_exit = FAR_in;
        const double H_exit = H_in
                            + FAR_exit * LHV * eff_b
                            / (1.0 + FAR_exit);

        // Step 3b — invert enthalpy to find exit temperature
        Tt_out = Thermo::getT(H_exit, FAR_exit);
    }

    // Step 4 — Apply fractional pressure loss
    // Pt_exit = Pt_inlet × (1 - dPqP)
    flowOut.Pt = Pt_in * (1.0 - dPqP);

    // Step 5 — Update exit conditions
    flowOut.Tt  = Tt_out;
    flowOut.W   = flowIn.W * (1.0 + FAR_exit);  // mass flow increases with fuel
    flowOut.MN  = flowIn.MN;
    flowOut.FAR = FAR_exit;

    // Step 6 — Real gas Cp and gamma at exit conditions
    flowOut.Cp    = Thermo::getCp   (flowOut.Tt, flowOut.FAR);
    flowOut.gamma = Thermo::getGamma(flowOut.Tt, flowOut.FAR);
}