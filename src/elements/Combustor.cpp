#include "Combustor.h"

/*
 * Combustor.cpp
 * -------------
 * Implements the Combustor element thermodynamics.
 *
 * COMPUTE SEQUENCE (both modes):
 * 1. Read inlet conditions from flowIn (Pt, Tt, Cp)
 * 2. Tt4 mode  — compute FAR from Tt4:    FAR = Cp×(Tt4-Tt_in) / (LHV×eff_b - Cp×Tt4)
 *    FAR mode  — compute Tt_exit from FAR: Tt_exit = Tt_in + FAR×LHV×eff_b / ((1+FAR)×Cp)
 * 3. Apply fractional pressure loss:       Pt_exit = Pt_inlet × (1 - dPqP)
 * 4. Update flowOut with exit conditions (Pt, Tt, W, MN, FAR, gamma, Cp)
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
    // Read inlet conditions
    double Pt_in  = flowIn.Pt;
    double Tt_in  = flowIn.Tt;
    double Cp     = flowIn.Cp;

    double FAR    = 0.0;
    double Tt_out = 0.0;

    if (mode == CombustorMode::Tt4) {
        // Tt4 mode — user specifies turbine inlet temperature
        // Solve for FAR required to reach Tt4
        // FAR = Cp × (Tt4 - Tt_in) / (LHV × eff_b - Cp × Tt4)
        FAR    = (Cp * (Tt4 - Tt_in)) / (LHV * eff_b - Cp * Tt4);
        Tt_out = Tt4;
    }
    else {
        // FAR mode — user specifies fuel-air ratio
        // Solve for Tt_exit from energy balance
        // Tt_exit = Tt_in + (FAR × LHV × eff_b) / ((1 + FAR) × Cp)
        FAR    = FAR_in;
        Tt_out = Tt_in + (FAR * LHV * eff_b) / ((1.0 + FAR) * Cp);
    }

    // Apply fractional pressure loss
    // Pt_exit = Pt_inlet × (1 - dPqP)
    flowOut.Pt = Pt_in * (1.0 - dPqP);

    // Update exit conditions
    flowOut.Tt    = Tt_out;
    flowOut.W     = flowIn.W * (1.0 + FAR);  // mass flow increases with fuel
    flowOut.MN    = flowIn.MN;
    flowOut.FAR   = FAR;
    flowOut.gamma = flowIn.gamma;
    flowOut.Cp    = flowIn.Cp;
}