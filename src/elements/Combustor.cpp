#include "Combustor.h"
#include "Thermo.h"
#include <cmath>
#include <iostream>

/*
 * Combustor.cpp
 * -------------
 * Implements the Combustor element thermodynamics.
 *
 * COMPUTE SEQUENCE (both modes):
 * 1. Read inlet conditions from flowIn (Pt, Tt, FAR, W)
 * 2. Compute corrected inlet flow Wc_in for map lookup
 * 3. Get eff_b and dPqP — from CombustorMap if loaded, else fallback
 * 4. Compute inlet enthalpy via Thermo::getH(Tt_in, FAR_in)
 * 5. Tt4 mode — iterate FAR until enthalpy balance is satisfied at Tt4
 *    FAR mode — compute H_exit from energy balance, invert via Thermo::getT
 * 6. Apply fractional pressure loss: Pt_exit = Pt_inlet × (1 - dPqP)
 * 7. Update flowOut with exit conditions (Pt, Tt, W, MN, FAR)
 * 8. Write real gas Cp and gamma to flowOut via Thermo at exit conditions
 *
 * MAP LOOKUP:
 *   eff_b = f(FAR)    — from EFF_B_TABLE in map file
 *   dPqP  = f(Wc_in)  — from DPQP_TABLE in map file
 *   Both fall back to constructor values if map not loaded.
 *
 * CORRECTED INLET FLOW:
 *   Wc_in = W * sqrt(Tt_in / T_ref) / (Pt_in / P_ref)
 *   Used as x-axis for dPqP table lookup.
 */

static constexpr double T_REF = 288.15;    // ISA sea level temperature [K]
static constexpr double P_REF = 101325.0;  // ISA sea level pressure [Pa]

// =========================================================================
// Constructor
// =========================================================================

Combustor::Combustor(CombustorMode mode,
                     double Tt4,
                     double FAR_in,
                     double eff_b,
                     double dPqP,
                     double LHV) noexcept
    : mode_  (mode)
    , Tt4_   (Tt4)
    , FAR_in_(FAR_in)
    , eff_b_ (eff_b)
    , dPqP_  (dPqP)
    , LHV_   (LHV)
{}

// =========================================================================
// loadMap
// =========================================================================

void Combustor::loadMap(const std::string& filepath)
{
    map_.emplace(filepath);

    if (!map_->isLoaded())
    {
        std::cerr << "[Combustor] WARNING: map failed to load from '"
                  << filepath << "' — using fixed eff_b=" << eff_b_
                  << " dPqP=" << dPqP_ << "\n";
        map_.reset();
    }
}

// =========================================================================
// compute
// =========================================================================

void Combustor::compute() noexcept
{
    // Step 1 — Read inlet conditions
    const double Pt_in     = flowIn.Pt;
    const double Tt_in     = flowIn.Tt;
    const double FAR_inlet = flowIn.FAR;

    // Step 2 — Compute corrected inlet flow for dPqP map lookup
    const double Wc_in = flowIn.W
                       * std::sqrt(Tt_in / T_REF)
                       / (Pt_in / P_REF);

    // Step 3 — Get eff_b and dPqP from map or fallback
    // FAR for eff_b lookup: use FAR_in_ in FAR mode, or initial guess in Tt4 mode
    // In Tt4 mode the actual FAR is unknown at this point — use inlet FAR as proxy
    // for the map lookup. This is a minor approximation consistent with NPSS S_eff.
    double eff_b = eff_b_;
    double dPqP  = dPqP_;

    if (map_.has_value())
    {
        const double FAR_for_lookup = (mode_ == CombustorMode::FAR)
                                    ? FAR_in_
                                    : FAR_inlet;   // proxy in Tt4 mode
        const CombustorMapResult res = map_->lookup(FAR_for_lookup, Wc_in);
        eff_b = res.eff_b;
        dPqP  = res.dPqP;
    }

    // Store for external inspection
    eff_b_out = eff_b;
    dPqP_out  = dPqP;

    // Step 4 — Inlet specific enthalpy via Thermo
    const double H_in = Thermo::getH(Tt_in, FAR_inlet);

    double FAR_exit = 0.0;
    double Tt_out   = 0.0;

    if (mode_ == CombustorMode::Tt4)
    {
        // Step 5 — Tt4 mode: iterate FAR until enthalpy balance satisfied
        const double Cp_guess = Thermo::getCp(Tt_in, FAR_inlet);
        FAR_exit = (Cp_guess * (Tt4_ - Tt_in))
                 / (LHV_ * eff_b - Cp_guess * Tt4_);

        constexpr double tol      = 1.0e-6;
        constexpr int    max_iter = 50;

        for (int i = 0; i < max_iter; ++i)
        {
            const double H_exit_thermo = Thermo::getH(Tt4_, FAR_exit);
            const double H_exit_energy = H_in
                                       + FAR_exit * LHV_ * eff_b
                                       / (1.0 + FAR_exit);
            const double residual = H_exit_thermo - H_exit_energy;

            constexpr double dFAR = 1.0e-6;
            const double H_exit_thermo_p = Thermo::getH(Tt4_, FAR_exit + dFAR);
            const double H_exit_energy_p = H_in
                                         + (FAR_exit + dFAR) * LHV_ * eff_b
                                         / (1.0 + FAR_exit + dFAR);
            const double dRdFAR = (H_exit_thermo_p - H_exit_energy_p
                                 - residual) / dFAR;

            if (std::abs(dRdFAR) < 1.0e-12) break;
            const double dFAR_step = residual / dRdFAR;
            FAR_exit -= dFAR_step;
            if (std::abs(dFAR_step) < tol) break;
        }

        Tt_out = Tt4_;
    }
    else
    {
        // Step 5 — FAR mode: compute H_exit from energy balance
        FAR_exit = FAR_in_;
        const double H_exit = H_in
                            + FAR_exit * LHV_ * eff_b
                            / (1.0 + FAR_exit);
        Tt_out = Thermo::getT(H_exit, FAR_exit);
    }

    // Step 6 — Apply fractional pressure loss
    flowOut.Pt = Pt_in * (1.0 - dPqP);

    // Step 7 — Update exit conditions
    flowOut.Tt  = Tt_out;
    flowOut.W   = flowIn.W * (1.0 + FAR_exit);
    flowOut.MN  = flowIn.MN;
    flowOut.FAR = FAR_exit;

    // Step 8 — Real gas Cp and gamma at exit conditions
    flowOut.Cp    = Thermo::getCp   (flowOut.Tt, flowOut.FAR);
    flowOut.gamma = Thermo::getGamma(flowOut.Tt, flowOut.FAR);
}