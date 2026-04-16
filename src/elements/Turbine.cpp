#include "Turbine.h"
#include "Thermo.h"
#include <cmath>
#include <iostream>

/*
 * Turbine.cpp
 * -----------
 * Implements the Turbine element thermodynamics.
 *
 * COMPUTE SEQUENCE:
 * 1. Read inlet conditions from flowIn (Pt, Tt, FAR)
 * 2. Evaluate real gas gamma and Cp at inlet conditions via Thermo
 * 3. Compute corrected speed Nc from shaft N_rpm (if shaft connected)
 * 4. Estimate initial Tt_exit using fallback PR_t_ and eff_t_
 * 5. Compute DhT from initial Tt_exit — map lookup x-axis
 * 6. Get PR_t and eff_t — from map lookup at (DhT, Nc) if map loaded,
 *    else fixed fallback values
 * 7. Apply tip clearance correction to eff_t:
 *      eff_t_corrected = eff_t_map * (1 - clearance_corr_)
 *    Store corrected value in eff_is for external inspection.
 * 8. Compute polytropic efficiency from corrected eff_t
 * 9. Recompute Tt_exit with final PR_t and corrected eff_t
 * 10. Compute exit total pressure — Pt_exit = Pt_inlet / PR_t
 * 11. Compute dHt — specific work extracted [J/kg]
 * 12. Pass W, MN, FAR through unchanged
 * 13. Write real gas Cp and gamma to flowOut via Thermo at exit conditions
 *
 * TIP CLEARANCE CORRECTION:
 *   eff_t_corrected = eff_t_map * (1 - clearance_corr_)
 *   Mirrors NPSS TipClearance subelement delta_eff correction.
 *   clearance_corr_ = 0.0 → no change (backward compatible default).
 *   Reference: Walsh & Fletcher Ch.5 — clearance effect on turbine efficiency.
 */

static constexpr double T_REF = 288.15;    // ISA sea level temperature [K]

// =========================================================================
// Constructor
// =========================================================================

Turbine::Turbine(double PR_t,
                 double eff_t,
                 double clearance_corr) noexcept
    : PR_t_          (PR_t)
    , eff_t_         (eff_t)
    , clearance_corr_(clearance_corr)
{}

// =========================================================================
// loadMap
// =========================================================================

void Turbine::loadMap(const std::string& filepath)
{
    map_.emplace(filepath);

    if (!map_->isLoaded())
    {
        std::cerr << "[Turbine] WARNING: map failed to load from '"
                  << filepath << "' — using fixed PR_t=" << PR_t_
                  << " eff_t=" << eff_t_ << "\n";
        map_.reset();
    }
}

// =========================================================================
// compute
// =========================================================================

void Turbine::compute() noexcept
{
    // Step 1 — Read inlet conditions
    const double Pt_in = flowIn.Pt;
    const double Tt_in = flowIn.Tt;

    // Step 2 — Real gas properties at inlet conditions
    const double gamma    = Thermo::getGamma(Tt_in, flowIn.FAR);
    const double Cp       = Thermo::getCp   (Tt_in, flowIn.FAR);
    const double exponent = (gamma - 1.0) / gamma;

    // Step 3 — Compute corrected speed from shaft
    double Nc = 0.0;
    if (shaft_ != nullptr)
        Nc = shaft_->getSpeed() / std::sqrt(Tt_in / T_REF);
    else if (map_.has_value())
        Nc = map_->designPoint().Nc;

    // Step 4 — Estimate initial Tt_exit using fallback values
    const double Tt_ideal_0 = Tt_in * std::pow(1.0 / PR_t_, exponent);
    const double Tt_exit_0  = Tt_in - eff_t_ * (Tt_in - Tt_ideal_0);

    // Step 5 — Compute DhT for map lookup
    const double DhT = Cp * (Tt_in - Tt_exit_0) / Tt_in;

    // Step 6 — Get PR_t and eff_t from map or fallback
    double PR_t_use  = PR_t_;
    double eff_t_use = eff_t_;

    if (map_.has_value())
    {
        const TurbineMapResult res = map_->lookup(DhT, Nc);
        PR_t_use  = res.PR;
        eff_t_use = res.eff;
    }

    // Update public PR_t to reflect current operating value
    PR_t = PR_t_use;

    // Step 7 — Apply tip clearance correction
    // eff_t_corrected = eff_t_map * (1 - clearance_corr_)
    // clearance_corr_ = 0.0 → no change (default, backward compatible)
    // Mirrors NPSS TipClearance subelement delta_eff correction.
    // Reference: Walsh & Fletcher Ch.5
    eff_t_use *= (1.0 - clearance_corr_);

    // Store corrected isentropic efficiency for external inspection
    eff_is = eff_t_use;

    // Step 8 — Compute polytropic efficiency from corrected eff_t
    if (PR_t_use > 1.0 + 1.0e-6 && eff_t_use > 1.0e-6)
    {
        const double pr_exp = std::pow(1.0 / PR_t_use, exponent);
        const double inner  = 1.0 - eff_t_use * (1.0 - pr_exp);
        if (inner > 1.0e-6)
            eta_poly = std::log(inner) / (-exponent * std::log(PR_t_use));
    }

    // Step 9 — Recompute Tt_exit with final PR_t and corrected eff_t
    const double Tt_ideal = Tt_in * std::pow(1.0 / PR_t_use, exponent);
    flowOut.Tt = Tt_in - eff_t_use * (Tt_in - Tt_ideal);

    // Step 10 — Exit total pressure
    flowOut.Pt = Pt_in / PR_t_use;

    // Step 11 — Specific work extracted [J/kg]
    dHt = Cp * (Tt_in - flowOut.Tt);

    // Step 12 — Pass remaining properties through unchanged
    flowOut.W   = flowIn.W;
    flowOut.MN  = flowIn.MN;
    flowOut.FAR = flowIn.FAR;

    // Step 13 — Real gas Cp and gamma at exit conditions
    flowOut.Cp    = Thermo::getCp   (flowOut.Tt, flowOut.FAR);
    flowOut.gamma = Thermo::getGamma(flowOut.Tt, flowOut.FAR);
}