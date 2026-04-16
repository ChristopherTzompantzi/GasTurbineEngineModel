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
 * 7. Recompute Tt_exit with final PR_t and eff_t
 * 8. Compute exit total pressure — Pt_exit = Pt_inlet / PR_t
 * 9. Compute dHt — specific work extracted [J/kg]
 * 10. Pass W, MN, FAR through unchanged
 * 11. Write real gas Cp and gamma to flowOut via Thermo at exit conditions
 *
 * PR_T AND EFF_T SOURCE:
 *   Shaft + map → TurbineMap::lookup(DhT, Nc)        [Nc from shaft N_rpm]
 *   Map only    → TurbineMap::lookup(DhT, Nc_design)  [design Nc from header]
 *   No map      → fixed PR_t_ and eff_t_ from constructor / setPR()
 *
 * DhT COMPUTATION:
 *   DhT = Cp * (Tt_in - Tt_exit_0) / Tt_in  [-]
 *   Tt_exit_0 estimated from fallback PR_t_ and eff_t_.
 *   At design speed the map returns design values exactly.
 */

static constexpr double T_REF = 288.15;    // ISA sea level temperature [K]

// =========================================================================
// Constructor
// =========================================================================

Turbine::Turbine(double PR_t, double eff_t) noexcept
    : PR_t_ (PR_t)
    , eff_t_(eff_t)
{}

// =========================================================================
// loadMap
//
// Loads a TurbineMap from file. Mirrors NPSS S_map subelement pattern.
// On success: compute() uses map lookup for PR_t and eff_t.
// On failure: falls back to fixed PR_t_ and eff_t_ with a warning.
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
//
// Performs turbine thermodynamics. Called once per solver iteration.
// =========================================================================

void Turbine::compute() noexcept
{
    // Step 1 — Read inlet conditions
    const double Pt_in = flowIn.Pt;
    const double Tt_in = flowIn.Tt;

    // Step 2 — Real gas properties at inlet conditions
    const double gamma = Thermo::getGamma(Tt_in, flowIn.FAR);
    const double Cp    = Thermo::getCp   (Tt_in, flowIn.FAR);
    const double exponent = (gamma - 1.0) / gamma;

    // Step 3 — Compute corrected speed from shaft
    // Nc = N_rpm / sqrt(Tt_in / T_ref)   [RPM corrected]
    // No shaft → use design Nc from map header (design point operation)
    double Nc = 0.0;
    if (shaft_ != nullptr)
        Nc = shaft_->getSpeed() / std::sqrt(Tt_in / T_REF);
    else if (map_.has_value())
        Nc = map_->designPoint().Nc;

    // Step 4 — Estimate initial Tt_exit using fallback values
    const double Tt_ideal_0 = Tt_in * std::pow(1.0 / PR_t_, exponent);
    const double Tt_exit_0  = Tt_in - eff_t_ * (Tt_in - Tt_ideal_0);

    // Step 5 — Compute DhT for map lookup
    // DhT = Cp * (Tt_in - Tt_exit) / Tt_in  [-]
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

    // Compute polytropic efficiency — diagnostic output only
    // eta_poly = ln(1 - eta_is*(1-(1/PR)^exp)) / (-exp * ln(PR))
    // eta_poly < eta_is for turbine (opposite direction to compressor)
    // Reference: Mattingly Ch.5 — small-stage efficiency derivation
    if (PR_t_use > 1.0 + 1.0e-6 && eff_t_use > 1.0e-6)
    {
        const double exponent  = (gamma - 1.0) / gamma;
        const double pr_exp    = std::pow(1.0 / PR_t_use, exponent);
        const double inner     = 1.0 - eff_t_use * (1.0 - pr_exp);
        if (inner > 1.0e-6)
            eta_poly = std::log(inner) / (-exponent * std::log(PR_t_use));
    }

    // Step 7 — Recompute Tt_exit with final PR_t and eff_t
    const double Tt_ideal = Tt_in * std::pow(1.0 / PR_t_use, exponent);
    flowOut.Tt = Tt_in - eff_t_use * (Tt_in - Tt_ideal);

    // Step 8 — Exit total pressure
    flowOut.Pt = Pt_in / PR_t_use;

    // Step 9 — Specific work extracted [J/kg]
    dHt = Cp * (Tt_in - flowOut.Tt);

    // Step 10 — Pass remaining properties through unchanged
    flowOut.W   = flowIn.W;
    flowOut.MN  = flowIn.MN;
    flowOut.FAR = flowIn.FAR;

    // Step 11 — Real gas Cp and gamma at exit conditions
    flowOut.Cp    = Thermo::getCp   (flowOut.Tt, flowOut.FAR);
    flowOut.gamma = Thermo::getGamma(flowOut.Tt, flowOut.FAR);
}