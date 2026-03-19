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
 * 3. Compute initial Tt_exit using current PR_t_ and eff_t_ (fallback)
 * 4. Compute DhT from initial Tt_exit — map lookup x-axis
 * 5. Get PR_t and eff_t — from map if loaded, else fixed fallback values
 * 6. Recompute Tt_exit with final PR_t and eff_t
 * 7. Compute exit total pressure — Pt_exit = Pt_inlet / PR_t
 * 8. Compute dHt — specific work extracted [J/kg]
 * 9. Pass W, MN, FAR through unchanged
 * 10. Write real gas Cp and gamma to flowOut via Thermo at exit conditions
 *
 * PR_T AND EFF_T SOURCE:
 *   Map loaded    → TurbineMap::lookup(DhT, 100.0) [Phase 3: Nc%=100 always]
 *   No map loaded → fixed PR_t_ and eff_t_ from constructor / setPR()
 *
 * DhT COMPUTATION:
 *   DhT = Cp * (Tt_in - Tt_exit) / Tt_in  [-]
 *   Tt_exit is first estimated using fallback PR_t_ and eff_t_.
 *   At design point (Nc%=100) the map returns design values exactly.
 */

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

    // Step 3 — Initial Tt_exit using fallback PR_t_ and eff_t_
    // Used to compute DhT for map lookup
    const double exponent     = (gamma - 1.0) / gamma;
    const double Tt_ideal_0   = Tt_in * std::pow(1.0 / PR_t_, exponent);
    const double Tt_exit_0    = Tt_in - eff_t_ * (Tt_in - Tt_ideal_0);

    // Step 4 — Compute DhT for map lookup
    // DhT = Cp * (Tt_in - Tt_exit) / Tt_in  [-]
    const double DhT = Cp * (Tt_in - Tt_exit_0) / Tt_in;

    // Step 5 — Get PR_t and eff_t from map or fallback
    // Phase 3: Nc% = 100.0 — design speed line always
    // Phase 4: replace 100.0 with computed corrected shaft speed
    double PR_t_use  = PR_t_;
    double eff_t_use = eff_t_;

    if (map_.has_value())
    {
        const TurbineMapResult res = map_->lookup(DhT, 100.0);
        PR_t_use  = res.PR;
        eff_t_use = res.eff;
    }

    // Update public PR_t to reflect current operating value
    PR_t = PR_t_use;

    // Step 6 — Recompute Tt_exit with final PR_t and eff_t
    const double Tt_ideal = Tt_in * std::pow(1.0 / PR_t_use, exponent);
    flowOut.Tt = Tt_in - eff_t_use * (Tt_in - Tt_ideal);

    // Step 7 — Exit total pressure
    flowOut.Pt = Pt_in / PR_t_use;

    // Step 8 — Specific work extracted [J/kg]
    dHt = Cp * (Tt_in - flowOut.Tt);

    // Step 9 — Pass remaining properties through unchanged
    flowOut.W   = flowIn.W;
    flowOut.MN  = flowIn.MN;
    flowOut.FAR = flowIn.FAR;

    // Step 10 — Real gas Cp and gamma at exit conditions
    flowOut.Cp    = Thermo::getCp   (flowOut.Tt, flowOut.FAR);
    flowOut.gamma = Thermo::getGamma(flowOut.Tt, flowOut.FAR);
}