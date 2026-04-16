#include "Fan.h"
#include "Thermo.h"
#include <cmath>
#include <iostream>

/*
 * Fan.cpp
 * -------
 * Implements Fan element thermodynamics.
 * Equations identical to Compressor — see Compressor.cpp for derivation.
 *
 * COMPUTE SEQUENCE:
 * 1. Read inlet total conditions from flowIn
 * 2. Compute corrected mass flow Wc
 * 3. Compute corrected speed Nc from shaft N_rpm (if shaft connected)
 * 4. Get PR and eff — from map lookup at (Wc, Nc) if map loaded,
 *    else fixed fallback values
 * 5. Evaluate real gas gamma and Cp at inlet conditions via Thermo
 * 6. Compute exit total pressure     — Pt_exit = Pt_inlet × PR_f
 * 7. Compute ideal exit temperature  — Tt_ideal = Tt_in × PR_f^((γ-1)/γ)
 * 8. Compute actual exit temperature — Tt_exit = Tt_in + (Tt_ideal - Tt_in) / eff_f
 * 9. Compute specific work dHt       — dHt = Cp × (Tt_exit - Tt_in) [J/kg]
 * 10. Pass W, MN, FAR through unchanged
 * 11. Write real gas Cp and gamma to flowOut via Thermo at exit conditions
 *
 * PR AND EFF SOURCE:
 *   Shaft + map → CompressorMap::lookup(Wc, Nc)       [Nc from shaft N_rpm]
 *   Map only    → CompressorMap::lookup(Wc, Nc_design) [design Nc from header]
 *   No map      → fixed PR_f_ and eff_f_ from constructor
 *
 * CORRECTED FLOW AND SPEED:
 *   Wc = W * sqrt(Tt_in / T_REF) / (Pt_in / P_REF)
 *   Nc = N_rpm / sqrt(Tt_in / T_REF)
 */

static constexpr double T_REF = 288.15;    // ISA sea level temperature [K]
static constexpr double P_REF = 101325.0;  // ISA sea level pressure [Pa]

// =========================================================================
// Constructor
// =========================================================================

Fan::Fan(double PR_f, double eff_f) noexcept
    : PR_f_ (PR_f)
    , eff_f_(eff_f)
{}

// =========================================================================
// loadMap
//
// Loads a CompressorMap from file. Mirrors NPSS S_map subelement pattern.
// On success: compute() uses map lookup for PR and eff.
// On failure: falls back to fixed PR_f_ and eff_f_ with a warning.
// =========================================================================

void Fan::loadMap(const std::string& filepath)
{
    map_.emplace(filepath);

    if (!map_->isLoaded())
    {
        std::cerr << "[Fan] WARNING: map failed to load from '"
                  << filepath << "' — using fixed PR_f=" << PR_f_
                  << " eff_f=" << eff_f_ << "\n";
        map_.reset();
    }
}

// =========================================================================
// compute
//
// Performs fan thermodynamics. Called once per solver iteration.
// Equations identical to Compressor — see Compressor.cpp for derivation.
// =========================================================================

void Fan::compute() noexcept
{
    // Step 1 — Read inlet conditions
    const double Pt_in = flowIn.Pt;
    const double Tt_in = flowIn.Tt;

    // Step 2 — Compute corrected mass flow
    const double Wc = flowIn.W
                    * std::sqrt(Tt_in / T_REF)
                    / (Pt_in / P_REF);

    // Step 3 — Compute corrected speed from shaft
    // Nc = N_rpm / sqrt(Tt_in / T_ref)   [RPM corrected]
    // No shaft → use design Nc from map header (design point operation)
    double Nc = 0.0;
    if (shaft_ != nullptr)
        Nc = shaft_->getSpeed() / std::sqrt(Tt_in / T_REF);
    else if (map_.has_value())
        Nc = map_->designPoint().Nc;

    // Step 4 — Get PR and eff from map or fallback
    double PR_f  = PR_f_;
    double eff_f = eff_f_;

    if (map_.has_value())
    {
        const CompressorMapResult res = map_->lookup(Wc, Nc);
        PR_f  = res.PR;
        eff_f = res.eff;
        surge_margin = map_->surgeMargin(Wc, PR_f);
    }

    // Store corrected flow and speed for external inspection
    this->Wc = Wc;
    this->Nc = Nc;
    this->eff_is = eff_f;

    // Step 5 — Real gas properties at inlet conditions
    const double gamma = Thermo::getGamma(Tt_in, flowIn.FAR);
    const double Cp    = Thermo::getCp   (Tt_in, flowIn.FAR);

    // Compute polytropic efficiency — diagnostic output only
    if (PR_f > 1.0 + 1.0e-6 && eff_f > 1.0e-6)
    {
        const double exponent = (gamma - 1.0) / gamma;
        const double pr_exp   = std::pow(PR_f, exponent);
        eta_poly = std::log(PR_f) * exponent
                 / std::log(1.0 + (pr_exp - 1.0) / eff_f);
    }

    // Step 6 — Exit total pressure
    flowOut.Pt = Pt_in * PR_f;

    // Step 7 — Ideal exit temperature (isentropic compression)
    const double exponent = (gamma - 1.0) / gamma;
    const double Tt_ideal = Tt_in * std::pow(PR_f, exponent);

    // Step 8 — Actual exit temperature
    flowOut.Tt = Tt_in + (Tt_ideal - Tt_in) / eff_f;

    // Step 9 — Specific work input
    dHt = Cp * (flowOut.Tt - Tt_in);

    // Step 10 — Pass remaining properties through unchanged
    flowOut.W   = flowIn.W;
    flowOut.MN  = flowIn.MN;
    flowOut.FAR = flowIn.FAR;

    // Step 11 — Real gas Cp and gamma at exit conditions
    flowOut.Cp    = Thermo::getCp   (flowOut.Tt, flowOut.FAR);
    flowOut.gamma = Thermo::getGamma(flowOut.Tt, flowOut.FAR);
}