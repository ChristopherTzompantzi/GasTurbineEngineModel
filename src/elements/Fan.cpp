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
 * 3. Get PR and eff — from map if loaded, else fixed fallback values
 * 4. Evaluate real gas gamma and Cp at inlet conditions via Thermo
 * 5. Compute exit total pressure     — Pt_exit = Pt_inlet × PR_f
 * 6. Compute ideal exit temperature  — Tt_ideal = Tt_in × PR_f^((γ-1)/γ)
 * 7. Compute actual exit temperature — Tt_exit = Tt_in + (Tt_ideal - Tt_in) / eff_f
 * 8. Compute specific work dHt       — dHt = Cp × (Tt_exit - Tt_in) [J/kg]
 * 9. Pass W, MN, FAR through unchanged
 * 10. Write real gas Cp and gamma to flowOut via Thermo at exit conditions
 *
 * PR AND EFF SOURCE:
 *   Map loaded    → CompressorMap::lookup(Wc, 100.0) [Phase 3: Nc%=100 always]
 *   No map loaded → fixed PR_f_ and eff_f_ from constructor
 *
 * CORRECTED FLOW:
 *   Wc = W * sqrt(Tt_in / 288.15) / (Pt_in / 101325.0)
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

    // Step 3 — Get PR and eff from map or fallback
    // Phase 3: Nc% = 100.0 — design speed line always
    // Phase 4: replace 100.0 with computed LP shaft corrected speed
    double PR_f  = PR_f_;
    double eff_f = eff_f_;

    if (map_.has_value())
    {
        const CompressorMapResult res = map_->lookup(Wc, 100.0);
        PR_f  = res.PR;
        eff_f = res.eff;
    }

    // Step 4 — Real gas properties at inlet conditions
    const double gamma = Thermo::getGamma(Tt_in, flowIn.FAR);
    const double Cp    = Thermo::getCp   (Tt_in, flowIn.FAR);

    // Step 5 — Exit total pressure
    flowOut.Pt = Pt_in * PR_f;

    // Step 6 — Ideal exit temperature (isentropic compression)
    const double exponent = (gamma - 1.0) / gamma;
    const double Tt_ideal = Tt_in * std::pow(PR_f, exponent);

    // Step 7 — Actual exit temperature
    flowOut.Tt = Tt_in + (Tt_ideal - Tt_in) / eff_f;

    // Step 8 — Specific work input
    dHt = Cp * (flowOut.Tt - Tt_in);

    // Step 9 — Pass remaining properties through unchanged
    flowOut.W   = flowIn.W;
    flowOut.MN  = flowIn.MN;
    flowOut.FAR = flowIn.FAR;

    // Step 10 — Real gas Cp and gamma at exit conditions
    flowOut.Cp    = Thermo::getCp   (flowOut.Tt, flowOut.FAR);
    flowOut.gamma = Thermo::getGamma(flowOut.Tt, flowOut.FAR);
}