#include "Compressor.h"
#include "Thermo.h"
#include <cmath>
#include <iostream>

/*
 * Compressor.cpp
 * --------------
 * Implements the Compressor element thermodynamics.
 *
 * COMPUTE SEQUENCE:
 * 1. Read inlet conditions from flowIn (Pt, Tt, W, FAR)
 * 2. Compute corrected mass flow Wc
 * 3. Get PR and eff — from map if loaded, else fixed fallback values
 * 4. Evaluate real gas gamma and Cp at inlet conditions via Thermo
 * 5. Compute exit total pressure     — Pt_exit = Pt_inlet × PR
 * 6. Compute ideal exit temperature  — Tt_ideal = Tt_inlet × PR^((γ-1)/γ)
 * 7. Compute actual exit temperature — Tt_exit = Tt_inlet + (Tt_ideal - Tt_inlet) / eff
 * 8. Compute dHt                     — dHt = Cp × (Tt_exit - Tt_inlet) [J/kg]
 * 9. Pass W, MN, FAR through unchanged
 * 10. Write real gas Cp and gamma to flowOut via Thermo at exit conditions
 *
 * PR AND EFF SOURCE:
 *   Map loaded    → CompressorMap::lookup(Wc, 100.0) [Phase 3: Nc%=100 always]
 *   No map loaded → fixed PR_ and eff_ from constructor
 *
 * CORRECTED FLOW:
 *   Wc = W * sqrt(Tt_in / 288.15) / (Pt_in / 101325.0)
 *   Reference: ISA sea level T=288.15 K, P=101325 Pa
 */

// Reference conditions for corrected flow calculation
static constexpr double T_REF = 288.15;    // ISA sea level temperature [K]
static constexpr double P_REF = 101325.0;  // ISA sea level pressure [Pa]

// =========================================================================
// Constructor
// =========================================================================

Compressor::Compressor(double PR, double eff) noexcept
    : PR_ (PR)
    , eff_(eff)
{}

// =========================================================================
// loadMap
//
// Loads a CompressorMap from file. Mirrors NPSS S_map subelement pattern.
// On success: compute() uses map lookup for PR and eff.
// On failure: falls back to fixed PR_ and eff_ with a warning.
// =========================================================================

void Compressor::loadMap(const std::string& filepath)
{
    map_.emplace(filepath);

    if (!map_->isLoaded())
    {
        std::cerr << "[Compressor] WARNING: map failed to load from '"
                  << filepath << "' — using fixed PR=" << PR_
                  << " eff=" << eff_ << "\n";
        map_.reset();
    }
}

// =========================================================================
// compute
//
// Performs compressor thermodynamics. Called once per solver iteration.
// =========================================================================

void Compressor::compute() noexcept
{
    // Step 1 — Read inlet conditions
    const double Pt_in = flowIn.Pt;
    const double Tt_in = flowIn.Tt;

    // Step 2 — Compute corrected mass flow
    // Wc = W * sqrt(Tt_in / T_ref) / (Pt_in / P_ref)
    const double Wc = flowIn.W
                    * std::sqrt(Tt_in / T_REF)
                    / (Pt_in / P_REF);

    // Step 3 — Get PR and eff from map or fallback
    // Phase 3: Nc% = 100.0 — design speed line always
    // Phase 4: replace 100.0 with computed corrected shaft speed
    double PR  = PR_;
    double eff = eff_;

    if (map_.has_value())
    {
        const CompressorMapResult res = map_->lookup(Wc, 100.0);
        PR  = res.PR;
        eff = res.eff;
    }

    // Step 4 — Real gas properties at inlet conditions
    const double gamma = Thermo::getGamma(Tt_in, flowIn.FAR);
    const double Cp    = Thermo::getCp   (Tt_in, flowIn.FAR);

    // Step 5 — Exit total pressure
    flowOut.Pt = Pt_in * PR;

    // Step 6 — Ideal exit temperature (isentropic compression)
    const double exponent = (gamma - 1.0) / gamma;
    const double Tt_ideal = Tt_in * std::pow(PR, exponent);

    // Step 7 — Actual exit temperature (accounting for efficiency losses)
    flowOut.Tt = Tt_in + (Tt_ideal - Tt_in) / eff;

    // Step 8 — Total enthalpy rise [J/kg]
    dHt = Cp * (flowOut.Tt - Tt_in);

    // Step 9 — Pass remaining properties through unchanged
    flowOut.W   = flowIn.W;
    flowOut.MN  = flowIn.MN;
    flowOut.FAR = flowIn.FAR;

    // Step 10 — Real gas Cp and gamma at exit conditions
    flowOut.Cp    = Thermo::getCp   (flowOut.Tt, flowOut.FAR);
    flowOut.gamma = Thermo::getGamma(flowOut.Tt, flowOut.FAR);
}