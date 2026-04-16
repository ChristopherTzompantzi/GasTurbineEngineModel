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
 * 3. Compute corrected speed Nc from shaft N_rpm (if shaft connected)
 * 4. Get PR and eff — from map lookup at (Wc, Nc) if map loaded,
 *    else fixed fallback values
 * 5. Evaluate real gas gamma and Cp at inlet conditions via Thermo
 * 6. Compute exit total pressure     — Pt_exit = Pt_inlet × PR
 * 7. Compute ideal exit temperature  — Tt_ideal = Tt_inlet × PR^((γ-1)/γ)
 * 8. Compute actual exit temperature — Tt_exit = Tt_inlet + (Tt_ideal - Tt_inlet) / eff
 * 9. Compute dHt                     — dHt = Cp × (Tt_exit - Tt_inlet) [J/kg]
 * 10. Pass W, MN, FAR through unchanged
 * 11. Write real gas Cp and gamma to flowOut via Thermo at exit conditions
 *
 * PR AND EFF SOURCE:
 *   Shaft + map → CompressorMap::lookup(Wc, Nc)  [Nc from shaft N_rpm]
 *   Map only    → CompressorMap::lookup(Wc, Nc_design) [design Nc from header]
 *   No map      → fixed PR_ and eff_ from constructor
 *
 * CORRECTED FLOW AND SPEED:
 *   Wc = W * sqrt(Tt_in / T_REF) / (Pt_in / P_REF)
 *   Nc = N_rpm / sqrt(Tt_in / T_REF)
 *   Reference: ISA sea level T=288.15 K, P=101325 Pa
 */

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

    // Step 3 — Compute corrected speed from shaft
    // Nc = N_rpm / sqrt(Tt_in / T_ref)   [RPM corrected]
    // No shaft → use design Nc from map header (design point operation)
    double Nc = 0.0;
    if (shaft_ != nullptr)
        Nc = shaft_->getSpeed() / std::sqrt(Tt_in / T_REF);
    else if (map_.has_value())
        Nc = map_->designPoint().Nc;   // design corrected speed from header

    // Step 4 — Get PR and eff from map or fallback
    double PR  = PR_;
    double eff = eff_;

    if (map_.has_value())
    {
        const CompressorMapResult res = map_->lookup(Wc, Nc);
        PR  = res.PR;
        eff = res.eff;
        surge_margin = map_->surgeMargin(Wc, PR);
    }

    // Store corrected flow and speed for external inspection
    this->Wc = Wc;
    this->Nc = Nc;

    // Step 5 — Real gas properties at inlet conditions
    const double gamma = Thermo::getGamma(Tt_in, flowIn.FAR);
    const double Cp    = Thermo::getCp   (Tt_in, flowIn.FAR);

    // Step 6 — Exit total pressure
    flowOut.Pt = Pt_in * PR;

    // Step 7 — Ideal exit temperature (isentropic compression)
    const double exponent = (gamma - 1.0) / gamma;
    const double Tt_ideal = Tt_in * std::pow(PR, exponent);

    // Step 8 — Actual exit temperature
    flowOut.Tt = Tt_in + (Tt_ideal - Tt_in) / eff;

    // Step 9 — Total enthalpy rise [J/kg]
    dHt = Cp * (flowOut.Tt - Tt_in);

    // Step 10 — Pass remaining properties through unchanged
    flowOut.W   = flowIn.W;
    flowOut.MN  = flowIn.MN;
    flowOut.FAR = flowIn.FAR;

    // Step 11 — Real gas Cp and gamma at exit conditions
    flowOut.Cp    = Thermo::getCp   (flowOut.Tt, flowOut.FAR);
    flowOut.gamma = Thermo::getGamma(flowOut.Tt, flowOut.FAR);
}