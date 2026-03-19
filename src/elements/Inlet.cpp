#include "Inlet.h"
#include "Thermo.h"
#include <iostream>

/*
 * Inlet.cpp
 * ---------
 * Implements the Inlet element thermodynamics.
 *
 * COMPUTE SEQUENCE:
 * 1. Get ambient static conditions from ISA model
 * 2. Evaluate real gas gamma at ambient static temperature (FAR=0)
 * 3. Compute freestream total conditions (isentropic)
 * 4. Get pressure recovery eta_r — from map if loaded, else fixed value
 * 5. Apply eta_r to total pressure
 * 6. Pass mass flow through unchanged
 * 7. Write real gas Cp and gamma to flowOut via Thermo
 *
 * RECOVERY FACTOR SOURCE:
 *   Map loaded    → eta_r = map_->lookup(MN_)   [varies with Mach number]
 *   No map loaded → eta_r = recovery_factor_    [fixed constant]
 *
 * NOTE ON MASS FLOW:
 *   W is set externally by the solver before compute() is called.
 *   The inlet passes it through unchanged — mirrors NPSS behaviour.
 *
 * NOTE ON GAMMA:
 *   gamma is evaluated at Ts (static temperature) for isentropic
 *   relations — physically correct since static-to-total compression
 *   occurs close to Ts. flowOut.gamma is re-evaluated at Tt for
 *   downstream elements.
 */

// =========================================================================
// Constructor
// =========================================================================

Inlet::Inlet(double altitude_m, double MN,
             double recovery_factor) noexcept
    : altitude_m_     (altitude_m)
    , MN_             (MN)
    , recovery_factor_(recovery_factor)
{
    // flowIn and flowOut initialised to ISA sea level conditions
    // by FlowStation constructor — safe starting point
    // map_ left as nullopt — loadMap() must be called to enable map-based recovery
}

// =========================================================================
// loadMap
//
// Loads an InletMap from file. Mirrors NPSS S_Recovery subelement pattern.
// On success: compute() will use map lookup for eta_r.
// On failure: falls back to fixed recovery_factor_ with a warning.
// =========================================================================

void Inlet::loadMap(const std::string& filepath)
{
    map_.emplace(filepath);

    if (!map_->isLoaded())
    {
        std::cerr << "[Inlet] WARNING: map failed to load from '"
                  << filepath << "' — using fixed recovery_factor="
                  << recovery_factor_ << "\n";
        map_.reset();
        return;
    }
}

// =========================================================================
// compute
//
// Performs inlet thermodynamics. Called once per solver iteration.
// =========================================================================

void Inlet::compute() noexcept
{
    // Step 1 — Get ambient static conditions from ISA model
    const double Ts = ISA::getStaticTemperature(altitude_m_);
    const double Ps = ISA::getStaticPressure(altitude_m_);

    // Step 2 — Evaluate real gas gamma at ambient static conditions
    // FAR=0 — pure air upstream of combustor
    // Ts used because isentropic relations act on static-to-total
    // compression, which occurs close to Ts not Tt
    const double gamma = Thermo::getGamma(Ts, 0.0);

    // Step 3 — Compute freestream total conditions
    const double Tt = ISA::getTotalTemperature(Ts, MN_, gamma);
    const double Pt = ISA::getTotalPressure(Ps, MN_, gamma);

    // Step 4 — Get pressure recovery eta_r
    // Map loaded → interpolate from InletMap at current Mach number
    // No map    → use fixed recovery_factor_ from constructor
    const double eta_r = map_.has_value()
                       ? map_->lookup(MN_)
                       : recovery_factor_;

    // Step 5 — Apply recovery to total pressure
    // Tt unchanged — inlet is adiabatic
    flowOut.Pt = Pt * eta_r;
    flowOut.Tt = Tt;

    // Step 6 — Pass remaining properties through unchanged
    flowOut.W   = flowIn.W;
    flowOut.MN  = MN_;
    flowOut.FAR = 0.0;   // No fuel added in inlet

    // Step 7 — Write real gas Cp and gamma to flowOut
    // Evaluated at exit total temperature Tt, FAR=0 (pure air)
    flowOut.Cp    = Thermo::getCp   (flowOut.Tt, flowOut.FAR);
    flowOut.gamma = Thermo::getGamma(flowOut.Tt, flowOut.FAR);

    // Update flowIn to reflect freestream total conditions
    // flowIn of Inlet records ambient total conditions for diagnostics
    flowIn.Pt = Pt;
    flowIn.Tt = Tt;
}