#include "Nozzle.h"
#include "Thermo.h"
#include <cmath>
#include <iostream>

/*
 * Nozzle.cpp
 * ----------
 * Implements the Nozzle element thermodynamics.
 *
 * COMPUTE SEQUENCE:
 * 1. Read inlet conditions from flowIn (Pt, Tt, W, FAR)
 * 2. Evaluate real gas gamma, Cp, R at inlet conditions via Thermo
 * 3. Get ambient static pressure from ISA
 * 4. Compute NPR and critical NPR — determine choked/unchoked
 * 5. Get Cfg — from map if loaded, else fixed fallback value
 * 6. Compute ideal jet velocity (isentropic expansion)
 * 7. Apply Cfg to get actual jet velocity
 * 8. Determine throat area:
 *    Scheduled mode (setAth called): Ath = ath_scheduled_
 *    Computed mode  (default):       Ath from flow function
 * 9. Compute gross thrust Fg
 * 10. Write results to flowOut
 *
 * VARIABLE AREA NOZZLE:
 *   Scheduled mode: Ath is fixed hardware — cycle must balance with it.
 *   Computed mode:  Ath sizes itself to current flow (design point use).
 *
 * THROAT AREA — COMPUTED MODE:
 *   At choked conditions: MN_throat = 1.0
 *   FF  = sqrt(γ/R) × MN × (1 + (γ-1)/2 × MN²)^(-(γ+1)/(2(γ-1)))
 *   Ath = W × sqrt(Tt) / (Pt × FF)
 */

// =========================================================================
// Constructor
// =========================================================================

Nozzle::Nozzle(double altitude_m, double Cfg) noexcept
    : altitude_m_(altitude_m)
    , Cfg_       (Cfg)
{}

// =========================================================================
// loadMap
// =========================================================================

void Nozzle::loadMap(const std::string& filepath)
{
    map_.emplace(filepath);

    if (!map_->isLoaded())
    {
        std::cerr << "[Nozzle] WARNING: map failed to load from '"
                  << filepath << "' — using fixed Cfg=" << Cfg_ << "\n";
        map_.reset();
    }
}

// =========================================================================
// compute
// =========================================================================

void Nozzle::compute() noexcept
{
    // Step 1 — Read inlet conditions
    const double Pt_in = flowIn.Pt;
    const double Tt_in = flowIn.Tt;
    const double W     = flowIn.W;

    // Step 2 — Real gas properties at inlet conditions
    const double gamma = Thermo::getGamma(Tt_in, flowIn.FAR);
    const double Cp    = Thermo::getCp   (Tt_in, flowIn.FAR);
    const double R     = Cp * (gamma - 1.0) / gamma;

    // Step 3 — Ambient static pressure from ISA
    const double Ps_amb = ISA::getStaticPressure(altitude_m_);

    // Step 4 — NPR and critical NPR
    NPR = Pt_in / Ps_amb;
    const double NPR_crit = std::pow((gamma + 1.0) / 2.0,
                                      gamma / (gamma - 1.0));
    const double Ps_exit  = (NPR < NPR_crit) ? Ps_amb : Pt_in / NPR_crit;

    // Step 5 — Get Cfg from map or fallback
    const double Cfg = map_.has_value() ? map_->lookup(NPR) : Cfg_;

    // Step 6 — Ideal jet velocity (isentropic expansion)
    const double pressure_ratio = Ps_exit / Pt_in;
    const double Vjet_ideal = std::sqrt(
        2.0 * Cp * Tt_in
            * (1.0 - std::pow(pressure_ratio, (gamma - 1.0) / gamma))
    );

    // Step 7 — Actual jet velocity
    Vjet = Cfg * Vjet_ideal;

    // Step 8 — Throat area
    // Scheduled mode: use fixed Ath set by operator via setAth()
    // Computed mode:  derive from continuity + flow function
    if (use_scheduled_ath_)
    {
        // Scheduled mode — Ath is a fixed geometric constraint
        Ath = ath_scheduled_;
    }
    else
    {
        // Computed mode — Ath sizes itself to current flow
        const double MN_throat = (NPR >= NPR_crit)
                               ? 1.0
                               : Vjet / std::sqrt(gamma * R * Tt_in);
        const double FF_exp = -(gamma + 1.0) / (2.0 * (gamma - 1.0));
        const double FF     = std::sqrt(gamma / R) * MN_throat
                            * std::pow(1.0 + (gamma - 1.0) / 2.0
                                           * MN_throat * MN_throat, FF_exp);
        Ath = (W * std::sqrt(Tt_in)) / (Pt_in * FF);
    }

    // Step 9 — Gross thrust
    Fg = W * Vjet + (Ps_exit - Ps_amb) * Ath;

    // Step 10 — Update flowOut
    flowOut.Pt    = Pt_in;
    flowOut.Tt    = Tt_in;
    flowOut.W     = W;
    flowOut.MN    = (NPR >= NPR_crit) ? 1.0
                  : Vjet / std::sqrt(gamma * R * Tt_in);
    flowOut.FAR   = flowIn.FAR;
    flowOut.Cp    = Thermo::getCp   (flowOut.Tt, flowOut.FAR);
    flowOut.gamma = Thermo::getGamma(flowOut.Tt, flowOut.FAR);
}