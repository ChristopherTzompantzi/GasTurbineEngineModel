#include "Nozzle.h"
#include <cmath>

/*
 * Nozzle.cpp
 * ----------
 * Implements the Nozzle element thermodynamics.
 *
 * COMPUTE SEQUENCE:
 * 1. Get ambient conditions from ISA model
 * 2. Compute NPR and determine choked/unchoked condition
 * 3. Compute ideal jet velocity (isentropic expansion)
 * 4. Apply Cfg to get actual jet velocity
 * 5. Calculate throat area from flow function
 * 6. Compute gross thrust Fg
 */

Nozzle::Nozzle(double altitude_m, double Cfg) noexcept
    : altitude_m(altitude_m)
    , Cfg(Cfg)
{}

void Nozzle::compute() noexcept
{
    // Read inlet conditions from flowIn
    double Pt_in  = flowIn.Pt;
    double Tt_in  = flowIn.Tt;
    double W      = flowIn.W;
    double gamma  = flowIn.gamma;
    double Cp     = flowIn.Cp;
    double R      = Cp * (gamma - 1.0) / gamma;   // Gas constant [J/kg·K]

    // Get ambient static pressure from ISA
    double Ps_amb = ISA::getStaticPressure(altitude_m);

    // Nozzle pressure ratio and critical NPR
    // NPR_crit = ((γ+1)/2)^(γ/(γ-1))
    NPR = Pt_in / Ps_amb;
    double NPR_crit = std::pow((gamma + 1.0) / 2.0, gamma / (gamma - 1.0));

    // Determine exit static pressure
    // Unchoked: Ps_exit = Ps_ambient
    // Choked:   Ps_exit = Pt_inlet / NPR_crit
    double Ps_exit = (NPR < NPR_crit) ? Ps_amb : Pt_in / NPR_crit;

    // Ideal jet velocity (isentropic expansion)
    // Vjet_ideal = sqrt(2 × Cp × Tt_in × (1 - (Ps_exit/Pt_in)^((γ-1)/γ)))
    double pressure_ratio = Ps_exit / Pt_in;
    double Vjet_ideal = std::sqrt(
        2.0 * Cp * Tt_in * (1.0 - std::pow(pressure_ratio, (gamma - 1.0) / gamma))
    );

    // Actual jet velocity with nozzle efficiency
    Vjet = Cfg * Vjet_ideal;

    // Calculate throat area from flow function
    // At choked conditions MN = 1.0
    // FF  = sqrt(γ/R) × MN × (1 + (γ-1)/2 × MN²)^(-(γ+1)/(2×(γ-1)))
    // Ath = W × sqrt(Tt_in) / (Pt_in × FF)
    double MN_throat = (NPR >= NPR_crit) ? 1.0 : Vjet / std::sqrt(gamma * R * Tt_in);
    double FF_exp    = -(gamma + 1.0) / (2.0 * (gamma - 1.0));
    double FF        = std::sqrt(gamma / R) * MN_throat *
                       std::pow(1.0 + (gamma - 1.0) / 2.0 * MN_throat * MN_throat, FF_exp);
    Ath = (W * std::sqrt(Tt_in)) / (Pt_in * FF);

    // Gross thrust
    // Fg = W × Vjet + (Ps_exit - Ps_amb) × Ath
    Fg = W * Vjet + (Ps_exit - Ps_amb) * Ath;

    // Update flowOut
    // Pt is the total pressure — conserved through an ideal nozzle (losses captured by Cfg)
    flowOut.Pt    = Pt_in;
    flowOut.Tt    = Tt_in;     // total temperature conserved (adiabatic)
    flowOut.W     = W;
    flowOut.MN    = MN_throat;
    flowOut.FAR   = flowIn.FAR;
    flowOut.gamma = gamma;
    flowOut.Cp    = Cp;
}