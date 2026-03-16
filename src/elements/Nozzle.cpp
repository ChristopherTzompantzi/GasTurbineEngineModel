#include "Nozzle.h"
#include "Thermo.h"
#include <cmath>

/*
 * Nozzle.cpp
 * ----------
 * Implements the Nozzle element thermodynamics.
 *
 * COMPUTE SEQUENCE:
 * 1. Read inlet conditions from flowIn (Pt, Tt, W)
 * 2. Evaluate real gas gamma and Cp at inlet conditions via Thermo
 * 3. Get ambient conditions from ISA model
 * 4. Compute NPR and determine choked/unchoked condition
 * 5. Compute ideal jet velocity (isentropic expansion)
 * 6. Apply Cfg to get actual jet velocity
 * 7. Calculate throat area from flow function
 * 8. Compute gross thrust Fg
 * 9. Write real gas Cp and gamma to flowOut via Thermo
 */

Nozzle::Nozzle(double altitude_m, double Cfg) noexcept
    : altitude_m(altitude_m)
    , Cfg(Cfg)
{}

void Nozzle::compute() noexcept
{
    // Step 1 — Read inlet conditions
    const double Pt_in = flowIn.Pt;
    const double Tt_in = flowIn.Tt;
    const double W     = flowIn.W;

    // Step 2 — Real gas properties at inlet conditions via Thermo
    const double gamma = Thermo::getGamma(Tt_in, flowIn.FAR);
    const double Cp    = Thermo::getCp   (Tt_in, flowIn.FAR);
    // R derived from real gas Cp and gamma — consistent with Thermo mixture
    const double R     = Cp * (gamma - 1.0) / gamma;   // [J/kg·K]

    // Step 3 — Get ambient static pressure from ISA
    double Ps_amb = ISA::getStaticPressure(altitude_m);

    // Step 4 — Nozzle pressure ratio and critical NPR
    // NPR_crit = ((γ+1)/2)^(γ/(γ-1))
    NPR = Pt_in / Ps_amb;
    double NPR_crit = std::pow((gamma + 1.0) / 2.0, gamma / (gamma - 1.0));

    // Step 4b — Determine exit static pressure
    // Unchoked: Ps_exit = Ps_ambient
    // Choked:   Ps_exit = Pt_inlet / NPR_crit
    double Ps_exit = (NPR < NPR_crit) ? Ps_amb : Pt_in / NPR_crit;

    // Step 5 — Ideal jet velocity (isentropic expansion)
    // Vjet_ideal = sqrt(2 × Cp × Tt_in × (1 - (Ps_exit/Pt_in)^((γ-1)/γ)))
    double pressure_ratio = Ps_exit / Pt_in;
    double Vjet_ideal = std::sqrt(
        2.0 * Cp * Tt_in * (1.0 - std::pow(pressure_ratio, (gamma - 1.0) / gamma))
    );

    // Step 6 — Actual jet velocity with nozzle efficiency
    Vjet = Cfg * Vjet_ideal;

    // Step 7 — Calculate throat area from flow function
    // At choked conditions MN = 1.0
    // FF  = sqrt(γ/R) × MN × (1 + (γ-1)/2 × MN²)^(-(γ+1)/(2×(γ-1)))
    // Ath = W × sqrt(Tt_in) / (Pt_in × FF)
    double MN_throat = (NPR >= NPR_crit) ? 1.0 : Vjet / std::sqrt(gamma * R * Tt_in);
    double FF_exp    = -(gamma + 1.0) / (2.0 * (gamma - 1.0));
    double FF        = std::sqrt(gamma / R) * MN_throat *
                       std::pow(1.0 + (gamma - 1.0) / 2.0 * MN_throat * MN_throat, FF_exp);
    Ath = (W * std::sqrt(Tt_in)) / (Pt_in * FF);

    // Step 8 — Compute gross thrust
    // Fg = W × Vjet + (Ps_exit - Ps_amb) × Ath
    Fg = W * Vjet + (Ps_exit - Ps_amb) * Ath;

    // Step 9 — Update flowOut
    // Pt is the total pressure — conserved through an ideal nozzle (losses captured by Cfg)
    flowOut.Pt  = Pt_in;
    flowOut.Tt  = Tt_in;     // total temperature conserved (adiabatic)
    flowOut.W   = W;
    flowOut.MN  = MN_throat;
    flowOut.FAR = flowIn.FAR;

    // Real gas Cp and gamma at exit — same as inlet (Tt and FAR unchanged)
    flowOut.Cp    = Thermo::getCp   (flowOut.Tt, flowOut.FAR);
    flowOut.gamma = Thermo::getGamma(flowOut.Tt, flowOut.FAR);
}