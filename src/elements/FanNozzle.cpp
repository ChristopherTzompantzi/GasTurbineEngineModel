#include "FanNozzle.h"
#include "Thermo.h"
#include <cmath>

/*
 * FanNozzle.cpp
 * -------------
 * Implements FanNozzle element thermodynamics.
 * Logic identical to Nozzle.cpp — see Nozzle.cpp for full derivation.
 *
 * COMPUTE SEQUENCE:
 * 1. Read bypass stream conditions from flowIn (fed from Splitter::bypassOut)
 * 2. Evaluate real gas gamma and Cp at inlet conditions via Thermo
 * 3. Get ambient static pressure from ISA
 * 4. Compute NPR and critical NPR — determine choked/unchoked
 * 5. Compute exit static pressure
 * 6. Compute ideal jet velocity (isentropic expansion)
 * 7. Apply Cfg to get actual jet velocity
 * 8. Compute throat area from flow function
 * 9. Compute cold gross thrust Fg_cold and update flowOut
 *
 * Source: Mattingly, J.D., "Elements of Gas Turbine Propulsion",
 *         McGraw-Hill, 1996. Chapter 5 — nozzle thermodynamics.
 */

FanNozzle::FanNozzle(double altitude_m, double Cfg) noexcept
    : altitude_m(altitude_m)
    , Cfg(Cfg)
{}

void FanNozzle::compute() noexcept
{
    // Step 1 — Read bypass stream inlet conditions
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
    // NPR_crit = ((γ+1)/2)^(γ/(γ-1)) ≈ 1.893 for γ=1.4
    NPR = Pt_in / Ps_amb;
    double NPR_crit = std::pow((gamma + 1.0) / 2.0, gamma / (gamma - 1.0));

    // Step 5 — Exit static pressure
    // Unchoked: Ps_exit = Ps_amb
    // Choked:   Ps_exit = Pt_in / NPR_crit
    double Ps_exit = (NPR < NPR_crit) ? Ps_amb : Pt_in / NPR_crit;

    // Step 6 — Ideal jet velocity — isentropic expansion
    // Vjet_ideal = sqrt(2 × Cp × Tt_in × (1 - (Ps_exit/Pt_in)^((γ-1)/γ)))
    double pressure_ratio = Ps_exit / Pt_in;
    double Vjet_ideal = std::sqrt(
        2.0 * Cp * Tt_in * (1.0 - std::pow(pressure_ratio, (gamma - 1.0) / gamma))
    );

    // Step 7 — Actual jet velocity with nozzle efficiency
    Vjet = Cfg * Vjet_ideal;

    // Step 8 — Throat area from flow function
    // At choked: MN_throat = 1.0
    // FF  = sqrt(γ/R) × MN × (1 + (γ-1)/2 × MN²)^(-(γ+1)/(2×(γ-1)))
    // Ath = W × sqrt(Tt_in) / (Pt_in × FF)
    double MN_throat = (NPR >= NPR_crit) ? 1.0 : Vjet / std::sqrt(gamma * R * Tt_in);
    double FF_exp    = -(gamma + 1.0) / (2.0 * (gamma - 1.0));
    double FF        = std::sqrt(gamma / R) * MN_throat *
                       std::pow(1.0 + (gamma - 1.0) / 2.0 * MN_throat * MN_throat, FF_exp);
    Ath = (W * std::sqrt(Tt_in)) / (Pt_in * FF);

    // Step 9a — Cold gross thrust
    // Fg_cold = W × Vjet + (Ps_exit - Ps_amb) × Ath
    Fg_cold = W * Vjet + (Ps_exit - Ps_amb) * Ath;

    // Step 9b — Update flowOut — total conditions conserved through nozzle
    flowOut.Pt  = Pt_in;
    flowOut.Tt  = Tt_in;
    flowOut.W   = W;
    flowOut.MN  = MN_throat;
    flowOut.FAR = flowIn.FAR;

    // Step 9c — Real gas Cp and gamma at exit — same as inlet (Tt and FAR unchanged)
    flowOut.Cp    = Thermo::getCp   (flowOut.Tt, flowOut.FAR);
    flowOut.gamma = Thermo::getGamma(flowOut.Tt, flowOut.FAR);
}