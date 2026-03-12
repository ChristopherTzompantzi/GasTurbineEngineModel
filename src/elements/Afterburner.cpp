#include "Afterburner.h"

/*
 * Afterburner.cpp
 * ---------------
 * Implements Afterburner element thermodynamics.
 * Similar to Combustor.cpp — see Combustor.cpp for base derivation.
 *
 * COMPUTE SEQUENCE:
 * 1. Read turbine exit conditions from flowIn
 * 2. Compute afterburner FAR increment from energy balance
 * 3. Compute total FAR — additive on existing core FAR
 * 4. Apply pressure loss
 * 5. Update mass flow with afterburner fuel addition
 * 6. Write results to flowOut
 *
 * FAR ACCOUNTING:
 * The afterburner adds fuel to a gas mixture that already contains
 * combustion products from the core. The total FAR must account for
 * both the original core FAR and the afterburner FAR increment:
 *
 *   FAR_ab    = Cp × (Tt7 - Tt_in) / (LHV × eff_ab - Cp × Tt7)
 *
 * This is the same equation as Combustor in Tt4 mode, applied to
 * the afterburner temperature rise (Tt7 - Tt_in).
 *
 *   FAR_total = FAR_core + FAR_ab × (1 + FAR_core)
 *
 * The (1 + FAR_core) factor scales FAR_ab because the afterburner
 * fuel is injected per unit of mixed gas flow, not per unit of
 * original air flow. This preserves correct mass accounting.
 *
 * Source: Mattingly, J.D., "Elements of Gas Turbine Propulsion",
 *         McGraw-Hill, 1996. Chapter 5 — afterburner cycle analysis.
 */

Afterburner::Afterburner(double Tt7,
                         double eff_ab,
                         double dPqP,
                         double LHV) noexcept
    : Tt7(Tt7)
    , eff_ab(eff_ab)
    , dPqP(dPqP)
    , LHV(LHV)
{}

void Afterburner::compute() noexcept
{
    // Read inlet conditions
    double Pt_in    = flowIn.Pt;
    double Tt_in    = flowIn.Tt;
    double W_in     = flowIn.W;
    double FAR_core = flowIn.FAR;
    double Cp       = flowIn.Cp;
    double gamma    = flowIn.gamma;

    // Afterburner FAR increment — energy balance
    // Same equation as Combustor Tt4 mode applied to afterburner temperature rise
    // FAR_ab = Cp × (Tt7 - Tt_in) / (LHV × eff_ab - Cp × Tt7)
    FAR_ab = Cp * (Tt7 - Tt_in) / (LHV * eff_ab - Cp * Tt7);

    // Total FAR — additive on existing core FAR
    // FAR_total = FAR_core + FAR_ab × (1 + FAR_core)
    double FAR_total = FAR_core + FAR_ab * (1.0 + FAR_core);

    // Exit total pressure — afterburner duct pressure loss
    double Pt_exit = Pt_in * (1.0 - dPqP);

    // Exit mass flow — adds afterburner fuel mass
    // W_exit = W_in × (1 + FAR_total) / (1 + FAR_core)
    double W_exit = W_in * (1.0 + FAR_total) / (1.0 + FAR_core);

    // Write to flowOut
    flowOut.Pt    = Pt_exit;
    flowOut.Tt    = Tt7;
    flowOut.W     = W_exit;
    flowOut.MN    = flowIn.MN;
    flowOut.FAR   = FAR_total;
    flowOut.gamma = gamma;
    flowOut.Cp    = Cp;
}