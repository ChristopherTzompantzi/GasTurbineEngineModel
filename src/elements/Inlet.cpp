#include "Inlet.h"

/*
 * Inlet.cpp
 * ---------
 * Implements the Inlet element thermodynamics.
 *
 * COMPUTE SEQUENCE:
 * 1. Get ambient static conditions from ISA model
 * 2. Compute freestream total conditions (isentropic)
 * 3. Apply recovery factor to total pressure
 * 4. Pass mass flow through unchanged
 *
 * NOTE ON MASS FLOW:
 * Mass flow (W) is set externally by the solver before compute()
 * is called. The inlet does not generate or consume mass flow —
 * it passes it through unchanged. This mirrors NPSS behavior
 * where W is an independent variable iterated by the solver.
 */

Inlet::Inlet(double altitude_m, double Mach, double recovery_factor) noexcept
    : altitude_m(altitude_m)
    , Mach(Mach)
    , recovery_factor(recovery_factor)
{
    // flowIn and flowOut are initialized to ISA sea level
    // conditions by FlowStation constructor — safe starting point
}

void Inlet::compute() noexcept
{
    // Step 1 — Get ambient static conditions from ISA model
    double Ts = ISA::getStaticTemperature(altitude_m);
    double Ps = ISA::getStaticPressure(altitude_m);

    // Step 2 — Compute freestream total conditions
    // Uses perfect gas isentropic relations (Phase 1)
    double gamma = flowIn.gamma;
    double Tt    = ISA::getTotalTemperature(Ts, Mach, gamma);
    double Pt    = ISA::getTotalPressure(Ps, Mach, gamma);

    // Step 3 — Apply pressure recovery factor
    // Pt_exit = Pt_freestream × recovery_factor
    // Total temperature unchanged — inlet is adiabatic
    flowOut.Pt  = Pt * recovery_factor;
    flowOut.Tt  = Tt;

    // Step 4 — Pass remaining properties through unchanged
    flowOut.W     = flowIn.W;
    flowOut.Mach  = Mach;
    flowOut.FAR   = 0.0;      // No fuel added in inlet
    flowOut.gamma = flowIn.gamma;
    flowOut.Cp    = flowIn.Cp;

    // Update flowIn to reflect freestream total conditions.
    // This is intentional — flowIn of the Inlet records ambient total
    // conditions for diagnostics and output, not for upstream chaining.
    flowIn.Pt = Pt;
    flowIn.Tt = Tt;
}