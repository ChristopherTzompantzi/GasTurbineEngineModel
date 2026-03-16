#include "Inlet.h"
#include "Thermo.h"

/*
 * Inlet.cpp
 * ---------
 * Implements the Inlet element thermodynamics.
 *
 * COMPUTE SEQUENCE:
 * 1. Get ambient static conditions from ISA model
 * 2. Evaluate real gas properties at ambient static temperature (FAR=0)
 * 3. Compute freestream total conditions (isentropic)
 * 4. Apply recovery factor to total pressure
 * 5. Pass mass flow through unchanged
 * 6. Write real gas Cp and gamma to flowOut via Thermo
 *
 * NOTE ON MASS FLOW:
 * Mass flow (W) is set externally by the solver before compute()
 * is called. The inlet does not generate or consume mass flow —
 * it passes it through unchanged. This mirrors NPSS behavior
 * where W is an independent variable iterated by the solver.
 *
 * NOTE ON GAMMA:
 * gamma is evaluated at the ambient static temperature (Ts) and
 * FAR=0 (pure air). This is physically correct — the isentropic
 * relations apply to the static-to-total compression of ambient air,
 * which occurs at conditions close to Ts, not Tt.
 * flowOut.gamma is then re-evaluated at Tt for downstream elements.
 */

Inlet::Inlet(double altitude_m, double MN, double recovery_factor) noexcept
    : altitude_m(altitude_m)
    , MN(MN)
    , recovery_factor(recovery_factor)
{
    // flowIn and flowOut are initialized to ISA sea level
    // conditions by FlowStation constructor — safe starting point
}

void Inlet::compute() noexcept
{
    // Step 1 — Get ambient static conditions from ISA model
    const double Ts = ISA::getStaticTemperature(altitude_m);
    const double Ps = ISA::getStaticPressure(altitude_m);

    // Step 2 — Evaluate real gas gamma at ambient static conditions
    // FAR=0 — pure air upstream of combustor
    // Ts used here because isentropic relations act on static-to-total
    // compression, which occurs close to Ts not Tt
    const double gamma = Thermo::getGamma(Ts, 0.0);

    // Step 3 — Compute freestream total conditions
    // Isentropic relations using real gas gamma at Ts
    const double Tt = ISA::getTotalTemperature(Ts, MN, gamma);
    const double Pt = ISA::getTotalPressure(Ps, MN, gamma);

    // Step 4 — Apply pressure recovery factor
    // Pt_exit = Pt_freestream × recovery_factor
    // Total temperature unchanged — inlet is adiabatic
    flowOut.Pt = Pt * recovery_factor;
    flowOut.Tt = Tt;

    // Step 5 — Pass remaining properties through unchanged
    flowOut.W   = flowIn.W;
    flowOut.MN  = MN;
    flowOut.FAR = 0.0;   // No fuel added in inlet

    // Step 6 — Write real gas Cp and gamma to flowOut
    // Evaluated at exit total temperature Tt, FAR=0 (pure air)
    // These values will be read by the downstream element (Fan or Compressor)
    flowOut.Cp    = Thermo::getCp   (flowOut.Tt, flowOut.FAR);
    flowOut.gamma = Thermo::getGamma(flowOut.Tt, flowOut.FAR);

    // Update flowIn to reflect freestream total conditions.
    // This is intentional — flowIn of the Inlet records ambient total
    // conditions for diagnostics and output, not for upstream chaining.
    flowIn.Pt = Pt;
    flowIn.Tt = Tt;
}