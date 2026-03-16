#include "Splitter.h"
#include "Thermo.h"

/*
 * Splitter.cpp
 * ------------
 * Implements Splitter element — divides fan exit flow into
 * bypass and core streams by bypass ratio (BPR).
 *
 * COMPUTE SEQUENCE:
 * 1. Read total mass flow and conditions from flowIn
 * 2. Compute bypass and core mass flows from BPR
 * 3. Write core stream to flowOut   — W_core,   same Pt/Tt/FAR
 *    Write real gas Cp and gamma to flowOut via Thermo
 * 4. Write bypass stream to bypassOut — W_bypass, same Pt/Tt/FAR
 *    Write real gas Cp and gamma to bypassOut via Thermo
 *
 * NO THERMODYNAMIC WORK:
 * The splitter is a pure flow divider — it does not change
 * total pressure or temperature. Both outlets share identical
 * Pt and Tt from the fan exit. Only mass flow differs.
 *
 * Source: Mattingly, J.D., "Elements of Gas Turbine Propulsion",
 *         McGraw-Hill, 1996. Chapter 4 — turbofan cycle analysis.
 */

Splitter::Splitter(double BPR) noexcept
    : BPR(BPR)
{}

void Splitter::compute() noexcept
{
    // Step 1 — Read total mass flow
    const double W_total = flowIn.W;

    // Step 2 — Divide mass flow by bypass ratio
    // W_bypass = W_total × BPR / (1 + BPR)
    // W_core   = W_total × 1   / (1 + BPR)
    double W_bypass = W_total * BPR  / (1.0 + BPR);
    double W_core   = W_total * 1.0  / (1.0 + BPR);

    // Step 3 — Core stream — flowOut (inherited from Element)
    // Pt, Tt, FAR identical to fan exit — only W differs
    flowOut.Pt  = flowIn.Pt;
    flowOut.Tt  = flowIn.Tt;
    flowOut.W   = W_core;
    flowOut.MN  = flowIn.MN;
    flowOut.FAR = flowIn.FAR;

    // Real gas Cp and gamma — recomputed via Thermo at core exit conditions
    flowOut.Cp    = Thermo::getCp   (flowOut.Tt, flowOut.FAR);
    flowOut.gamma = Thermo::getGamma(flowOut.Tt, flowOut.FAR);

    // Step 4 — Bypass stream — bypassOut (additional port)
    // Pt, Tt, FAR identical to fan exit — only W differs
    bypassOut.Pt  = flowIn.Pt;
    bypassOut.Tt  = flowIn.Tt;
    bypassOut.W   = W_bypass;
    bypassOut.MN  = flowIn.MN;
    bypassOut.FAR = flowIn.FAR;

    // Real gas Cp and gamma — recomputed via Thermo at bypass exit conditions
    bypassOut.Cp    = Thermo::getCp   (bypassOut.Tt, bypassOut.FAR);
    bypassOut.gamma = Thermo::getGamma(bypassOut.Tt, bypassOut.FAR);
}