#include "Splitter.h"

/*
 * Splitter.cpp
 * ------------
 * Implements Splitter element — divides fan exit flow into
 * bypass and core streams by bypass ratio (BPR).
 *
 * COMPUTE SEQUENCE:
 * 1. Read total mass flow and conditions from flowIn
 * 2. Compute bypass and core mass flows from BPR
 * 3. Write core stream to flowOut   — W_core,   same Pt/Tt
 * 4. Write bypass stream to bypassOut — W_bypass, same Pt/Tt
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
    double W_total = flowIn.W;

    // Divide mass flow by bypass ratio
    // W_bypass = W_total × BPR / (1 + BPR)
    // W_core   = W_total × 1   / (1 + BPR)
    double W_bypass = W_total * BPR  / (1.0 + BPR);
    double W_core   = W_total * 1.0  / (1.0 + BPR);

    // Core stream — flowOut (inherited from Element)
    // Pt, Tt, FAR, gamma, Cp identical to fan exit
    flowOut.Pt    = flowIn.Pt;
    flowOut.Tt    = flowIn.Tt;
    flowOut.W     = W_core;
    flowOut.MN    = flowIn.MN;
    flowOut.FAR   = flowIn.FAR;
    flowOut.gamma = flowIn.gamma;
    flowOut.Cp    = flowIn.Cp;

    // Bypass stream — bypassOut (additional port)
    // Pt, Tt, FAR, gamma, Cp identical to fan exit
    bypassOut.Pt    = flowIn.Pt;
    bypassOut.Tt    = flowIn.Tt;
    bypassOut.W     = W_bypass;
    bypassOut.MN    = flowIn.MN;
    bypassOut.FAR   = flowIn.FAR;
    bypassOut.gamma = flowIn.gamma;
    bypassOut.Cp    = flowIn.Cp;
}