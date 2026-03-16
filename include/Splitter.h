#ifndef SPLITTER_H
#define SPLITTER_H

#include "Element.h"
#include "Thermo.h"

/*
 * Splitter.h
 * ----------
 * Divides the fan exit flow into bypass and core streams
 * based on the bypass ratio (BPR).
 *
 * WHAT THE SPLITTER DOES:
 * 1. Receives total fan exit mass flow on flowIn
 * 2. Divides mass flow into bypass and core streams by BPR
 * 3. Total pressure and temperature are identical in both streams
 *    — the splitter does no thermodynamic work
 * 4. Writes real gas Cp and gamma to both flowOut and bypassOut via Thermo
 *
 * BYPASS RATIO (BPR):
 *   BPR = bypass mass flow / core mass flow
 *   W_bypass = W_total × BPR / (1 + BPR)
 *   W_core   = W_total × 1   / (1 + BPR)
 *
 * TWO OUTLET PORTS:
 *   flowOut   — core stream   → Compressor (inherited from Element)
 *   bypassOut — bypass stream → FanNozzle  (additional port)
 *
 * SHAFT WORK:
 *   getWork() returns 0.0 — Splitter has no shaft interaction
 *   (inherited default from Element)
 *
 * TYPICAL BPR VALUES:
 *   Low bypass  (military):  0.2 – 1.0   e.g. F404
 *   Medium bypass (regional): 4.0 – 6.0  e.g. CFM56
 *   High bypass (widebody):   8.0 – 12.0 e.g. GE9X
 *
 * FUTURE WORK:
 *   Splitter pressure loss — small total pressure drop at the bifurcation.
 *   Variable BPR for off-design analysis.
 */

class Splitter : public Element
{
public:
    // Constructor — bypass ratio
    explicit Splitter(double BPR) noexcept;

    // Implements Element::compute() — divides mass flow by BPR
    void compute() noexcept override;

    // Bypass stream outlet — connects to FanNozzle
    // Total pressure and temperature identical to flowOut (core stream)
    // Only mass flow differs
    FlowStation bypassOut;

private:
    // Members always initialized via constructor — no in-class defaults needed.
    double BPR;   // Bypass ratio [-]
};

#endif // SPLITTER_H