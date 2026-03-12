#ifndef FANNOZZLE_H
#define FANNOZZLE_H

#include "Element.h"
#include "ISA.h"

/*
 * FanNozzle.h
 * -----------
 * Models the bypass (cold) nozzle of a turbofan engine.
 * Expands the bypass stream to produce cold thrust.
 * Thermodynamically identical to Nozzle — same choked/unchoked
 * logic, same thrust calculation, same flow function approach.
 *
 * WHAT THE FAN NOZZLE DOES:
 * 1. Receives bypass stream from Splitter (bypassOut)
 * 2. Computes nozzle pressure ratio (NPR) vs ambient
 * 3. Determines choked or unchoked condition
 * 4. Computes ideal jet velocity via isentropic expansion
 * 5. Applies Cfg to get actual jet velocity
 * 6. Computes throat area from flow function
 * 7. Computes cold gross thrust Fg_cold
 *
 * DIFFERENCE FROM CORE NOZZLE:
 * Processes bypass stream mass flow only — W_bypass from Splitter.
 * Typically lower NPR than core nozzle — bypass stream is cooler
 * and at lower pressure than core exhaust.
 * Usually unchoked at typical turbofan cruise conditions.
 *
 * SHAFT WORK:
 *   getWork() returns 0.0 — no shaft interaction
 *   (inherited default from Element)
 *
 * NAMING:
 *   Fg_cold — cold (bypass) stream gross thrust [N]
 *   Vjet    — actual bypass jet velocity [m/s]
 *   NPR     — nozzle pressure ratio [-]
 *   Ath     — throat area [m²]
 *   Cfg     — nozzle velocity coefficient [-]
 *
 * PHASE 2:
 *   Variable area fan nozzle for off-design operation.
 *   Cfg vs NPR map for more accurate thrust prediction.
 *   Mixer option — bypass and core streams mixed before expansion.
 */

class FanNozzle : public Element
{
public:
    // Constructor — altitude for ambient pressure, nozzle efficiency
    FanNozzle(double altitude_m, double Cfg = 0.98) noexcept;

    // Implements Element::compute() — performs fan nozzle thermodynamics
    void compute() noexcept override;

    // Performance outputs — available after compute()
    double Fg_cold = 0.0;   // Cold stream gross thrust [N]
    double Vjet    = 0.0;   // Actual bypass jet velocity [m/s]
    double NPR     = 0.0;   // Nozzle pressure ratio [-]
    double Ath     = 0.0;   // Throat area [m²]

private:
    // Members always initialized via constructor — no in-class defaults needed.
    double altitude_m;   // Flight altitude [m] — for ISA ambient pressure
    double Cfg;          // Nozzle velocity coefficient [-]
};

#endif // FANNOZZLE_H