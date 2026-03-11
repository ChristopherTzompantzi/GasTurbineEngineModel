#ifndef INLET_H
#define INLET_H

#include "Element.h"
#include "ISA.h"

/*
 * Inlet.h
 * -------
 * Declares the Inlet element — the first component in the engine cycle.
 * Inherits from Element and implements compute() with inlet thermodynamics.
 *
 * WHAT THE INLET DOES (Phase 1):
 * 1. Takes flight conditions (altitude, Mach number)
 * 2. Uses ISA model to compute ambient static conditions
 * 3. Computes freestream total conditions via isentropic relations
 * 4. Applies pressure recovery factor to account for inlet losses
 * 5. Total temperature is unchanged (adiabatic process)
 *
 * THERMODYNAMIC EQUATIONS:
 *   Ts  = ISA::getStaticTemperature(altitude)
 *   Ps  = ISA::getStaticPressure(altitude)
 *   Tt  = ISA::getTotalTemperature(Ts, Mach, gamma)
 *   Pt  = ISA::getTotalPressure(Ps, Mach, gamma) × recovery_factor
 *
 * RECOVERY FACTOR:
 * Set via constructor in Phase 1 (constant value).
 * Typical values: 0.99 (bellmouth), 0.97-0.98 (flight inlet)
 * Phase 2 consideration: replace with inlet performance map
 * that varies recovery with Mach number and angle of attack.
 *
 * DISTORTION:
 * Not modeled in Phase 1 — uniform flow assumed at compressor face.
 * Phase 2 consideration: add distortion index (DC60, DPcrit).
 *
 * UNITS:
 * altitude_m — meters [m]
 * Mach       — dimensionless [-]
 * recovery   — dimensionless [-], must be between 0.0 and 1.0
 */

class Inlet : public Element {
public:
    // Constructor — sets flight conditions and recovery factor
    Inlet(double altitude_m, double Mach, double recovery_factor = 0.98) noexcept;

    // Implements Element::compute() — performs inlet thermodynamics
    void compute() noexcept override;

private:
    // Members always initialized via constructor — no in-class defaults needed.
    double altitude_m;       // Flight altitude [m]
    double Mach;             // Flight Mach number [-]
    double recovery_factor;  // Inlet pressure recovery [-]
};

#endif // INLET_H