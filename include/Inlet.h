#ifndef INLET_H
#define INLET_H

#include "Element.h"
#include "ISA.h"
#include "Thermo.h"

/*
 * Inlet.h
 * -------
 * Declares the Inlet element — the first component in the engine cycle.
 * Inherits from Element and implements compute() with inlet thermodynamics.
 *
 * WHAT THE INLET DOES:
 * 1. Takes flight conditions (altitude, Mach number)
 * 2. Uses ISA model to compute ambient static conditions
 * 3. Evaluates real gas gamma at ambient static temperature via Thermo
 * 4. Computes freestream total conditions via isentropic relations
 * 5. Applies pressure recovery factor to account for inlet losses
 * 6. Writes real gas Cp and gamma to flowOut for downstream elements
 *    Total temperature is unchanged (adiabatic process)
 *
 * THERMODYNAMIC EQUATIONS:
 *   Ts    = ISA::getStaticTemperature(altitude)
 *   Ps    = ISA::getStaticPressure(altitude)
 *   gamma = Thermo::getGamma(Ts, 0.0)        — real gas, evaluated at Ts
 *   Tt    = ISA::getTotalTemperature(Ts, Mach, gamma)
 *   Pt    = ISA::getTotalPressure(Ps, Mach, gamma) × recovery_factor
 *   Cp    = Thermo::getCp(Tt, 0.0)           — written to flowOut for downstream
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
 * MN         — dimensionless [-]
 * recovery   — dimensionless [-], must be between 0.0 and 1.0
 */

class Inlet : public Element {
public:
    // Constructor — sets flight conditions and recovery factor
    Inlet(double altitude_m, double MN, double recovery_factor = 0.98) noexcept;

    // Implements Element::compute() — performs inlet thermodynamics
    void compute() noexcept override;

private:
    // Members always initialized via constructor — no in-class defaults needed.
    double altitude_m;       // Flight altitude [m]
    double MN;               // Flight Mach number [-]
    double recovery_factor;  // Inlet pressure recovery [-]
};

#endif // INLET_H