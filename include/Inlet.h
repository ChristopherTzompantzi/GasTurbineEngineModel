#ifndef INLET_H
#define INLET_H

#include "Element.h"
#include "ISA.h"
#include "InletMap.h"

#include <optional>

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
 * 5. Applies pressure recovery to account for inlet losses
 * 6. Writes real gas Cp and gamma to flowOut for downstream elements
 *    Total temperature is unchanged (adiabatic process)
 *
 * THERMODYNAMIC EQUATIONS:
 *   Ts    = ISA::getStaticTemperature(altitude)
 *   Ps    = ISA::getStaticPressure(altitude)
 *   gamma = Thermo::getGamma(Ts, 0.0)        — real gas, evaluated at Ts
 *   Tt    = ISA::getTotalTemperature(Ts, Mach, gamma)
 *   Pt    = ISA::getTotalPressure(Ps, Mach, gamma) × eta_r
 *   Cp    = Thermo::getCp(Tt, 0.0)           — written to flowOut for downstream
 *
 * RECOVERY FACTOR (eta_r):
 *   If a map is loaded via loadMap(), eta_r is interpolated from the
 *   InletMap at the current flight Mach number — matches NPSS S_Recovery
 *   subelement pattern.
 *   If no map is loaded, the fixed recovery_factor_ from the constructor
 *   is used as fallback. This mirrors NPSS default constant behaviour.
 *
 * NPSS ALIGNMENT:
 *   loadMap() mirrors the NPSS pattern of setting map properties after
 *   element construction:
 *     inlet.S_Recovery.filename = "inlet.map"
 *     inlet.S_Recovery.load()
 *
 * FUTURE WORK:
 *   Distortion index (DC60, DPcrit) — uniform flow assumed at compressor face.
 *   Angle of attack correction on eta_r.
 *   Supersonic inlet with normal shock recovery model.
 *
 * UNITS:
 *   altitude_m      [m]
 *   MN              [-]
 *   recovery_factor [-]  0.0–1.0
 *   eta_r           [-]  total pressure recovery from map
 */

class Inlet : public Element {
public:

    /*
     * Constructor — sets flight conditions and fallback recovery factor.
     * Call loadMap() after construction to enable map-based recovery.
     *
     * @param altitude_m      Flight altitude [m]
     * @param MN              Flight Mach number [-]
     * @param recovery_factor Fallback recovery factor if no map loaded [-]
     */
    Inlet(double altitude_m, double MN,
          double recovery_factor = 0.98) noexcept;

    /*
     * loadMap — loads an InletMap from file.
     * Mirrors NPSS S_Recovery subelement pattern.
     * Once loaded, compute() uses map lookup instead of fixed recovery_factor_.
     * If load fails, falls back to fixed recovery_factor_ and warns.
     *
     * @param filepath  Path to the .map file (TYPE must be INLET)
     */
    void loadMap(const std::string& filepath);

    /*
     * compute — performs inlet thermodynamics.
     * Implements Element::compute().
     */
    void compute() noexcept override;

private:

    // =========================================================================
    // PRIVATE MEMBERS
    // =========================================================================

    double                   altitude_m_;      // Flight altitude [m]
    double                   MN_;             // Flight Mach number [-]
    double                   recovery_factor_; // Fallback recovery factor [-] — used if no map
    std::optional<InletMap>  map_;             // InletMap subelement — present only if loadMap() succeeded
};

#endif // INLET_H