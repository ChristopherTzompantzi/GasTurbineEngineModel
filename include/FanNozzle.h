#ifndef FANNOZZLE_H
#define FANNOZZLE_H

#include "Element.h"
#include "ISA.h"
#include "NozzleMap.h"
#include <optional>

/*
 * FanNozzle.h
 * -----------
 * Models the bypass (cold) nozzle of a turbofan engine.
 * Thermodynamically identical to Nozzle — same choked/unchoked
 * logic, same thrust calculation, same flow function approach.
 *
 * WHAT THE FAN NOZZLE DOES:
 * 1. Receives bypass stream from Splitter (bypassOut)
 * 2. Evaluates real gas gamma and Cp at inlet conditions via Thermo
 * 3. Computes NPR vs ambient
 * 4. Gets Cfg from NozzleMap at current NPR if map loaded,
 *    otherwise uses fixed Cfg_ from constructor as fallback
 * 5. Determines choked or unchoked condition
 * 6. Computes jet velocity and cold gross thrust Fg_cold
 *
 * THERMODYNAMIC EQUATIONS (real gas via Thermo):
 *   NPR      = Pt_inlet / Ps_ambient
 *   gamma    = Thermo::getGamma(Tt_inlet, FAR)
 *   Cp       = Thermo::getCp(Tt_inlet, FAR)
 *   Cfg      = NozzleMap::lookup(NPR)  if map loaded, else fixed Cfg_
 *   Vjet     = Cfg × sqrt(2 × Cp × Tt_inlet × (1 - (Ps_exit/Pt_inlet)^((γ-1)/γ)))
 *   Fg_cold  = W_bypass × Vjet + (Ps_exit - Ps_ambient) × Ath
 *
 * NPSS ALIGNMENT:
 *   loadMap() mirrors NPSS S_Cfg subelement pattern.
 *   Same NozzleMap class as core Nozzle — different map file.
 *
 * FUTURE WORK:
 *   Variable area fan nozzle scheduling (Phase 5).
 *   Mixer option — bypass and core streams mixed before expansion.
 *   IGV support deferred to Phase 5.
 *
 * UNITS:
 *   NPR     [-]
 *   Cfg     [-]
 *   Vjet    [m/s]
 *   Fg_cold [N]
 *   Ath     [m²]
 */

class FanNozzle : public Element {
public:

    /*
     * Constructor — sets altitude and fallback discharge coefficient.
     * Call loadMap() after construction to enable map-based Cfg.
     *
     * @param altitude_m  Flight altitude [m]
     * @param Cfg         Fallback discharge coefficient [-]
     */
    FanNozzle(double altitude_m, double Cfg = 0.98) noexcept;

    /*
     * loadMap — loads a NozzleMap from file.
     * Mirrors NPSS S_Cfg subelement pattern.
     * Once loaded, compute() uses map lookup for Cfg.
     * If load fails, falls back to fixed Cfg_ with a warning.
     *
     * @param filepath  Path to the .map file (TYPE must be NOZZLE)
     */
    void loadMap(const std::string& filepath);

    /*
     * compute — performs fan nozzle thermodynamics.
     * Implements Element::compute().
     */
    void compute() noexcept override;

    // Outputs available after compute()
    double Fg_cold = 0.0;   // Cold stream gross thrust [N]
    double Vjet    = 0.0;   // Actual bypass jet velocity [m/s]
    double NPR     = 0.0;   // Nozzle pressure ratio [-]
    double Ath     = 0.0;   // Throat area [m²]

private:

    // =========================================================================
    // PRIVATE MEMBERS
    // =========================================================================

    double                   altitude_m_;   // Flight altitude [m]
    double                   Cfg_;          // Fallback discharge coefficient [-]
    std::optional<NozzleMap> map_;          // NozzleMap subelement — present only if loadMap() called
};

#endif // FANNOZZLE_H