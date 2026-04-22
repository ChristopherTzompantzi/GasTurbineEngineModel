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
 * 7. Computes throat area — from scheduled Ath if set, else flow function
 *
 * THERMODYNAMIC EQUATIONS (real gas via Thermo):
 *   NPR      = Pt_inlet / Ps_ambient
 *   gamma    = Thermo::getGamma(Tt_inlet, FAR)
 *   Cp       = Thermo::getCp(Tt_inlet, FAR)
 *   Cfg      = NozzleMap::lookup(NPR)  if map loaded, else fixed Cfg_
 *   Vjet     = Cfg × sqrt(2 × Cp × Tt_inlet × (1 - (Ps_exit/Pt_inlet)^((γ-1)/γ)))
 *   Fg_cold  = W_bypass × Vjet + (Ps_exit - Ps_ambient) × Ath
 *
 * VARIABLE AREA FAN NOZZLE (Phase 5.5):
 *   Same two modes as core Nozzle:
 *   Computed mode  (default):  Ath from flow function — design point use
 *   Scheduled mode (setAth):   Ath fixed by operator — off-design use
 *   Physical basis: fan nozzle area is scheduled against BPR and thrust
 *   setting on high-bypass engines for noise and efficiency optimisation.
 *
 * NPSS ALIGNMENT:
 *   loadMap() mirrors NPSS S_Cfg subelement pattern.
 *   setAth()  mirrors NPSS FanNozzle Ath as geometric input.
 *
 * FUTURE WORK:
 *   Mixer option — bypass and core streams mixed before expansion.
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
     */
    FanNozzle(double altitude_m, double Cfg = 0.98) noexcept;

    /*
     * loadMap — loads a NozzleMap from file.
     * Mirrors NPSS S_Cfg subelement pattern.
     */
    void loadMap(const std::string& filepath);

    /*
     * setAth — sets a fixed throat area, switching to scheduled mode.
     * Once called, compute() uses this Ath instead of the flow function.
     * Call resetAth() to return to computed mode.
     *
     * @param Ath_m2  Throat area [m²]
     */
    void setAth(double Ath_m2) noexcept
    {
        ath_scheduled_     = Ath_m2;
        use_scheduled_ath_ = true;
    }

    /*
     * resetAth — returns to computed mode (Ath derived from flow function).
     */
    void resetAth() noexcept { use_scheduled_ath_ = false; }

    /*
     * compute — performs fan nozzle thermodynamics.
     * Implements Element::compute().
     */
    void compute() noexcept override;

    // Outputs available after compute()
    double Fg_cold = 0.0;   // Cold stream gross thrust [N]
    double Vjet    = 0.0;   // Actual bypass jet velocity [m/s]
    double NPR     = 0.0;   // Nozzle pressure ratio [-]
    double Ath     = 0.0;   // Throat area [m²] — computed or scheduled

private:

    double                   altitude_m_;
    double                   Cfg_;
    std::optional<NozzleMap> map_;
    double                   ath_scheduled_     = 0.0;
    bool                     use_scheduled_ath_ = false;
};

#endif // FANNOZZLE_H