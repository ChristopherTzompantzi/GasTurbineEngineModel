#ifndef NOZZLE_H
#define NOZZLE_H

#include "Element.h"
#include "ISA.h"
#include "NozzleMap.h"
#include <optional>

/*
 * Nozzle.h
 * --------
 * Declares the Nozzle element — converts remaining total pressure
 * into jet velocity and thrust, completing the Brayton cycle.
 * Inherits from Element.
 *
 * WHAT THE NOZZLE DOES:
 * 1. Receives expanded gas from Turbine exit
 * 2. Evaluates real gas gamma and Cp at inlet conditions via Thermo
 * 3. Determines choked or unchoked condition from NPR
 * 4. Gets Cfg from NozzleMap at current NPR if map loaded,
 *    otherwise uses fixed Cfg_ from constructor as fallback
 * 5. Expands gas to ambient (or critical) pressure
 * 6. Computes jet velocity and gross thrust Fg
 *
 * CHOKED vs UNCHOKED:
 *   NPR_crit = ((γ+1)/2)^(γ/(γ-1)) ≈ 1.893 for γ=1.4
 *   NPR < NPR_crit → unchoked → Ps_exit = Ps_ambient
 *   NPR ≥ NPR_crit → choked  → Ps_exit = Pt_inlet / NPR_crit
 *
 * THERMODYNAMIC EQUATIONS (real gas via Thermo):
 *   NPR      = Pt_inlet / Ps_ambient
 *   gamma    = Thermo::getGamma(Tt_inlet, FAR)
 *   Cp       = Thermo::getCp(Tt_inlet, FAR)
 *   Cfg      = NozzleMap::lookup(NPR)  if map loaded, else fixed Cfg_
 *   Vjet     = Cfg × sqrt(2 × Cp × Tt_inlet × (1 - (Ps_exit/Pt_inlet)^((γ-1)/γ)))
 *   Fg       = W × Vjet + (Ps_exit - Ps_ambient) × Ath
 *
 * NPSS ALIGNMENT:
 *   loadMap() mirrors NPSS S_Cfg subelement pattern.
 *
 * FUTURE WORK:
 *   Convergent-divergent nozzle modelling.
 *   Fixed Ath as geometric input.
 *   Variable area nozzle scheduling (Phase 5).
 *
 * UNITS:
 *   NPR  [-]
 *   Cfg  [-]
 *   Vjet [m/s]
 *   Fg   [N]
 *   Ath  [m²]
 */

class Nozzle : public Element {
public:

    /*
     * Constructor — sets altitude and fallback discharge coefficient.
     * Call loadMap() after construction to enable map-based Cfg.
     *
     * @param altitude_m  Flight altitude [m]
     * @param Cfg         Fallback discharge coefficient [-]
     */
    Nozzle(double altitude_m, double Cfg = 0.98) noexcept;

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
     * compute — performs nozzle thermodynamics.
     * Implements Element::compute().
     */
    void compute() noexcept override;

    // Outputs available after compute()
    double Fg   = 0.0;   // Gross thrust [N]
    double Vjet = 0.0;   // Jet exit velocity [m/s]
    double Ath  = 0.0;   // Calculated throat area [m²]
    double NPR  = 0.0;   // Nozzle pressure ratio [-]

private:

    // =========================================================================
    // PRIVATE MEMBERS
    // =========================================================================

    double                   altitude_m_;   // Flight altitude [m]
    double                   Cfg_;          // Fallback discharge coefficient [-]
    std::optional<NozzleMap> map_;          // NozzleMap subelement — present only if loadMap() called
};

#endif // NOZZLE_H