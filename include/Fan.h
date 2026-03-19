#ifndef FAN_H
#define FAN_H

#include "Element.h"
#include "CompressorMap.h"
#include <optional>

/*
 * Fan.h
 * -----
 * Models the fan of a turbofan engine.
 * Aerodynamically identical to a compressor — raises total pressure
 * and temperature of the ENTIRE inlet mass flow (core + bypass).
 *
 * WHAT THE FAN DOES:
 * 1. Receives total inlet mass flow from Inlet
 * 2. Computes corrected mass flow Wc from inlet conditions
 * 3. Looks up PR and eff from CompressorMap at (Wc, Nc%) if map loaded
 *    Otherwise uses fixed PR_f_ and eff_f_ from constructor as fallback
 * 4. Raises total pressure by fan pressure ratio
 * 5. Raises total temperature accounting for isentropic efficiency
 * 6. Passes full mass flow to Splitter — BPR is handled there
 *
 * THERMODYNAMIC EQUATIONS (real gas via Thermo):
 *   Wc            = W * sqrt(Tt_in / T_ref) / (Pt_in / P_ref)
 *   Pt_exit       = Pt_inlet × PR_f
 *   gamma         = Thermo::getGamma(Tt_inlet, FAR)
 *   Tt_exit_ideal = Tt_inlet × PR_f^((γ-1)/γ)
 *   Tt_exit       = Tt_inlet + (Tt_exit_ideal - Tt_inlet) / eff_f
 *   dHt           = Cp × (Tt_exit - Tt_inlet)   [J/kg]
 *
 * MAP LOOKUP (Phase 3):
 *   Nc% = 100.0 — design speed line used throughout Phase 3.
 *   Phase 4 will replace 100.0 with computed LP shaft corrected speed.
 *
 * NPSS ALIGNMENT:
 *   loadMap() mirrors NPSS S_map subelement pattern.
 *   Fan does not use VSV — IGV support deferred to Phase 5.
 *
 * WHY FAN IS SEPARATE FROM COMPRESSOR:
 *   Fan sits on LP shaft alongside LP Turbine.
 *   Compressor sits on HP shaft alongside HP Turbine.
 *   Separate classes make shaft connections explicit in main.cpp.
 *
 * FUTURE WORK (Phase 4):
 *   Replace Nc%=100.0 with computed LP shaft corrected speed.
 *   IGV schedule support.
 *   Fan diameter and tip speed constraints.
 *
 * UNITS:
 *   PR_f  [-]
 *   eff_f [-]
 *   dHt   [J/kg]
 *   Wc    [kg/s corrected]
 *   Nc    [% of design corrected speed]
 */

class Fan : public Element {
public:

    /*
     * Constructor — sets fallback fan pressure ratio and isentropic efficiency.
     * Call loadMap() after construction to enable map-based PR and eff.
     *
     * @param PR_f   Fallback fan pressure ratio [-]
     * @param eff_f  Fallback isentropic efficiency [-]
     */
    Fan(double PR_f, double eff_f) noexcept;

    /*
     * loadMap — loads a CompressorMap from file.
     * Mirrors NPSS S_map subelement pattern.
     * Once loaded, compute() uses map lookup for PR_f and eff_f.
     * If load fails, falls back to fixed PR_f_ and eff_f_ with a warning.
     *
     * @param filepath  Path to the .map file (TYPE must be COMPRESSOR)
     */
    void loadMap(const std::string& filepath);

    /*
     * compute — performs fan thermodynamics.
     * Implements Element::compute().
     */
    void compute() noexcept override;

    /*
     * getWork — returns negative dHt [J/kg].
     * Sign convention: negative = power consumed from LP shaft.
     */
    double getWork() const noexcept override { return -dHt; }

    // Total enthalpy rise [J/kg] — available after compute()
    // dHt = Cp × (Tt_exit - Tt_inlet)
    double dHt = 0.0;

private:

    // =========================================================================
    // PRIVATE MEMBERS
    // =========================================================================

    double                       PR_f_;   // Fallback fan pressure ratio [-]
    double                       eff_f_;  // Fallback isentropic efficiency [-]
    std::optional<CompressorMap> map_;    // CompressorMap subelement — present only if loadMap() called
};

#endif // FAN_H