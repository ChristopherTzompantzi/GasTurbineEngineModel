#ifndef FAN_H
#define FAN_H

#include "Element.h"
#include "Shaft.h"
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
 * 2. Computes corrected mass flow Wc and corrected speed Nc from shaft
 * 3. Looks up PR and eff from CompressorMap at (Wc, Nc) if map loaded
 *    Otherwise uses fixed PR_f_ and eff_f_ from constructor as fallback
 * 4. Raises total pressure by fan pressure ratio
 * 5. Raises total temperature accounting for isentropic efficiency
 * 6. Passes full mass flow to Splitter — BPR is handled there
 *
 * THERMODYNAMIC EQUATIONS (real gas via Thermo):
 *   Wc            = W * sqrt(Tt_in / T_ref) / (Pt_in / P_ref)  [kg/s corrected]
 *   Nc            = N_rpm / sqrt(Tt_in / T_ref)                 [RPM corrected]
 *   Pt_exit       = Pt_inlet × PR_f
 *   gamma         = Thermo::getGamma(Tt_inlet, FAR)
 *   Tt_exit_ideal = Tt_inlet × PR_f^((γ-1)/γ)
 *   Tt_exit       = Tt_inlet + (Tt_exit_ideal - Tt_inlet) / eff_f
 *   dHt           = Cp × (Tt_exit - Tt_inlet)   [J/kg]
 *
 * MAP LOOKUP:
 *   Shaft connected → Nc from shaft N_rpm → map lookup at (Wc, Nc)
 *   No shaft        → map lookup at design Nc from map header
 *   No map          → fixed PR_f_ and eff_f_ from constructor
 *
 * NPSS ALIGNMENT:
 *   loadMap()      mirrors NPSS S_map subelement pattern.
 *   connectShaft() mirrors NPSS shaft connection — element reads Nmech.
 *   Fan does not use VSV — IGV support deferred to Phase 5.
 *
 * WHY FAN IS SEPARATE FROM COMPRESSOR:
 *   Fan sits on LP shaft alongside LP Turbine.
 *   Compressor sits on HP shaft alongside HP Turbine.
 *   Separate classes make shaft connections explicit in main.cpp.
 *
 * FUTURE WORK:
 *   IGV schedule support (Phase 5).
 *   Fan diameter and tip speed constraints.
 *
 * UNITS:
 *   PR_f  [-]
 *   eff_f [-]
 *   dHt   [J/kg]
 *   Wc    [kg/s corrected]
 *   Nc    [RPM corrected]
 */

class Fan : public Element {
public:

    /*
     * Constructor — sets fallback fan pressure ratio and isentropic efficiency.
     * Call loadMap() and connectShaft() after construction.
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
     * connectShaft — connects this fan to its driving shaft (LP shaft).
     * Mirrors NPSS shaft connection pattern.
     * Once connected, compute() reads N_rpm from shaft to compute Nc.
     * If no shaft connected, map is queried at design Nc from header.
     *
     * @param shaft  Pointer to the connected Shaft
     */
    void connectShaft(Shaft* shaft) noexcept { shaft_ = shaft; }

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
    double dHt          = 0.0;

    // Corrected mass flow [kg/s] — available after compute()
    double Wc           = 0.0;

    // Corrected speed [RPM corrected] — available after compute()
    double Nc           = 0.0;

    // Surge margin [%] — available after compute() if map with surge line loaded
    // SM = (PR_surge(Wc) - PR_operating) / PR_surge(Wc) * 100
    // 0.0 if no surge line present in map file
    double surge_margin = 0.0;

private:

    // =========================================================================
    // PRIVATE MEMBERS
    // =========================================================================

    double                       PR_f_;          // Fallback fan pressure ratio [-]
    double                       eff_f_;         // Fallback isentropic efficiency [-]
    std::optional<CompressorMap> map_;           // CompressorMap subelement — present only if loadMap() called
    Shaft*                       shaft_ = nullptr; // Connected shaft — null if not connected
};

#endif // FAN_H