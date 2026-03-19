#ifndef TURBINE_H
#define TURBINE_H

#include "Element.h"
#include "TurbineMap.h"
#include <optional>

/*
 * Turbine.h
 * ---------
 * Declares the Turbine element — extracts work from the flow by expanding
 * hot combustion gases, driving the compressor shaft.
 * Inherits from Element.
 *
 * WHAT THE TURBINE DOES:
 * 1. Receives hot gas from Combustor exit
 * 2. Computes DhT from inlet conditions and current PR_t/eff_t
 * 3. Looks up PR_t and eff_t from TurbineMap at (DhT, Nc%) if map loaded
 *    Otherwise uses fixed PR_t and eff_t from constructor as fallback
 * 4. Expands the gas — drops total pressure and temperature
 * 5. Extracts shaft work to drive the compressor
 * 6. Writes real gas Cp and gamma to flowOut via Thermo at exit conditions
 *
 * THERMODYNAMIC EQUATIONS (real gas via Thermo):
 *   DhT           = Cp * (Tt_in - Tt_exit) / Tt_in   [-] dimensionless
 *   Pt_exit       = Pt_inlet / PR_t
 *   gamma         = Thermo::getGamma(Tt_inlet, FAR)
 *   Tt_exit_ideal = Tt_inlet × (1/PR_t)^((γ-1)/γ)
 *   Tt_exit       = Tt_inlet - eff_t × (Tt_inlet - Tt_exit_ideal)
 *   dHt           = Cp × (Tt_inlet - Tt_exit)         [J/kg]
 *
 * DhT CALCULATION (map lookup x-axis):
 *   DhT is computed from the current PR_t and eff_t (fallback or previous
 *   iteration) to determine the operating point on the map.
 *   At design point with Nc%=100, the map returns design PR_t and eff_t
 *   exactly — no iteration needed in Phase 3.
 *
 * MAP LOOKUP (Phase 3):
 *   Nc% = 100.0 — design speed line used throughout Phase 3.
 *   Phase 4 will replace 100.0 with computed shaft corrected speed.
 *
 * NPSS ALIGNMENT:
 *   loadMap() mirrors NPSS S_map subelement pattern.
 *   setPR() mirrors NPSS independent variable injection by solver.
 *
 * SHAFT POWER BALANCE:
 *   getWork() returns dHt × (1 + FAR) — accounts for fuel mass added
 *   in combustor. Turbine processes (1+FAR) kg per kg of core air.
 *
 * FUTURE WORK (Phase 4):
 *   Replace Nc%=100.0 with computed corrected shaft speed.
 *   Cooling flow modelling — HP turbine blade cooling air.
 *   Turbine tip clearance control (Phase 5).
 *
 * UNITS:
 *   PR_t  [-]
 *   eff_t [-]
 *   dHt   [J/kg]
 *   DhT   [-]  dimensionless energy function (map axis)
 *   Nc    [% of design corrected speed]
 */

class Turbine : public Element {
public:

    /*
     * Constructor — sets fallback pressure ratio and isentropic efficiency.
     * Call loadMap() after construction to enable map-based PR and eff.
     *
     * @param PR_t   Fallback turbine pressure ratio [-]
     * @param eff_t  Fallback turbine isentropic efficiency [-]
     */
    Turbine(double PR_t, double eff_t) noexcept;

    /*
     * loadMap — loads a TurbineMap from file.
     * Mirrors NPSS S_map subelement pattern.
     * Once loaded, compute() uses map lookup for PR_t and eff_t.
     * If load fails, falls back to fixed PR_t and eff_t with a warning.
     *
     * @param filepath  Path to the .map file (TYPE must be TURBINE)
     */
    void loadMap(const std::string& filepath);

    /*
     * compute — performs turbine thermodynamics.
     * Implements Element::compute().
     */
    void compute() noexcept override;

    /*
     * setPR — updates the fallback pressure ratio directly.
     * Used by the Newton-Raphson solver to inject independent variable
     * values into the cycle on each iteration.
     *
     * @param PR_t_new  New turbine pressure ratio [-]
     */
    void setPR(double PR_t_new) noexcept { PR_t_ = PR_t_new; }

    /*
     * getWork — returns dHt × (1 + FAR) [J/kg].
     * Sign convention: positive = power delivered to shaft.
     * (1+FAR) accounts for fuel mass added in combustor.
     */
    double getWork() const noexcept override
    {
        return dHt * (1.0 + flowOut.FAR);
    }

    // Specific work extracted [J/kg] — available after compute()
    double dHt = 0.0;

    // Turbine pressure ratio [-] — readable after construction, updated by setPR()
    double PR_t = 1.0;   // updated by compute() to reflect current operating value

private:

    // =========================================================================
    // PRIVATE MEMBERS
    // =========================================================================

    double                    PR_t_;   // Fallback turbine pressure ratio [-]
    double                    eff_t_;  // Fallback turbine isentropic efficiency [-]
    std::optional<TurbineMap> map_;    // TurbineMap subelement — present only if loadMap() called
};

#endif // TURBINE_H