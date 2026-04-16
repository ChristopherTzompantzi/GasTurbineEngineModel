#ifndef TURBINE_H
#define TURBINE_H

#include "Element.h"
#include "Shaft.h"
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
 * 2. Computes corrected speed Nc from shaft N_rpm
 * 3. Estimates initial Tt_exit to compute DhT for map lookup
 * 4. Looks up PR_t and eff_t from TurbineMap at (DhT, Nc) if map loaded
 *    Otherwise uses fixed PR_t_ and eff_t_ from constructor as fallback
 * 5. Expands the gas — drops total pressure and temperature
 * 6. Extracts shaft work to drive the compressor
 * 7. Writes real gas Cp and gamma to flowOut via Thermo at exit conditions
 *
 * THERMODYNAMIC EQUATIONS (real gas via Thermo):
 *   Nc            = N_rpm / sqrt(Tt_in / T_ref)       [RPM corrected]
 *   DhT           = Cp * (Tt_in - Tt_exit) / Tt_in    [-] dimensionless
 *   Pt_exit       = Pt_inlet / PR_t
 *   gamma         = Thermo::getGamma(Tt_inlet, FAR)
 *   Tt_exit_ideal = Tt_inlet × (1/PR_t)^((γ-1)/γ)
 *   Tt_exit       = Tt_inlet - eff_t × (Tt_inlet - Tt_exit_ideal)
 *   dHt           = Cp × (Tt_inlet - Tt_exit)         [J/kg]
 *
 * DhT CALCULATION (map lookup x-axis):
 *   DhT is computed from the current PR_t_ and eff_t_ (fallback) to
 *   estimate the operating point on the map. The map then returns the
 *   actual PR_t and eff_t for that operating point.
 *
 * MAP LOOKUP:
 *   Shaft connected → Nc from shaft N_rpm → map lookup at (DhT, Nc)
 *   No shaft        → map lookup at design Nc from map header
 *   No map          → fixed PR_t_ and eff_t_ from constructor / setPR()
 *
 * NPSS ALIGNMENT:
 *   loadMap()      mirrors NPSS S_map subelement pattern.
 *   connectShaft() mirrors NPSS shaft connection — element reads Nmech.
 *   setPR()        kept for fallback operation without map.
 *
 * SHAFT POWER BALANCE:
 *   getWork() returns dHt × (1 + FAR) — accounts for fuel mass added
 *   in combustor. Turbine processes (1+FAR) kg per kg of core air.
 *
 * FUTURE WORK:
 *   Cooling flow modelling — HP turbine blade cooling air.
 *   Turbine tip clearance control (Phase 5).
 *
 * UNITS:
 *   PR_t  [-]
 *   eff_t [-]
 *   dHt   [J/kg]
 *   DhT   [-]  dimensionless energy function (map x-axis)
 *   Nc    [RPM corrected]
 */

class Turbine : public Element {
public:

    /*
     * Constructor — sets fallback pressure ratio and isentropic efficiency.
     * Call loadMap() and connectShaft() after construction.
     *
     * @param PR_t   Fallback turbine pressure ratio [-]
     * @param eff_t  Fallback turbine isentropic efficiency [-]
     */
    Turbine(double PR_t, double eff_t) noexcept;

    /*
     * loadMap — loads a TurbineMap from file.
     * Mirrors NPSS S_map subelement pattern.
     * Once loaded, compute() uses map lookup for PR_t and eff_t.
     * If load fails, falls back to fixed PR_t_ and eff_t_ with a warning.
     *
     * @param filepath  Path to the .map file (TYPE must be TURBINE)
     */
    void loadMap(const std::string& filepath);

    /*
     * connectShaft — connects this turbine to its driven shaft.
     * Mirrors NPSS shaft connection pattern.
     * Once connected, compute() reads N_rpm from shaft to compute Nc.
     * If no shaft connected, map is queried at design Nc from header.
     *
     * @param shaft  Pointer to the connected Shaft
     */
    void connectShaft(Shaft* shaft) noexcept { shaft_ = shaft; }

    /*
     * compute — performs turbine thermodynamics.
     * Implements Element::compute().
     */
    void compute() noexcept override;

    /*
     * setPR — sets the fallback pressure ratio directly.
     * Used when operating without a map (no shaft speed iteration).
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
    double dHt  = 0.0;

    // Current turbine pressure ratio [-] — updated by compute()
    double PR_t = 1.0;

    // Polytropic efficiency [-] — available after compute()
    // Diagnostic output — does not drive thermodynamics (isentropic internally)
    // eta_poly < eta_is for turbine — reheat factor effect
    // Formula: ln(1 - eta_is*(1-(1/PR)^((γ-1)/γ))) / (-(γ-1)/γ * ln(PR))
    // Reference: Mattingly Ch.5, Walsh & Fletcher Ch.3
    double eta_poly = 0.0;

private:

    // =========================================================================
    // PRIVATE MEMBERS
    // =========================================================================

    double                    PR_t_;           // Fallback pressure ratio [-]
    double                    eff_t_;          // Fallback isentropic efficiency [-]
    std::optional<TurbineMap> map_;            // TurbineMap subelement — present only if loadMap() called
    Shaft*                    shaft_ = nullptr; // Connected shaft — null if not connected
};

#endif // TURBINE_H