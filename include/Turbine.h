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
 * 5. Applies tip clearance correction to eff_t (Phase 5.3)
 * 6. Expands the gas — drops total pressure and temperature
 * 7. Extracts shaft work to drive the compressor
 * 8. Writes real gas Cp and gamma to flowOut via Thermo at exit conditions
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
 * TIP CLEARANCE CORRECTION (Phase 5.3):
 *   Models the effect of blade tip-to-casing clearance on efficiency.
 *   Applied as a fractional efficiency penalty after map lookup:
 *     eff_t_corrected = eff_t_map * (1 - clearance_corr)
 *   clearance_corr = 0.0  → no correction (default)
 *   clearance_corr = 0.005 → 0.5% efficiency penalty (typical HP turbine cruise)
 *
 *   Physical basis: tip leakage flow bypasses the blade passage without
 *   doing work. Larger clearance = more leakage = lower efficiency.
 *   Active Clearance Control (ACC) reduces clearance at cruise by cooling
 *   the casing, shrinking it thermally toward the blade tips.
 *
 *   Typical values (Walsh & Fletcher Ch.5, Mattingly App D):
 *     HP turbine at cruise with ACC: 0.003 – 0.005 (0.3 – 0.5%)
 *     HP turbine at takeoff (hot, large clearance): 0.008 – 0.015
 *     LP turbine: lower correction (larger blades, better relative clearance)
 *
 * MAP LOOKUP:
 *   Shaft connected → Nc from shaft N_rpm → map lookup at (DhT, Nc)
 *   No shaft        → map lookup at design Nc from map header
 *   No map          → fixed PR_t_ and eff_t_ from constructor / setPR()
 *
 * NPSS ALIGNMENT:
 *   loadMap()        mirrors NPSS S_map subelement pattern.
 *   connectShaft()   mirrors NPSS shaft connection — element reads Nmech.
 *   setPR()          kept for fallback operation without map.
 *   clearance_corr   mirrors NPSS TipClearance subelement delta_eff correction.
 *
 * SHAFT POWER BALANCE:
 *   getWork() returns dHt × (1 + FAR) — accounts for fuel mass added
 *   in combustor. Turbine processes (1+FAR) kg per kg of core air.
 *
 * FUTURE WORK:
 *   Clearance correction as function of power setting (ACC scheduling).
 *   Cooling flow modelling — HP turbine blade cooling air.
 *
 * UNITS:
 *   PR_t           [-]
 *   eff_t          [-]
 *   clearance_corr [-]  fractional efficiency penalty
 *   dHt            [J/kg]
 *   DhT            [-]  dimensionless energy function (map x-axis)
 *   Nc             [RPM corrected]
 */

class Turbine : public Element {
public:

    /*
     * Constructor — sets fallback pressure ratio, isentropic efficiency,
     * and optional tip clearance correction factor.
     * Call loadMap() and connectShaft() after construction.
     *
     * @param PR_t          Fallback turbine pressure ratio [-]
     * @param eff_t         Fallback turbine isentropic efficiency [-]
     * @param clearance_corr  Fractional efficiency penalty from tip clearance [-]
     *                        0.0 = no correction (default)
     *                        0.005 = 0.5% penalty (typical HP turbine cruise)
     */
    Turbine(double PR_t,
            double eff_t,
            double clearance_corr = 0.0) noexcept;

    /*
     * loadMap — loads a TurbineMap from file.
     * Mirrors NPSS S_map subelement pattern.
     */
    void loadMap(const std::string& filepath);

    /*
     * connectShaft — connects this turbine to its driven shaft.
     */
    void connectShaft(Shaft* shaft) noexcept { shaft_ = shaft; }

    /*
     * compute — performs turbine thermodynamics.
     * Implements Element::compute().
     */
    void compute() noexcept override;

    /*
     * setPR — sets the fallback pressure ratio directly.
     */
    void setPR(double PR_t_new) noexcept { PR_t_ = PR_t_new; }

    /*
     * getWork — returns dHt × (1 + FAR) [J/kg].
     * Sign convention: positive = power delivered to shaft.
     */
    double getWork() const noexcept override
    {
        return dHt * (1.0 + flowOut.FAR);
    }

    // Specific work extracted [J/kg] — available after compute()
    double dHt = 0.0;

    // Current turbine pressure ratio [-] — updated by compute()
    double PR_t = 1.0;

    // Isentropic efficiency [-] — current value after map lookup and
    // tip clearance correction, available after compute()
    double eff_is = 0.0;

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

    double                    PR_t_;            // Fallback pressure ratio [-]
    double                    eff_t_;           // Fallback isentropic efficiency [-]
    double                    clearance_corr_;  // Tip clearance efficiency penalty [-]
    std::optional<TurbineMap> map_;             // TurbineMap subelement
    Shaft*                    shaft_ = nullptr; // Connected shaft
};

#endif // TURBINE_H