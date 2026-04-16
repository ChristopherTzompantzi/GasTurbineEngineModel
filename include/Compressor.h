#ifndef COMPRESSOR_H
#define COMPRESSOR_H

#include "Element.h"
#include "Shaft.h"
#include "CompressorMap.h"
#include <optional>

/*
 * Compressor.h
 * ------------
 * Declares the Compressor element — raises total pressure and temperature
 * by doing work on the flow. Inherits from Element.
 *
 * WHAT THE COMPRESSOR DOES:
 * 1. Receives flow from Inlet at compressor face conditions
 * 2. Computes corrected mass flow Wc and corrected speed Nc from shaft
 * 3. Looks up PR and eff from CompressorMap at (Wc, Nc) if map loaded
 *    Otherwise uses fixed PR_ and eff_ from constructor as fallback
 * 4. Raises total pressure by pressure ratio (PR)
 * 5. Raises total temperature accounting for isentropic efficiency (eff)
 * 6. Writes real gas Cp and gamma to flowOut via Thermo at exit conditions
 * 7. Mass flow passes through unchanged
 *
 * THERMODYNAMIC EQUATIONS (real gas via Thermo):
 *   Wc             = W * sqrt(Tt_in / T_ref) / (Pt_in / P_ref)  [kg/s corrected]
 *   Nc             = N_rpm / sqrt(Tt_in / T_ref)                 [RPM corrected]
 *   Pt_exit        = Pt_inlet × PR
 *   gamma          = Thermo::getGamma(Tt_inlet, FAR)
 *   Tt_exit_ideal  = Tt_inlet × PR^((γ-1)/γ)
 *   Tt_exit        = Tt_inlet + (Tt_exit_ideal - Tt_inlet) / eff
 *   dHt            = Cp × (Tt_exit - Tt_inlet)                   [J/kg]
 *
 * CORRECTED FLOW AND SPEED:
 *   Wc = W * sqrt(Tt_in / 288.15) / (Pt_in / 101325.0)   [kg/s corrected]
 *   Nc = N_rpm / sqrt(Tt_in / 288.15)                     [RPM corrected]
 *   Reference: T_ref=288.15 K, P_ref=101325 Pa (ISA sea level)
 *
 * MAP LOOKUP:
 *   Shaft connected → Nc computed from shaft N_rpm → map lookup at (Wc, Nc)
 *   No shaft        → map lookup at design Nc from map header (design point)
 *   No map          → fixed PR_ and eff_ from constructor
 *
 * NPSS ALIGNMENT:
 *   loadMap()      mirrors NPSS S_map subelement pattern
 *   connectShaft() mirrors NPSS shaft connection — element reads Nmech
 *
 * ISENTROPIC EFFICIENCY:
 *   eff = 1.0 → perfect isentropic compression (no losses).
 *   Typical range: 0.85–0.92 for modern compressors.
 *
 * POLYTROPIC EFFICIENCY (Phase 5.2):
 *   eta_poly is a diagnostic output computed after each map lookup.
 *   It does not drive thermodynamics — isentropic efficiency is used
 *   internally throughout. This matches NPSS eff_poly output behaviour.
 *   Formula (perfect gas approximation, Mattingly Ch.5):
 *     eta_poly = ln(PR) * (γ-1)/γ
 *              / ln(1 + (PR^((γ-1)/γ) - 1) / eta_is)
 *
 * UNITS:
 *   PR   [-]
 *   eff  [-]
 *   dHt  [J/kg]
 *   Wc   [kg/s corrected]
 *   Nc   [RPM corrected]
 */

class Compressor : public Element {
public:

    /*
     * Constructor — sets fallback pressure ratio and isentropic efficiency.
     * Call loadMap() and connectShaft() after construction.
     *
     * @param PR   Fallback pressure ratio [-]
     * @param eff  Fallback isentropic efficiency [-]
     */
    Compressor(double PR, double eff) noexcept;

    /*
     * loadMap — loads a CompressorMap from file.
     * Mirrors NPSS S_map subelement pattern.
     * Once loaded, compute() uses map lookup for PR and eff.
     * If load fails, falls back to fixed PR_ and eff_ with a warning.
     *
     * @param filepath  Path to the .map file (TYPE must be COMPRESSOR)
     */
    void loadMap(const std::string& filepath);

    /*
     * connectShaft — connects this compressor to its driving shaft.
     * Mirrors NPSS shaft connection pattern.
     * Once connected, compute() reads N_rpm from shaft to compute Nc.
     * If no shaft connected, map is queried at design Nc from header.
     *
     * @param shaft  Pointer to the connected Shaft
     */
    void connectShaft(Shaft* shaft) noexcept { shaft_ = shaft; }

    /*
     * compute — performs compressor thermodynamics.
     * Implements Element::compute().
     */
    void compute() noexcept override;

    /*
     * getWork — returns negative dHt [J/kg].
     * Sign convention: negative = power consumed from shaft.
     */
    double getWork() const noexcept override { return -dHt; }

    // Total enthalpy rise [J/kg] — available after compute()
    // dHt = Cp × (Tt_exit - Tt_inlet)
    double dHt         = 0.0;

    // Corrected mass flow [kg/s] — available after compute()
    double Wc           = 0.0;

    // Corrected speed [RPM corrected] — available after compute()
    double Nc           = 0.0;

    // Surge margin [%] — available after compute() if map with surge line loaded
    // SM = (PR_surge(Wc) - PR_operating) / PR_surge(Wc) * 100
    // 0.0 if no surge line present in map file
    double surge_margin = 0.0;

    // Polytropic efficiency [-] — available after compute()
    // Diagnostic output — does not drive thermodynamics (isentropic internally)
    // eta_poly > eta_is for compressor — gap increases with PR
    // Formula: ln(PR) * (γ-1)/γ / ln(1 + (PR^((γ-1)/γ) - 1) / eta_is)
    // Reference: Mattingly Ch.5, Walsh & Fletcher Ch.3, NPSS eff_poly output
    double eta_poly     = 0.0;

    // Isentropic efficiency [-] — current operating value after compute()
    // Same as eff from map lookup, stored for output alongside eta_poly
    double eff_is = 0.0;
    
private:

    // =========================================================================
    // PRIVATE MEMBERS
    // =========================================================================

    double                        PR_;     // Fallback pressure ratio [-]
    double                        eff_;    // Fallback isentropic efficiency [-]
    std::optional<CompressorMap>  map_;    // CompressorMap subelement — present only if loadMap() called
    Shaft*                        shaft_ = nullptr;  // Connected shaft — null if not connected
};

#endif // COMPRESSOR_H