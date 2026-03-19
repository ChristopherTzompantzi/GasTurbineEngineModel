#ifndef COMPRESSOR_H
#define COMPRESSOR_H

#include "Element.h"
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
 * 2. Computes corrected mass flow Wc from inlet conditions
 * 3. Looks up PR and eff from CompressorMap at (Wc, Nc%) if map loaded
 *    Otherwise uses fixed PR_ and eff_ from constructor as fallback
 * 4. Raises total pressure by pressure ratio (PR)
 * 5. Raises total temperature accounting for isentropic efficiency (eff)
 * 6. Writes real gas Cp and gamma to flowOut via Thermo at exit conditions
 * 7. Mass flow passes through unchanged
 *
 * THERMODYNAMIC EQUATIONS (real gas via Thermo):
 *   Wc             = W * sqrt(Tt_in / T_ref) / (Pt_in / P_ref)  [kg/s corrected]
 *   Pt_exit        = Pt_inlet × PR
 *   gamma          = Thermo::getGamma(Tt_inlet, FAR)
 *   Tt_exit_ideal  = Tt_inlet × PR^((γ-1)/γ)
 *   Tt_exit        = Tt_inlet + (Tt_exit_ideal - Tt_inlet) / eff
 *   dHt            = Cp × (Tt_exit - Tt_inlet)                   [J/kg]
 *
 * CORRECTED FLOW:
 *   Wc = W * sqrt(Tt_in / 288.15) / (Pt_in / 101325.0)
 *   Reference conditions: T_ref=288.15 K, P_ref=101325 Pa (ISA sea level)
 *   Wc is the map lookup x-axis in Phase 3.
 *
 * MAP LOOKUP (Phase 3):
 *   Nc% = 100.0 — design speed line used throughout Phase 3.
 *   Phase 4 will replace 100.0 with computed shaft corrected speed.
 *
 * NPSS ALIGNMENT:
 *   loadMap() mirrors NPSS S_map subelement pattern:
 *     compressor.S_map.filename = "hp_compressor.map"
 *     compressor.S_map.load()
 *
 * ISENTROPIC EFFICIENCY:
 *   eff = 1.0 → perfect isentropic compression (no losses).
 *   Typical range: 0.85–0.92 for modern compressors.
 *
 * FUTURE WORK (Phase 4):
 *   Replace Nc%=100.0 with computed corrected shaft speed.
 *   Surge margin and stall detection.
 *   Polytropic efficiency as alternative input mode.
 *
 * UNITS:
 *   PR   [-]
 *   eff  [-]
 *   dHt  [J/kg]
 *   Wc   [kg/s corrected]
 *   Nc   [% of design corrected speed]
 */

class Compressor : public Element {
public:

    /*
     * Constructor — sets fallback pressure ratio and isentropic efficiency.
     * Call loadMap() after construction to enable map-based PR and eff.
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
     * compute — performs compressor thermodynamics.
     * Implements Element::compute().
     */
    void compute() noexcept override;

    /*
     * getWork — returns negative dHt [J/kg].
     * Sign convention: negative = power consumed from shaft.
     */
    double getWork() const noexcept override { return -dHt; }

    /*
     * setPR — sets the fallback pressure ratio directly.
     * Used by the solver when operating without a map.
     *
     * @param PR  Pressure ratio [-]
     */
    void setPR(double PR) noexcept { PR_ = PR; }

    // Total enthalpy rise [J/kg] — available after compute()
    // dHt = Cp × (Tt_exit - Tt_inlet)
    double dHt = 0.0;

private:

    // =========================================================================
    // PRIVATE MEMBERS
    // =========================================================================

    double                        PR_;    // Fallback pressure ratio [-]
    double                        eff_;   // Fallback isentropic efficiency [-]
    std::optional<CompressorMap>  map_;   // CompressorMap subelement — present only if loadMap() called
};

#endif // COMPRESSOR_H