#ifndef SHAFT_H
#define SHAFT_H

#include <vector>
#include <string>
#include "Element.h"

/*
 * Shaft.h
 * -------
 * Models a mechanical shaft connecting turbine(s) to compressor(s)/fan(s).
 *
 * WHAT THE SHAFT DOES:
 * At steady state the net power on the shaft must be zero — power delivered
 * by turbines must exactly equal power consumed by compressors and fans.
 *
 * SHAFT SPEED (Phase 4):
 * N_rpm is the current physical shaft speed [RPM] — set by the solver on
 * each iteration. N_design_rpm is the design point shaft speed [RPM] set
 * at construction. Elements read N_rpm from their connected shaft to compute
 * corrected speed Nc for map lookup:
 *   Nc = N_rpm / sqrt(Tt_in / T_ref)   [RPM corrected]
 *
 * SIGN CONVENTION:
 *   Turbines            → positive getWork() — deliver power TO shaft
 *   Compressors/Fans    → negative getWork() — consume power FROM shaft
 *   Balance = sum of all getWork() → zero at steady state
 *
 * NPSS ALIGNMENT:
 *   Mirrors NPSS Shaft element — Nmech is the physical shaft speed [RPM]
 *   iterated by the solver. NmechDes is the design speed reference.
 *
 * UNITS:
 *   N_rpm        [RPM]
 *   N_design_rpm [RPM]
 *   balanceError [J/kg]
 */

class Shaft {
public:

    /*
     * Constructor — name identifies shaft in output.
     * N_design_rpm sets the reference design speed.
     * Default 0.0 preserves backward compatibility — Phase 3 shafts
     * do not use speed tracking.
     *
     * @param name          Shaft identifier string
     * @param N_design_rpm  Design shaft speed [RPM] (default 0.0)
     */
    explicit Shaft(const std::string& name,
                   double             N_design_rpm = 0.0) noexcept;

    /*
     * addElement — connects an element to this shaft.
     * Call for every compressor, fan, and turbine on this shaft.
     */
    void addElement(Element* element);

    /*
     * setSpeed — sets current physical shaft speed [RPM].
     * Called by the solver on each Newton iteration.
     *
     * @param N_rpm  Physical shaft speed [RPM]
     */
    void setSpeed(double N_rpm) noexcept { N_rpm_ = N_rpm; }

    /*
     * getSpeed — returns current physical shaft speed [RPM].
     * Read by Compressor, Fan, Turbine to compute corrected speed.
     */
    double getSpeed() const noexcept { return N_rpm_; }

    /*
     * getDesignSpeed — returns design shaft speed [RPM].
     */
    double getDesignSpeed() const noexcept { return N_design_rpm_; }

    /*
     * computeBalance — computes shaft power balance.
     * Must be called AFTER all connected elements have called compute().
     */
    void computeBalance() noexcept;

    /*
     * getBalanceError — returns net shaft work [J/kg].
     * Zero = balanced. Positive = over-powered. Negative = under-powered.
     */
    double getBalanceError() const noexcept { return balanceError_; }

    /*
     * getBalanceErrorPercent — balance error as % of compressor work.
     */
    double getBalanceErrorPercent() const noexcept;

    /*
     * printBalance — prints shaft balance summary to stdout.
     */
    void printBalance() const noexcept;

    /*
     * getName — returns shaft name.
     */
    const std::string& getName() const noexcept { return name_; }

private:

    // =========================================================================
    // PRIVATE MEMBERS
    // =========================================================================

    std::string           name_;            // Shaft identifier
    std::vector<Element*> elements_;        // Connected elements
    double                N_rpm_        = 0.0;   // Current shaft speed [RPM]
    double                N_design_rpm_ = 0.0;   // Design shaft speed [RPM]
    double                balanceError_  = 0.0;   // Net shaft work [J/kg]
    double                totalDrivenWork_ = 0.0;  // Compressors + fans [J/kg]
    double                totalDriverWork_ = 0.0;  // Turbines [J/kg]
};

#endif // SHAFT_H