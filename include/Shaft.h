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
 * The Shaft class tracks all connected elements and computes the balance.
 *
 * SIGN CONVENTION (inherited from Element::getWork()):
 *   Turbines   → positive getWork() — deliver power TO the shaft
 *   Compressors/Fans → negative getWork() — consume power FROM the shaft
 *   Balance = sum of all getWork() values → zero at steady state
 *
 * USAGE — Single spool turbojet:
 *   Shaft hpShaft("HP Shaft");
 *   hpShaft.addElement(&compressor);
 *   hpShaft.addElement(&turbine);
 *   // after all elements have called compute():
 *   hpShaft.computeBalance();
 *   double error = hpShaft.getBalanceError();
 *
 * USAGE — Two spool turbofan:
 *   Shaft lpShaft("LP Shaft");   // Fan ←→ LP Turbine
 *   Shaft hpShaft("HP Shaft");   // HP Compressor ←→ HP Turbine
 *
 * PHASE 2:
 *   The Newton-Raphson solver will iterate turbine pressure ratios until
 *   getBalanceError() → 0 on all shafts simultaneously.
 *   RPM tracking and mechanical losses will be added at that point.
 */

class Shaft
{
public:
    // Constructor — name identifies the shaft in printed output
    explicit Shaft(const std::string& name);

    // Connect an element to this shaft
    // Call this for every compressor, fan, and turbine on this shaft
    // Order does not matter — Shaft sums all getWork() values
    void addElement(Element* element);

    // Compute shaft power balance
    // Must be called AFTER all connected elements have called compute()
    // Stores result internally — retrieve with getBalanceError()
    void computeBalance() noexcept;

    // Returns net shaft work [J/kg core air]
    // Zero means perfectly balanced — turbine work equals compressor work
    // Positive means turbine is over-powering — PR_t too high
    // Negative means turbine is under-powering — PR_t too low
    double getBalanceError() const noexcept { return balanceError; }

    // Returns balance error as a percentage of compressor work
    // More intuitive than raw J/kg for validation purposes
    double getBalanceErrorPercent() const noexcept;

    // Print shaft balance summary to stdout
    void printBalance() const noexcept;

    // Shaft name — used in printBalance() output
    const std::string& getName() const noexcept { return name; }

private:
    // name and elements initialized via constructor/default-init.
    // Doubles have in-class defaults — they are accumulators reset by computeBalance().
    std::string           name;
    std::vector<Element*> elements;   // all connected elements
    double                balanceError        = 0.0;
    double                totalDrivenWork     = 0.0;   // compressors + fans [J/kg]
    double                totalDriverWork     = 0.0;   // turbines [J/kg]
};

#endif // SHAFT_H