#include "Shaft.h"
#include <iostream>
#include <iomanip>
#include <cmath>

/*
 * Shaft.cpp
 * ---------
 * Implements the Shaft mechanical element.
 *
 * COMPUTE SEQUENCE:
 * 1. Iterate over all connected elements
 * 2. Call getWork() on each — positive for turbines, negative for compressors
 * 3. Accumulate driver work (turbines) and driven work (compressors/fans)
 * 4. Balance error = total driver work + total driven work
 *    (driven work is already negative, so a balanced shaft sums to zero)
 *
 * WHY THIS WORKS:
 * Every element connected to the shaft reports its work via getWork().
 * Turbines return positive values — they put power into the shaft.
 * Compressors and fans return negative values — they take power out.
 * At steady state the sum is exactly zero.
 * The solver (Phase 2) will iterate turbine PR until this condition is met.
 */

Shaft::Shaft(const std::string& name)
    : name(name)
{}

void Shaft::addElement(Element* element)
{
    elements.push_back(element);
}

void Shaft::computeBalance() noexcept
{
    // Reset accumulators before each balance computation
    totalDriverWork = 0.0;
    totalDrivenWork = 0.0;

    for (auto* e : elements)
    {
        double work = e->getWork();

        if (work > 0.0)
            totalDriverWork += work;    // turbine — delivers power
        else
            totalDrivenWork += work;    // compressor/fan — consumes power
    }

    // Balance error — zero at steady state
    // Positive: turbine over-powering (PR_t too high)
    // Negative: turbine under-powering (PR_t too low)
    balanceError = totalDriverWork + totalDrivenWork;
}

double Shaft::getBalanceErrorPercent() const noexcept
{
    // Express balance error as percentage of driven work magnitude
    // Driven work is negative — take absolute value for percentage
    if (std::abs(totalDrivenWork) < 1e-10)
        return 0.0;

    return (balanceError / std::abs(totalDrivenWork)) * 100.0;
}

void Shaft::printBalance() const noexcept
{
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "--- Shaft: " << name << " ---\n";
    std::cout << "  Driver work (turbines)    = "
              << totalDriverWork << " J/kg\n";
    std::cout << "  Driven work (compressors) = "
              << totalDrivenWork << " J/kg\n";
    std::cout << "  Balance error             = "
              << balanceError << " J/kg"
              << "  (" << getBalanceErrorPercent() << " %)\n";
    std::cout << "\n";
}