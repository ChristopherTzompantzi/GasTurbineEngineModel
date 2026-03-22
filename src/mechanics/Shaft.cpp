#include "Shaft.h"
#include <iostream>
#include <iomanip>
#include <cmath>

/*
 * Shaft.cpp
 * ---------
 * Implements the Shaft mechanical element.
 *
 * COMPUTE SEQUENCE (computeBalance):
 * 1. Reset accumulators
 * 2. Iterate over all connected elements — call getWork()
 * 3. Accumulate driver work (turbines, positive) and
 *    driven work (compressors/fans, negative)
 * 4. balanceError_ = totalDriverWork_ + totalDrivenWork_
 *    Zero at steady state — solver drives this to zero.
 *
 * SHAFT SPEED:
 * N_rpm_ is set externally by the solver via setSpeed() on each
 * Newton iteration. Elements read it via getSpeed() to compute
 * corrected speed for map lookup.
 */

// =========================================================================
// Constructor
// =========================================================================

Shaft::Shaft(const std::string& name, double N_design_rpm) noexcept
    : name_        (name)
    , N_rpm_       (N_design_rpm)   // initialise to design speed
    , N_design_rpm_(N_design_rpm)
{}

// =========================================================================
// addElement
//
// Connects an element to this shaft.
// Order does not matter — computeBalance() sums all getWork() values.
// =========================================================================

void Shaft::addElement(Element* element)
{
    elements_.push_back(element);
}

// =========================================================================
// computeBalance
//
// Sums getWork() across all connected elements.
// Positive = turbine over-powering, Negative = under-powering.
// =========================================================================

void Shaft::computeBalance() noexcept
{
    totalDriverWork_ = 0.0;
    totalDrivenWork_ = 0.0;

    for (auto* e : elements_)
    {
        const double work = e->getWork();
        if (work > 0.0)
            totalDriverWork_ += work;
        else
            totalDrivenWork_ += work;
    }

    balanceError_ = totalDriverWork_ + totalDrivenWork_;
}

// =========================================================================
// getBalanceErrorPercent
//
// Returns balance error as percentage of driven work magnitude.
// More intuitive than raw J/kg for validation and printed output.
// =========================================================================

double Shaft::getBalanceErrorPercent() const noexcept
{
    if (std::abs(totalDrivenWork_) < 1.0e-10)
        return 0.0;
    return (balanceError_ / std::abs(totalDrivenWork_)) * 100.0;
}

// =========================================================================
// printBalance
//
// Prints shaft power balance summary to stdout.
// Includes shaft speed line if N_design_rpm_ > 0.0.
// =========================================================================

void Shaft::printBalance() const noexcept
{
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "--- Shaft: " << name_ << " ---\n";
    std::cout << "  Driver work (turbines)    = "
              << totalDriverWork_ << " J/kg\n";
    std::cout << "  Driven work (compressors) = "
              << totalDrivenWork_ << " J/kg\n";
    std::cout << "  Balance error             = "
              << balanceError_ << " J/kg"
              << "  (" << getBalanceErrorPercent() << " %)\n";
    if (N_design_rpm_ > 0.0)
    {
        std::cout << "  Shaft speed               = "
                  << N_rpm_ << " RPM"
                  << "  (" << (N_rpm_ / N_design_rpm_ * 100.0)
                  << " % design)\n";
    }
    std::cout << "\n";
}