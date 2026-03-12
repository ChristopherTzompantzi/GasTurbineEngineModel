#ifndef ELEMENT_H
#define ELEMENT_H

#include "FlowStation.h"

/*
 * Element.h
 * ---------
 * Abstract base class for all gas turbine engine elements.
 *
 * Every engine element (Inlet, Compressor, Combustor, Turbine, Nozzle,
 * Fan, Afterburner, ...) inherits from Element and implements compute().
 * Elements that exchange shaft work (Compressor, Turbine, Fan) override
 * getWork() — all others inherit the default return of 0.0.
 *
 * WORK SIGN CONVENTION:
 *   Positive getWork() = power delivered TO the shaft (turbines)
 *   Negative getWork() = power consumed FROM the shaft (compressors, fans)
 *   Zero     getWork() = no shaft interaction (inlet, combustor, nozzle)
 *
 * PHASE 2:
 *   flowIn/flowOut may become private with accessors for invariant checking.
 *   getWork() will feed directly into the Newton-Raphson solver via Shaft.
 */

class Element
{
public:
    FlowStation flowIn;
    FlowStation flowOut;

    // Pure virtual — every element must implement its thermodynamic cycle
    virtual void compute() noexcept = 0;

    // Default implementation returns 0.0 — no shaft interaction.
    // Override in elements that exchange work with the shaft (Compressor, Turbine, Fan).
    // Units: J/kg of core air mass flow
    // Convention: positive = power into shaft, negative = power out of shaft
    virtual double getWork() const noexcept { return 0.0; }

    virtual ~Element() noexcept = default;
};

#endif // ELEMENT_H
