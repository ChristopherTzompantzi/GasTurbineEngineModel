#ifndef ELEMENT_H
#define ELEMENT_H

#include "FlowStation.h"

/*
 * Element.h
 * ---------
 * Defines the abstract base class for all engine elements in the
 * GasTurbineEngineModel, inspired by the NPSS element-based architecture.
 *
 * Every engine component (Inlet, Compressor, Combustor, Turbine, Nozzle)
 * inherits from this class and must implement the compute() function
 * with its own thermodynamic calculations.
 *
 * DESIGN DECISION — ABSTRACT BASE CLASS:
 * Element is intentionally abstract (has pure virtual functions).
 * This means you cannot create a plain Element object — only specific
 * components like Compressor or Turbine can be instantiated.
 * This enforces the contract that every component MUST implement
 * its own compute() function — the compiler will reject any component
 * that doesn't.
 *
 * FLOWSTATION CONNECTIONS:
 * Every element owns two FlowStation objects:
 *   - flowIn  : the thermodynamic state of gas entering the element
 *   - flowOut : the thermodynamic state of gas leaving the element
 * The compute() function reads from flowIn and writes results to flowOut.
 * This mirrors how NPSS connects elements through flow ports.
 *
 * VIRTUAL DESTRUCTOR:
 * The destructor is virtual to ensure that when a derived class object
 * (e.g. Compressor) is deleted through a base class pointer (Element*),
 * the correct destructor is called. Without this, deleting through a
 * base pointer causes undefined behavior — a common C++ bug.
 */

class Element {
public:
    // Access pattern: derived classes write to flowIn/flowOut inside compute();
    // external code (e.g. the solver) only reads from them.
    // Phase 2 consideration: if invariant checking is needed (e.g. no negative
    // pressure), these can be made private with controlled accessor functions.
    FlowStation flowIn;   // Thermodynamic state at element inlet
    FlowStation flowOut;  // Thermodynamic state at element outlet

    // Pure virtual function — every derived element MUST implement this
    // This is where each component performs its thermodynamic calculations
    virtual void compute() = 0;

    // Virtual destructor — required for safe deletion of derived classes.
    // noexcept: destructors must never throw exceptions — marking it explicit
    // is a promise to the compiler that allows optimisations in containers.
    // Copy/move policy: compiler-generated copies are safe for Phase 1 because
    // FlowStation contains only doubles (no raw pointers or dynamic memory).
    // Revisit if derived classes add heap-allocated resources.
    virtual ~Element() noexcept = default;
};

#endif // ELEMENT_H