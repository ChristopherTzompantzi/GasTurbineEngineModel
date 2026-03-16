#ifndef FLOWSTATION_H
#define FLOWSTATION_H

/*
 * FlowStation.h
 * -------------
 * Defines the FlowStation struct — the core data container of the
 * GasTurbineEngineModel. A FlowStation represents the thermodynamic
 * state of the gas at any point in the engine cycle, inspired by
 * the NPSS element-based architecture.
 *
 * UNITS DECISION — SI UNITS INTERNALLY:
 * All variables are stored and calculated in SI units (Pa, K, kg/s).
 * This is critical for two reasons:
 *   1. All thermodynamic equations are derived in SI — using mixed
 *      units inside calculations produces silently wrong results
 *      with no error or warning from the compiler.
 *   2. Consistency — every element (Compressor, Turbine, etc.) can
 *      read and write FlowStation variables without unit conversion.
 * Output conversion to engineering units (bar, degC, lbf) happens
 * only at the results/output layer — never inside physics calculations.
 *
 * REAL GAS PROPERTIES (Phase 2):
 * gamma and Cp are no longer hardcoded constants. Each element is
 * responsible for writing the correct values to its flowOut station
 * after computing the new Tt and FAR, using:
 *
 *   flowOut.Cp    = Thermo::getCp(flowOut.Tt, flowOut.FAR);
 *   flowOut.gamma = Thermo::getGamma(flowOut.Tt, flowOut.FAR);
 *
 * This makes the data flow explicit — each element owns its output
 * station properties, consistent with the NPSS element-based pattern.
 * gamma and Cp carried on a FlowStation always reflect the actual
 * thermodynamic state at that station, not a global constant.
 *
 * DEFAULT VALUES:
 * The constructor initializes all variables to ISA (International
 * Standard Atmosphere) sea level conditions. This prevents
 * uninitialized memory — a common source of silent bugs in C++
 * physics simulations where garbage values produce plausible
 * but incorrect results.
 *
 * gamma and Cp are initialised to 0.0. Any element that reads these
 * values without first computing them via Thermo will produce an
 * obviously wrong result — a deliberate fail-loud sentinel rather
 * than a silent wrong answer from a stale hardcoded constant.
 */

struct FlowStation {
    // Thermodynamic state variables
    // Default values = ISA (International Standard Atmosphere) sea level conditions
    double Pt    = 101325.0;  // Total pressure    [Pa]      — standard sea level pressure
    double Tt    = 288.15;    // Total temperature [K]       — standard sea level temperature (15 degC)
    double W     = 0.0;       // Mass flow rate    [kg/s]
    double MN    = 0.0;       // Mach number       [-]
    double FAR   = 0.0;       // Fuel-air ratio    [-]

    // Real gas properties — set by each element via Thermo::getCp / Thermo::getGamma
    // after computing the new Tt and FAR. Initialised to 0.0 as a fail-loud sentinel:
    // any element that reads these without computing them first will produce an
    // obviously wrong result rather than a silent wrong answer.
    double Cp    = 0.0;       // Specific heat at constant pressure [J/kg·K]
    double gamma = 0.0;       // Ratio of specific heats            [-]

    // Constructor — all defaults are set above (in-class initializers).
    // noexcept: tells the compiler this constructor can never throw an exception,
    // which enables optimisations when FlowStation objects are used in containers.
    // = default: tells the compiler to generate this constructor automatically —
    // no custom logic needed since everything is already initialised above.
    FlowStation() noexcept = default;
};

#endif // FLOWSTATION_H