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
 * PERFECT GAS ASSUMPTION (Phase 1):
 * gamma and Cp are treated as constants in Phase 1 (perfect gas).
 * This simplifies the thermodynamic calculations and allows
 * validation against known analytical solutions (e.g. Mattingly).
 * In Phase 2 these will become functions of temperature and FAR
 * using NASA 7-coefficient polynomials for real gas behavior.
 *
 * DEFAULT VALUES:
 * The constructor initializes all variables to ISA (International
 * Standard Atmosphere) sea level conditions. This prevents
 * uninitialized memory — a common source of silent bugs in C++
 * physics simulations where garbage values produce plausible
 * but incorrect results.
 */

struct FlowStation {
    // Thermodynamic state variables
    // Default values = ISA (International Standard Atmosphere) sea level conditions
    double Pt    = 101325.0;  // Total pressure    [Pa]      — standard sea level pressure
    double Tt    = 288.15;    // Total temperature [K]       — standard sea level temperature (15 deg C)
    double W     = 0.0;       // Mass flow rate    [kg/s]
    double MN    = 0.0;       // Mach number       [-]
    double FAR   = 0.0;       // Fuel-air ratio    [-]
    double gamma = 1.4;       // Ratio of specific heats [-] — perfect gas assumption for air (Phase 1)
    double Cp    = 1005.0;    // Specific heat at constant pressure [J/(kg·K)] — air at standard conditions

    // Constructor — all defaults are set above (in-class initializers).
    // noexcept: tells the compiler this constructor can never throw an exception,
    // which enables optimisations when FlowStation objects are used in containers.
    // = default: tells the compiler to generate this constructor automatically —
    // no custom logic needed since everything is already initialised above.
    FlowStation() noexcept = default;
};

#endif // FLOWSTATION_H