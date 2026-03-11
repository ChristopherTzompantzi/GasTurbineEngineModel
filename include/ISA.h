#ifndef ISA_H
#define ISA_H

/*
 * ISA.h
 * -----
 * Declares the ISA (International Standard Atmosphere) class.
 * Provides ambient conditions (static pressure and temperature)
 * as a function of altitude.
 *
 * STANDARD COVERED:
 * ISO 2533:1975 International Standard Atmosphere
 * Valid for altitudes from 0 to 20,000 m (0 to 65,617 ft)
 * covering all practical gas turbine operating conditions.
 *
 * TWO ATMOSPHERE LAYERS (Phase 1):
 *   Troposphere  : 0 to 11,000 m — temperature drops linearly with altitude
 *   Stratosphere : 11,000 m to 20,000 m — temperature constant at 216.65 K
 *
 * UNITS:
 * Input  — altitude in meters [m] (SI, consistent with project convention)
 * Output — static pressure in Pascal [Pa], static temperature in Kelvin [K]
 *
 * ALTITUDE INPUT DECISION:
 * Altitude is accepted in meters internally.
 * A helper function is provided to convert feet to meters since
 * gas turbine performance data is commonly expressed in feet.
 * All internal calculations remain in SI.
 *
 * PHASE 2 CONSIDERATION:
 * This model uses standard day conditions only.
 * Hot day, cold day, and tropical atmosphere variants
 * (commonly used in engine certification) can be added in Phase 2
 * by extending this class with delta-temperature offsets.
 */

class ISA {
public:
    // Computes static temperature [K] at given altitude [m]
    static double getStaticTemperature(double altitude_m) noexcept;

    // Computes static pressure [Pa] at given altitude [m]
    static double getStaticPressure(double altitude_m) noexcept;

    // Computes total temperature [K] from static temperature and Mach number
    static double getTotalTemperature(double Ts, double MN, double gamma) noexcept;

    // Computes total pressure [Pa] from static pressure and Mach number
    static double getTotalPressure(double Ps, double MN, double gamma) noexcept;

    // Helper — converts altitude in feet to meters
    static double feetToMeters(double altitude_ft) noexcept;
};

#endif // ISA_H