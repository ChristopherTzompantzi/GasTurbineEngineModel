#include "ISA.h"
#include <cmath>

/*
 * ISA.cpp
 * -------
 * Implements the ISA (International Standard Atmosphere) model.
 * All equations follow ISO 2533:1975.
 *
 * CONSTANTS:
 *   T0    = 288.15  K    — sea level standard temperature
 *   P0    = 101325  Pa   — sea level standard pressure
 *   L     = 0.0065  K/m  — temperature lapse rate (troposphere)
 *   H_trop= 11000   m    — tropopause altitude
 *   T_trop= 216.65  K    — temperature at tropopause
 *   g     = 9.80665 m/s² — standard gravity
 *   R     = 287.058 J/(kg·K) — specific gas constant for air
 *
 * EQUATIONS USED:
 * Troposphere (0 to 11,000 m):
 *   Ts = T0 - L * altitude
 *   Ps = P0 * (Ts / T0)^(g / (R * L))
 *
 * Stratosphere (11,000 m to 20,000 m):
 *   Ts = T_trop (constant)
 *   Ps = P_trop * exp(-g * (altitude - H_trop) / (R * T_trop))
 *
 * Ram compression (from isentropic relations):
 *   Tt = Ts * (1 + (gamma-1)/2 * M²)
 *   Pt = Ps * (1 + (gamma-1)/2 * M²)^(gamma/(gamma-1))
 */

// ISA constants
static const double T0     = 288.15;   // Sea level temperature [K]
static const double P0     = 101325.0; // Sea level pressure [Pa]
static const double L      = 0.0065;   // Lapse rate [K/m]
static const double H_trop = 11000.0;  // Tropopause altitude [m]
static const double T_trop = 216.65;   // Tropopause temperature [K]
static const double g      = 9.80665;  // Standard gravity [m/s²]
static const double R      = 287.058;  // Gas constant for air [J/kg·K]

// Pressure at tropopause — computed once from troposphere equation
static const double P_trop = P0 * std::pow(T_trop / T0, g / (R * L));

double ISA::getStaticTemperature(double altitude_m) noexcept
{
    if (altitude_m <= H_trop) {
        // Troposphere — linear temperature drop
        return T0 - L * altitude_m;
    } else {
        // Stratosphere — constant temperature
        return T_trop;
    }
}

double ISA::getStaticPressure(double altitude_m) noexcept
{
    if (altitude_m <= H_trop) {
        // Troposphere — pressure follows temperature ratio
        double Ts = getStaticTemperature(altitude_m);
        return P0 * std::pow(Ts / T0, g / (R * L));
    } else {
        // Stratosphere — exponential pressure decay
        return P_trop * std::exp(-g * (altitude_m - H_trop) / (R * T_trop));
    }
}

double ISA::getTotalTemperature(double Ts, double MN, double gamma) noexcept
{
    // Isentropic relation — ram temperature rise
    // Tt = Ts * (1 + (gamma-1)/2 * MN²)
    // Phase 2: add input validation (MN >= 0, gamma > 1)
    double tau = 1.0 + (gamma - 1.0) / 2.0 * MN * MN;
    return Ts * tau;
}

double ISA::getTotalPressure(double Ps, double MN, double gamma) noexcept
{
    // Isentropic relation — ram pressure rise
    // Pt = Ps * (1 + (gamma-1)/2 * MN²)^(gamma/(gamma-1))
    // Phase 2: add input validation (MN >= 0, gamma > 1)
    double exponent = gamma / (gamma - 1.0);
    double base     = 1.0 + (gamma - 1.0) / 2.0 * MN * MN;
    return Ps * std::pow(base, exponent);
}

double ISA::feetToMeters(double altitude_ft) noexcept
{
    // 1 foot = 0.3048 meters exactly (international definition)
    return altitude_ft * 0.3048;
}