#ifndef THERMO_H
#define THERMO_H

#include <array>

/*
 * Thermo.h
 * --------
 * Real gas thermodynamic property functions for gas turbine cycle analysis.
 *
 * APPROACH — NASA 7-Coefficient Polynomials (JANAF):
 * Properties are computed from NASA polynomial curve fits for two species:
 *   1. Air          — working fluid upstream of combustor (FAR = 0)
 *   2. Burned gas   — combustion products of Jet-A/kerosene (FAR > 0)
 *
 * Properties of the gas mixture at any station are computed by
 * mass-fraction-weighted blending of the two species:
 *
 *   Cp_mix = (Cp_air + FAR * Cp_products) / (1.0 + FAR)
 *
 * This is Option A (two-species interpolation) — appropriate for engine
 * cycle analysis. Full species tracking (Option B / CEA) is not required
 * for thrust, shaft balance, or off-design performance analysis.
 *
 * POLYNOMIAL FORM (NASA 7-coefficient):
 *   Cp/R = a1 + a2*T + a3*T^2 + a4*T^3 + a5*T^4
 *   H/RT = a1 + a2/2*T + a3/3*T^2 + a4/4*T^3 + a5/5*T^4 + a6/T
 *
 * where R = 287.058 J/kg·K (specific gas constant for air/burned gas mixture).
 *
 * TWO TEMPERATURE RANGES:
 *   Low  range : 200 K  – 1000 K  (coefficients set 1)
 *   High range : 1000 K – 6000 K  (coefficients set 2)
 *
 * SOURCE:
 *   McBride, B.J., Gordon, S., Reno, M.A., "Coefficients for Calculating
 *   Thermodynamic and Transport Properties of Individual Species",
 *   NASA TM-4513, 1993.
 *
 * USAGE:
 *   #include "Thermo.h"
 *   double Cp    = Thermo::getCp(T, FAR);      // J/kg·K
 *   double gamma = Thermo::getGamma(T, FAR);   // [-]
 *   double H     = Thermo::getH(T, FAR);       // J/kg
 *   double T_out = Thermo::getT(H, FAR);       // K
 *
 * UNITS — SI throughout:
 *   Temperature  [K]
 *   Enthalpy     [J/kg]
 *   Cp           [J/kg·K]
 *   gamma        [-]
 */

namespace Thermo {

// =========================================================================
// NASA COEFFICIENT STRUCTURE
// =========================================================================

/*
 * NasaCoeffs — holds one set of NASA 7-coefficient polynomial data.
 * One instance covers a single species over a single temperature range.
 *
 * Polynomial:
 *   Cp/R = a[0] + a[1]*T + a[2]*T^2 + a[3]*T^3 + a[4]*T^4
 *   H/RT = a[0] + a[1]/2*T + a[2]/3*T^2 + a[3]/4*T^3 + a[4]/5*T^4 + a[5]/T
 *
 * a[6] is the entropy integration constant (not used in this implementation).
 */
struct NasaCoeffs {
    std::array<double, 7> a;   // NASA polynomial coefficients a1..a7
};

// =========================================================================
// SPECIES COEFFICIENT DATA
// =========================================================================
// Coefficient sets are defined in Thermo.cpp.
// extern declarations here allow any translation unit that includes
// Thermo.h to reference the data without causing duplicate definitions.

extern const NasaCoeffs AIR_LOW;       // Air,          200 K  – 1000 K
extern const NasaCoeffs AIR_HIGH;      // Air,          1000 K – 6000 K
extern const NasaCoeffs PROD_LOW;      // Burned gas,   200 K  – 1000 K
extern const NasaCoeffs PROD_HIGH;     // Burned gas,   1000 K – 6000 K

// =========================================================================
// TEMPERATURE RANGE CONSTANTS
// =========================================================================

constexpr double T_LOW  =  200.0;   // K — lower bound of valid range
constexpr double T_MID  = 1000.0;   // K — breakpoint between coefficient sets
constexpr double T_HIGH = 6000.0;   // K — upper bound of valid range

// =========================================================================
// PHYSICAL CONSTANTS
// =========================================================================

constexpr double R_AIR  = 287.058;   // J/kg·K — specific gas constant, air
constexpr double R_PROD = 287.058;   // J/kg·K — specific gas constant, burned gas
                                     // NOTE: R_PROD is an approximation for
                                     // Jet-A combustion products. A more
                                     // accurate value (~288.5 J/kg·K) can be
                                     // substituted in Phase 2b when full
                                     // species tracking is added.

// =========================================================================
// PUBLIC INTERFACE — called by engine elements
// =========================================================================

/*
 * getCp — mixture specific heat at constant pressure [J/kg·K]
 *
 * Evaluates NASA polynomial for air and burned gas, then blends
 * by mass fraction using FAR (fuel-air ratio).
 *
 * @param T    Total temperature [K]
 * @param FAR  Fuel-air ratio [-]  (0.0 = pure air)
 * @return     Cp [J/kg·K]
 */
double getCp(double T, double FAR) noexcept;

/*
 * getGamma — mixture ratio of specific heats [-]
 *
 * gamma = Cp / (Cp - R_mix)
 * where R_mix is the mass-fraction-weighted specific gas constant.
 *
 * @param T    Total temperature [K]
 * @param FAR  Fuel-air ratio [-]
 * @return     gamma [-]
 */
double getGamma(double T, double FAR) noexcept;

/*
 * getH — mixture specific enthalpy [J/kg]
 *
 * Integrates the NASA Cp polynomial from 0 K to T.
 * Used by the combustor to compute enthalpy rise from fuel addition,
 * and by compressor/turbine isentropic solvers.
 *
 * @param T    Temperature [K]
 * @param FAR  Fuel-air ratio [-]
 * @return     H [J/kg]
 */
double getH(double T, double FAR) noexcept;

/*
 * getT — mixture temperature from specific enthalpy [K]
 *
 * Inverse of getH(). Solves H = getH(T, FAR) for T using
 * Newton iteration. Used by the combustor to find Tt4 from
 * enthalpy addition, and by isentropic solvers to find exit
 * temperature from work input/extraction.
 *
 * Convergence: |dT| < 0.01 K, maximum 50 iterations.
 *
 * @param H    Target specific enthalpy [J/kg]
 * @param FAR  Fuel-air ratio [-]
 * @return     T [K]
 */
double getT(double H, double FAR) noexcept;

// =========================================================================
// INTERNAL HELPERS — not intended for direct use by element code
// =========================================================================

/*
 * getCpSpecies — evaluates NASA Cp polynomial for a single species [J/kg·K]
 *
 * Selects low or high range coefficients based on T, then evaluates:
 *   Cp/R = a1 + a2*T + a3*T^2 + a4*T^3 + a5*T^4
 *
 * @param T       Temperature [K]
 * @param coeffLo Coefficients for low  temperature range
 * @param coeffHi Coefficients for high temperature range
 * @param R       Specific gas constant for this species [J/kg·K]
 * @return        Cp [J/kg·K]
 */
double getCpSpecies(double T,
                    const NasaCoeffs& coeffLo,
                    const NasaCoeffs& coeffHi,
                    double R) noexcept;

/*
 * getHSpecies — evaluates NASA enthalpy polynomial for a single species [J/kg]
 *
 * Selects low or high range coefficients based on T, then evaluates:
 *   H/RT = a1 + a2/2*T + a3/3*T^2 + a4/4*T^3 + a5/5*T^4 + a6/T
 *
 * @param T       Temperature [K]
 * @param coeffLo Coefficients for low  temperature range
 * @param coeffHi Coefficients for high temperature range
 * @param R       Specific gas constant for this species [J/kg·K]
 * @return        H [J/kg]
 */
double getHSpecies(double T,
                   const NasaCoeffs& coeffLo,
                   const NasaCoeffs& coeffHi,
                   double R) noexcept;

}   // namespace Thermo

#endif // THERMO_H
