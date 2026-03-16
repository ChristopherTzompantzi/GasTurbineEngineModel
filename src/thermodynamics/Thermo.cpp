#include "Thermo.h"

#include <cmath>

/*
 * Thermo.cpp
 * ----------
 * Implements real gas thermodynamic property functions using NASA
 * 7-coefficient polynomials (JANAF database).
 *
 * Two species are modelled:
 *   1. Air          — FAR = 0  (upstream of combustor)
 *   2. Burned gas   — FAR > 0  (Jet-A combustion products)
 *
 * Mixture properties are computed by mass-fraction-weighted blending:
 *   Cp_mix = (Cp_air + FAR * Cp_products) / (1.0 + FAR)
 *   H_mix  = (H_air  + FAR * H_products)  / (1.0 + FAR)
 *   R_mix  = (R_AIR  + FAR * R_PROD)      / (1.0 + FAR)
 *
 * NASA POLYNOMIAL FORM:
 *   Cp/R = a1 + a2*T + a3*T^2 + a4*T^3 + a5*T^4
 *   H/RT = a1 + a2/2*T + a3/3*T^2 + a4/4*T^3 + a5/5*T^4 + a6/T
 *
 * SOURCE:
 *   McBride, B.J., Gordon, S., Reno, M.A., "Coefficients for Calculating
 *   Thermodynamic and Transport Properties of Individual Species",
 *   NASA TM-4513, 1993.
 *
 *   Air coefficients: standard dry air mixture fit (N2 ~78%, O2 ~21%, Ar ~1%).
 *   Burned gas coefficients: mass-fraction-weighted blend of CO2, H2O, N2, O2
 *   at cruise FAR (~0.025). Product gas mole composition: CO2=8.25%,
 *   H2O=7.92%, N2=72.90%, O2=10.93%. Blended from individual species
 *   NASA TM-4513 data using mass fractions.
 */

namespace Thermo {

// =========================================================================
// NASA COEFFICIENT DATA
// =========================================================================
// Layout: { a1, a2, a3, a4, a5, a6, a7 }
//   a1..a5 — Cp polynomial coefficients
//   a6     — enthalpy integration constant
//   a7     — entropy integration constant (not used)
//
// Units: T in Kelvin, Cp in J/kg·K when multiplied by R [J/kg·K].

// -------------------------------------------------------------------------
// Air — dry air mixture (N2 ~78%, O2 ~21%, Ar ~1%)
// Source: NASA TM-4513, standard dry air mixture fit
// -------------------------------------------------------------------------

// Low range: 200 K – 1000 K
// Source: NASA RP-1311 (McBride, Zehe, Gordon 1994) — dry air mixture
const NasaCoeffs AIR_LOW = {{
     3.57953347e+00,   // a1
    -6.10353680e-04,   // a2
     1.01681440e-06,   // a3
     9.07005884e-10,   // a4
    -9.04424499e-13,   // a5
    -1.04797753e+03,   // a6 — enthalpy integration constant
     0.00000000e+00    // a7 — entropy integration constant (unused)
}};

// High range: 1000 K – 6000 K
// Source: NASA RP-1311 (McBride, Zehe, Gordon 1994) — dry air mixture
const NasaCoeffs AIR_HIGH = {{
     3.08792717e+00,   // a1
     1.24597184e-03,   // a2
    -4.23718945e-07,   // a3
     6.74774789e-11,   // a4
    -3.97076709e-15,   // a5
    -9.95262101e+02,   // a6
     0.00000000e+00    // a7 (unused)
}};

// -------------------------------------------------------------------------
// Burned gas — Jet-A combustion products
// Mass-fraction-weighted blend of CO2, H2O, N2, O2
// Mole fractions: CO2=0.0825, H2O=0.0792, N2=0.7290, O2=0.1093
// Source: NASA TM-4513, individual species blended by mass fraction
// -------------------------------------------------------------------------

// Low range: 200 K – 1000 K
const NasaCoeffs PROD_LOW = {{
     3.51449148e+00,   // a1
     1.62251751e-04,   // a2
     6.38345444e-07,   // a3
     4.85408529e-10,   // a4
    -5.43999235e-13,   // a5
    -7.26948539e+03,   // a6
     0.00000000e+00    // a7 — entropy constant not normalised for blended species (unused)
}};

// High range: 1000 K – 6000 K
const NasaCoeffs PROD_HIGH = {{
     3.14711159e+00,   // a1
     1.55171628e-03,   // a2
    -5.17994555e-07,   // a3
     8.02594441e-11,   // a4
    -4.59479239e-15,   // a5
    -7.21798473e+03,   // a6
     0.00000000e+00    // a7 — entropy constant not normalised for blended species (unused)
}};

// =========================================================================
// INTERNAL HELPERS
// =========================================================================

double getCpSpecies(double T,
                    const NasaCoeffs& coeffLo,
                    const NasaCoeffs& coeffHi,
                    double R) noexcept
{
    // Select coefficient set based on temperature range
    const NasaCoeffs& c = (T < T_MID) ? coeffLo : coeffHi;

    // Evaluate NASA Cp polynomial
    // Cp/R = a1 + a2*T + a3*T^2 + a4*T^3 + a5*T^4
    const double CpR = c.a[0]
                     + c.a[1] * T
                     + c.a[2] * T * T
                     + c.a[3] * T * T * T
                     + c.a[4] * T * T * T * T;

    return CpR * R;   // [J/kg·K]
}

double getHSpecies(double T,
                   const NasaCoeffs& coeffLo,
                   const NasaCoeffs& coeffHi,
                   double R) noexcept
{
    // Select coefficient set based on temperature range
    const NasaCoeffs& c = (T < T_MID) ? coeffLo : coeffHi;

    // Evaluate NASA enthalpy polynomial
    // H/RT = a1 + a2/2*T + a3/3*T^2 + a4/4*T^3 + a5/5*T^4 + a6/T
    const double HRT = c.a[0]
                     + c.a[1] / 2.0 * T
                     + c.a[2] / 3.0 * T * T
                     + c.a[3] / 4.0 * T * T * T
                     + c.a[4] / 5.0 * T * T * T * T
                     + c.a[5] / T;

    return HRT * R * T;   // [J/kg]
}

// =========================================================================
// PUBLIC INTERFACE
// =========================================================================

double getCp(double T, double FAR) noexcept
{
    const double Cp_air  = getCpSpecies(T, AIR_LOW,  AIR_HIGH,  R_AIR);
    const double Cp_prod = getCpSpecies(T, PROD_LOW, PROD_HIGH, R_PROD);

    // Mass-fraction-weighted blend
    // Cp_mix = (Cp_air + FAR * Cp_products) / (1.0 + FAR)
    return (Cp_air + FAR * Cp_prod) / (1.0 + FAR);
}

double getGamma(double T, double FAR) noexcept
{
    const double Cp    = getCp(T, FAR);

    // Mass-fraction-weighted specific gas constant
    const double R_mix = (R_AIR + FAR * R_PROD) / (1.0 + FAR);

    // gamma = Cp / (Cp - R_mix)
    return Cp / (Cp - R_mix);
}

double getH(double T, double FAR) noexcept
{
    const double H_air  = getHSpecies(T, AIR_LOW,  AIR_HIGH,  R_AIR);
    const double H_prod = getHSpecies(T, PROD_LOW, PROD_HIGH, R_PROD);

    // Mass-fraction-weighted blend
    return (H_air + FAR * H_prod) / (1.0 + FAR);
}

double getT(double H, double FAR) noexcept
{
    // Newton iteration — solves getH(T, FAR) = H for T
    //
    // Update rule:
    //   T_new = T - (getH(T, FAR) - H) / getCp(T, FAR)
    //
    // Initial guess: T0 = H / Cp_ref, Cp_ref = 1100 J/kg·K
    // 1100 is midpoint between cold air (1005) and hot products (~1250).

    constexpr double Cp_ref   = 1100.0;   // J/kg·K — initial guess scaling
    constexpr double tol      =   0.01;   // K      — convergence tolerance
    constexpr int    max_iter =     50;   // maximum Newton iterations

    double T = H / Cp_ref;

    if (T < T_LOW)  T = T_LOW;
    if (T > T_HIGH) T = T_HIGH;

    for (int i = 0; i < max_iter; ++i) {
        const double H_T  = getH(T, FAR);
        const double Cp_T = getCp(T, FAR);
        const double dT   = (H_T - H) / Cp_T;

        T -= dT;

        if (T < T_LOW)  T = T_LOW;
        if (T > T_HIGH) T = T_HIGH;

        if (std::abs(dT) < tol) break;
    }

    return T;
}

}   // namespace Thermo