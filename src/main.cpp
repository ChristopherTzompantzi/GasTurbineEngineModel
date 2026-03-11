#include <iostream>
#include <iomanip>
#include <cmath>

#include "Inlet.h"
#include "Compressor.h"
#include "Combustor.h"
#include "Turbine.h"
#include "Nozzle.h"
#include "ISA.h"

/*
 * main.cpp
 * --------
 * Entry point for the GasTurbineEngineModel simulation.
 * Assembles and runs a single-spool turbojet at cruise conditions.
 *
 * PHASE 1a — DESIGN POINT VALIDATION:
 * Parameters are hardcoded for validation against Mattingly
 * "Elements of Gas Turbine Propulsion" turbojet example.
 * Once validated, these parameters will be replaced by a
 * YAML context file reader in Phase 2.
 *
 * ENGINE CONFIGURATION — Single Spool Turbojet:
 *   Inlet → Compressor → Combustor → Turbine → Nozzle
 *
 * DESIGN POINT CONDITIONS:
 * Source: Mattingly, J.D., "Elements of Gas Turbine Propulsion",
 *         McGraw-Hill, 1996. Chapter 5, Example 5.1 — Turbojet cycle.
 *         Flight conditions follow ISO 2533:1975 ISA standard day.
 *
 *   Altitude  : 10,668 m (35,000 ft) — ISA standard cruise altitude
 *   Mach      : 0.85                 — typical subsonic cruise
 *   W         : 20.0 kg/s            — representative small turbojet
 *   PR_c      : 10.0                 — compressor overall pressure ratio
 *   eff_c     : 0.87                 — compressor isentropic efficiency
 *   Tt4       : 1400.0 K             — turbine inlet temperature
 *   PR_t      : 3.5                  — turbine pressure ratio (initial estimate)
 *   eff_t     : 0.89                 — turbine isentropic efficiency
 *
 * NOTE: PR_t = 3.5 is an initial estimate — the solver (Phase 2)
 * will iterate PR_t to satisfy the shaft power balance exactly.
 * Current balance error is reported at runtime for reference.
 *
 * VALIDATION TARGET:
 *   Results will be compared against Mattingly textbook example.
 *   Acceptable tolerance: within 2% on thrust and SFC.
 *
 * UNITS — SI INTERNALLY:
 *   Pressures    [Pa]
 *   Temperatures [K]
 *   Mass flow    [kg/s]
 *   Thrust       [N]
 *   SFC          [kg/N/s]
 */

// Helper function — prints full FlowStation conditions at each engine station
// Static conditions (Ps, Ts) are derived from total conditions and Mach number
// using isentropic relations (perfect gas assumption — Phase 1)
void printStation(const std::string& name,
                  const FlowStation& fs) noexcept
{
    // Derive static conditions from total conditions and Mach number
    // Ps = Pt / (1 + (γ-1)/2 × MN²)^(γ/(γ-1))
    // Ts = Tt / (1 + (γ-1)/2 × MN²)
    double mach_term = 1.0 + (fs.gamma - 1.0) / 2.0 * fs.MN * fs.MN;
    double Ps = fs.Pt / std::pow(mach_term, fs.gamma / (fs.gamma - 1.0));
    double Ts = fs.Tt / mach_term;

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "--- " << name << " ---\n";
    std::cout << "  Pt     = " << fs.Pt / 1000.0   << " kPa"
              << "   (" << fs.Pt / 100000.0         << " bar)\n";
    std::cout << "  Ps     = " << Ps / 1000.0       << " kPa"
              << "   (" << Ps / 100000.0             << " bar)\n";
    std::cout << "  Tt     = " << fs.Tt             << " K"
              << "   (" << fs.Tt - 273.15            << " degC)\n";
    std::cout << "  Ts     = " << Ts                << " K"
              << "   (" << Ts - 273.15               << " degC)\n";
    std::cout << "  W      = " << fs.W              << " kg/s\n";
    std::cout << "  MN     = " << fs.MN             << "\n";
    std::cout << "  FAR    = " << fs.FAR            << "\n";
    std::cout << "  gamma  = " << fs.gamma          << "\n";
    std::cout << "  Cp     = " << fs.Cp             << " J/kg·K\n";
    std::cout << "\n";
}

int main()
{
    std::cout << "========================================\n";
    std::cout << "  GasTurbineEngineModel — Phase 1a\n";
    std::cout << "  Single Spool Turbojet — Design Point\n";
    std::cout << "========================================\n\n";

    // --- Flight Conditions ---
    double altitude_ft = 35000.0;
    double altitude_m  = ISA::feetToMeters(altitude_ft);
    double MN          = 0.85;

    std::cout << "Flight Conditions:\n";
    std::cout << "  Altitude = " << altitude_ft << " ft ("
              << std::fixed << std::setprecision(1)
              << altitude_m << " m)\n";
    std::cout << "  Mach     = " << MN << "\n\n";

    // --- Assemble Engine Elements ---

    // Inlet — flight inlet with typical recovery
    Inlet inlet(altitude_m, MN, 0.97);

    // Compressor — PR=10, isentropic efficiency=0.87
    Compressor compressor(10.0, 0.87);

    // Combustor — Tt4 mode, turbine inlet temperature = 1400 K
    // Second argument: Tt4 [K], third argument: FAR_in (unused in Tt4 mode)
    Combustor combustor(CombustorMode::Tt4, 1400.0, 0.0, 0.99, 0.04);

    // Turbine — PR=3.5 (initial estimate), isentropic efficiency=0.89
    // NOTE: PR_t will be iterated by solver in Phase 2
    Turbine turbine(3.5, 0.89);

    // Nozzle — convergent, typical efficiency
    Nozzle nozzle(altitude_m, 0.98);

    // --- Set Inlet Mass Flow ---
    // W is set on flowIn before compute() is called
    // The solver will iterate this in Phase 2
    inlet.flowIn.W = 20.0;   // kg/s

    // --- Run Engine Cycle ---
    // Each element computes its thermodynamics and passes
    // flowOut to the next element's flowIn

    // Station 2 — Inlet exit / Compressor inlet face
    inlet.compute();
    printStation("Station 2 — Inlet Exit", inlet.flowOut);

    // Station 3 — Compressor exit / Combustor inlet
    compressor.flowIn = inlet.flowOut;
    compressor.compute();
    printStation("Station 3 — Compressor Exit", compressor.flowOut);

    // Station 4 — Combustor exit / Turbine inlet (Tt4)
    combustor.flowIn = compressor.flowOut;
    combustor.compute();
    printStation("Station 4 — Turbine Inlet (Tt4)", combustor.flowOut);

    // Station 5 — Turbine exit / Nozzle inlet
    turbine.flowIn = combustor.flowOut;
    turbine.compute();
    printStation("Station 5 — Turbine Exit", turbine.flowOut);

    // Station 9 — Nozzle exit
    nozzle.flowIn = turbine.flowOut;
    nozzle.compute();
    printStation("Station 9 — Nozzle Exit", nozzle.flowOut);

    // --- Key Performance Parameters ---
    double Fg  = nozzle.Fg;
    double W   = inlet.flowIn.W;
    double FAR = combustor.flowOut.FAR;
    double Wf  = W * FAR;           // Fuel mass flow [kg/s]
    double TSFC = Wf / Fg;          // Thrust-specific fuel consumption [kg/N/s]

    // Shaft power balance check
    // Turbine work must equal compressor work (× mass flow ratio)
    double work_compressor = compressor.dHt;
    double work_turbine    = turbine.dHt * (1.0 + FAR);
    double balance_error   = (work_turbine - work_compressor)
                             / work_compressor * 100.0;

    std::cout << "========================================\n";
    std::cout << "  Key Performance Parameters\n";
    std::cout << "========================================\n";
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "  Gross Thrust (Fg)  = " << Fg           << " N\n";
    std::cout << "  Fuel Flow   (Wf)   = " << Wf           << " kg/s\n";
    std::cout << "  FAR                = " << FAR           << "\n";
    std::cout << "  TSFC               = "
              << std::setprecision(6) << TSFC              << " kg/N/s\n";
    std::cout << std::setprecision(2);
    std::cout << "  Jet Velocity       = " << nozzle.Vjet  << " m/s\n";
    std::cout << "  Nozzle PR (NPR)    = " << nozzle.NPR   << "\n";
    std::cout << "  Throat Area (Ath)  = "
              << std::setprecision(4) << nozzle.Ath        << " m²\n";
    std::cout << "\n";
    std::cout << "  Shaft Balance Check:\n";
    std::cout << "    Compressor work  = "
              << std::setprecision(1) << work_compressor   << " J/kg\n";
    std::cout << "    Turbine work     = " << work_turbine  << " J/kg\n";
    std::cout << "    Balance error    = "
              << std::setprecision(2) << balance_error     << " %\n";
    std::cout << "========================================\n";

    return 0;
}