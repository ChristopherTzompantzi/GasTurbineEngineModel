#include <iostream>
#include <iomanip>
#include <cmath>

#include "Inlet.h"
#include "Compressor.h"
#include "Combustor.h"
#include "Turbine.h"
#include "Nozzle.h"
#include "ISA.h"
#include "Shaft.h"
#include "Fan.h"
#include "Splitter.h"
#include "FanNozzle.h"
#include "Afterburner.h"
#include "Solver.h"

/*
 * main.cpp
 * --------
 * Entry point for the GasTurbineEngineModel simulation.
 * Assembles and runs engine cycle simulations:
 *   Phase 1a — single-spool turbojet (design point, validated vs Mattingly)
 *   Phase 1b — two-spool turbofan with afterburner (design point, solver-balanced)
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
 *   PR_t      : 2.286                — turbine pressure ratio (analytically derived)
 *   eff_t     : 0.89                 — turbine isentropic efficiency
 *
 * NOTE: PR_t = 2.286 is derived analytically from the shaft power balance.
 * The solver (Phase 2) will compute this automatically at runtime.
 * Current balance error is reported via hpShaft.printBalance().
 *
 * VALIDATION TARGET:
 *   Results will be compared against Mattingly textbook example.
 *   Acceptable tolerance: within 2% on thrust and TSFC.
 *
 * UNITS — SI INTERNALLY:
 *   Pressures    [Pa]
 *   Temperatures [K]
 *   Mass flow    [kg/s]
 *   Thrust       [N]
 *   TSFC         [kg/N/s]
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

    // Turbine — PR=2.286, isentropic efficiency=0.89
    // PR_t calculated analytically from shaft power balance using exact FAR:
    //   FAR_exact = Cp×(Tt4-Tt3) / (LHV×eff_b - Cp×Tt4) = 0.021462
    //   (1 + FAR) × (Tt4 - Tt5) = (Tt3 - Tt2)
    //   Tt5_required = 1400.0 - 267.89/1.021462 = 1137.74 K
    //   PR_t = 1 / (1 - (1 - Tt5/Tt4)/eff_t)^(γ/(γ-1)) = 2.286
    // The solver (Phase 2) will compute this automatically at runtime.
    Turbine turbine(2.286, 0.89);

    // Nozzle — convergent, typical efficiency
    Nozzle nozzle(altitude_m, 0.98);

    // HP Shaft — connects compressor and turbine
    // addElement() does not depend on compute() — safe to call here
    // computeBalance() is called after the cycle
    Shaft hpShaft("HP Shaft");
    hpShaft.addElement(&compressor);
    hpShaft.addElement(&turbine);

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

    // Shaft power balance — all elements have called compute(), balance is now valid
    hpShaft.computeBalance();

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
hpShaft.printBalance();
    std::cout << "========================================\n";

    //
    // ================================================================
    //  SECTION 2 — TWO SPOOL TURBOFAN WITH AFTERBURNER
    //  Design point conditions representative of a low-bypass
    //  military turbofan (e.g. F100/F110 class engine).
    //
    //  Configuration:
    //    Inlet → Fan → Splitter
    //                    ├→ FanNozzle         (bypass / cold stream)
    //                    └→ Compressor → Combustor → Turbine (HP)
    //                                              → Turbine (LP)
    //                                              → Afterburner
    //                                              → Nozzle   (core / hot stream)
    //
    //  Shafts:
    //    LP Shaft: Fan ←→ LP Turbine
    //    HP Shaft: Compressor ←→ HP Turbine
    //
    //  Source: Representative parameters for a low-bypass military
    //  turbofan. Not matched to a specific engine — for architecture
    //  validation only. Phase 2 will use published engine deck data.
    //
    //  Design Point Conditions:
    //    Altitude  : 10,668 m (35,000 ft) — same as turbojet baseline
    //    Mach      : 0.85                 — same as turbojet baseline
    //    W         : 60.0 kg/s            — larger engine
    //    BPR       : 0.8                  — low bypass (military class)
    //    PR_f      : 3.5                  — fan pressure ratio
    //    eff_f     : 0.89                 — fan efficiency
    //    PR_c      : 8.0                  — HP compressor pressure ratio
    //    eff_c     : 0.87                 — HP compressor efficiency
    //    Tt4       : 1700.0 K             — turbine inlet temperature
    //    PR_t_hp   : solved by Newton-Raphson solver (initial guess: 2.4186)
    //    PR_t_lp   : solved by Newton-Raphson solver (initial guess: 1.9663)
    //    eff_t     : 0.89                 — both turbine stages
    //    Tt7       : 2100.0 K             — afterburner exit temperature
    // ================================================================
    //

    std::cout << "\n\n";
    std::cout << "========================================\n";
    std::cout << "  GasTurbineEngineModel — Phase 1b\n";
    std::cout << "  Two Spool Turbofan — Design Point\n";
    std::cout << "========================================\n\n";

    // --- Flight conditions (same as turbojet baseline) ---
    // altitude_m and MN already defined above — reuse them

    std::cout << "Flight Conditions:\n";
    std::cout << "  Altitude = " << 35000.0 << " ft ("
              << std::fixed << std::setprecision(1)
              << altitude_m << " m)\n";
    std::cout << "  Mach     = " << MN << "\n\n";

    // --- Assemble turbofan elements ---

    Inlet      tf_inlet(altitude_m, MN, 0.97);
    Fan        tf_fan(3.5, 0.89);
    Splitter   tf_splitter(0.8);
    Compressor tf_compressor(8.0, 0.87);
    Combustor  tf_combustor(CombustorMode::Tt4, 1700.0, 0.0, 0.99, 0.04);

    // Turbines constructed with initial guess PRs.
    // The Newton-Raphson solver will find the converged values via setPR().
    // Initial guesses are from the Phase 1b analytical derivation — they are
    // close to the solution and will allow the solver to converge in few iterations.
    Turbine    tf_turbine_hp(2.4186, 0.89);   // initial guess — solver will refine
    Turbine    tf_turbine_lp(1.9663, 0.89);   // initial guess — solver will refine

    Afterburner tf_afterburner(2100.0);
    Nozzle      tf_nozzle(altitude_m, 0.98);
    FanNozzle   tf_fanNozzle(altitude_m, 0.98);

    // --- Shaft connections ---
    Shaft tf_hpShaft("HP Shaft");
    tf_hpShaft.addElement(&tf_compressor);
    tf_hpShaft.addElement(&tf_turbine_hp);

    Shaft tf_lpShaft("LP Shaft");
    // NOTE: Fan is NOT added via getWork() — mixed mass flow bases.
    // LP balance verified manually in physical power [W].
    tf_lpShaft.addElement(&tf_turbine_lp);

    // --- Target mass flow ---
    constexpr double tf_W_target = 60.0;   // kg/s — solver will enforce this

    // =========================================================================
    // NEWTON-RAPHSON SOLVER — CYCLE EVALUATOR LAMBDA
    // =========================================================================
    // The lambda is the bridge between the solver and the engine elements.
    // It receives x = [PR_t_hp, PR_t_lp, W] from the solver, runs the full
    // turbofan cycle, and returns F = [F_hp, F_lp, F_W]:
    //
    //   F[0] = HP shaft balance error [J/kg]
    //          = driver work (HP turbine) + driven work (HP compressor)
    //          Target: 0.0
    //
    //   F[1] = LP shaft balance error [W]
    //          = LP turbine power - fan power
    //          Target: 0.0
    //
    //   F[2] = mass flow residual [kg/s]
    //          = actual inlet W - target W
    //          Target: 0.0
    //
    // The lambda captures all element objects by reference. This means it
    // operates on the same objects that will be used for the final print pass,
    // so after the solver converges the elements already hold the correct
    // converged station data — no second solve needed.
    // =========================================================================

    auto cycleEvaluator = [&](const std::array<double,3>& x)
        -> std::array<double,3>
    {
        // Unpack independent variables
        const double PR_t_hp = x[0];
        const double PR_t_lp = x[1];
        const double W_in    = x[2];

        // Inject independent variables into elements
        tf_turbine_hp.setPR(PR_t_hp);
        tf_turbine_lp.setPR(PR_t_lp);
        tf_inlet.flowIn.W = W_in;

        // Run full turbofan cycle (same sequence as print pass below)
        tf_inlet.compute();

        tf_fan.flowIn = tf_inlet.flowOut;
        tf_fan.compute();

        tf_splitter.flowIn = tf_fan.flowOut;
        tf_splitter.compute();

        tf_compressor.flowIn = tf_splitter.flowOut;
        tf_compressor.compute();

        tf_combustor.flowIn = tf_compressor.flowOut;
        tf_combustor.compute();

        tf_turbine_hp.flowIn = tf_combustor.flowOut;
        tf_turbine_hp.compute();

        tf_turbine_lp.flowIn = tf_turbine_hp.flowOut;
        tf_turbine_lp.compute();

        tf_afterburner.flowIn = tf_turbine_lp.flowOut;
        tf_afterburner.compute();

        tf_nozzle.flowIn = tf_afterburner.flowOut;
        tf_nozzle.compute();

        tf_fanNozzle.flowIn = tf_splitter.bypassOut;
        tf_fanNozzle.compute();

        // Compute shaft balances
        tf_hpShaft.computeBalance();
        tf_lpShaft.computeBalance();

        // Evaluate residuals
        // F[0] — HP shaft balance error [J/kg]
        const double F_hp = tf_hpShaft.getBalanceError();

        // F[1] — LP shaft balance error [W] (physical power basis)
        const double tf_FAR_lambda = tf_combustor.flowOut.FAR;
        const double F_lp = tf_turbine_lp.dHt * (1.0 + tf_FAR_lambda)
                            * tf_splitter.flowOut.W
                            - tf_fan.dHt * tf_inlet.flowIn.W;

        // F[2] — mass flow residual [kg/s]
        const double F_W = tf_inlet.flowIn.W - tf_W_target;

        return { F_hp, F_lp, F_W };
    };

    // --- Construct and run solver ---
    Solver tf_solver(cycleEvaluator);
    const std::array<double,3> x0 = { tf_turbine_hp.PR_t,
                                       tf_turbine_lp.PR_t,
                                       tf_W_target };
    const SolverResult tf_result = tf_solver.solve(x0);
    Solver::printResult(tf_result);

    // --- Final print pass — re-run cycle with converged solution ---
    // The solver's last lambda call already set all element states to the
    // converged values. This pass re-runs the cycle to ensure the print
    // stations reflect the final converged x exactly.
    tf_inlet.flowIn.W = tf_result.x[2];   // confirm converged W is set

    // --- Run turbofan cycle ---

    // Station 2 — Inlet exit
    tf_inlet.compute();
    printStation("Station 2 — Inlet Exit", tf_inlet.flowOut);

    // Station 2.1 — Fan exit
    tf_fan.flowIn = tf_inlet.flowOut;
    tf_fan.compute();
    printStation("Station 2.1 — Fan Exit", tf_fan.flowOut);

    // Station 2.2 — Splitter
    tf_splitter.flowIn = tf_fan.flowOut;
    tf_splitter.compute();
    printStation("Station 2.2 — Core Stream (Splitter coreOut)",
                 tf_splitter.flowOut);
    printStation("Station 2.3 — Bypass Stream (Splitter bypassOut)",
                 tf_splitter.bypassOut);

    // Station 3 — HP Compressor exit
    tf_compressor.flowIn = tf_splitter.flowOut;
    tf_compressor.compute();
    printStation("Station 3 — HP Compressor Exit", tf_compressor.flowOut);

    // Station 4 — Combustor exit / HP Turbine inlet
    tf_combustor.flowIn = tf_compressor.flowOut;
    tf_combustor.compute();
    printStation("Station 4 — Combustor Exit (Tt4)", tf_combustor.flowOut);

    // Station 4.5 — HP Turbine exit / LP Turbine inlet
    tf_turbine_hp.flowIn = tf_combustor.flowOut;
    tf_turbine_hp.compute();
    printStation("Station 4.5 — HP Turbine Exit", tf_turbine_hp.flowOut);

    // Station 5 — LP Turbine exit
    tf_turbine_lp.flowIn = tf_turbine_hp.flowOut;
    tf_turbine_lp.compute();
    printStation("Station 5 — LP Turbine Exit", tf_turbine_lp.flowOut);

    // Station 7 — Afterburner exit
    tf_afterburner.flowIn = tf_turbine_lp.flowOut;
    tf_afterburner.compute();
    printStation("Station 7 — Afterburner Exit (Tt7)", tf_afterburner.flowOut);

    // Station 9 — Core nozzle exit
    tf_nozzle.flowIn = tf_afterburner.flowOut;
    tf_nozzle.compute();
    printStation("Station 9 — Core Nozzle Exit", tf_nozzle.flowOut);

    // Station 19 — Fan nozzle exit (bypass stream)
    tf_fanNozzle.flowIn = tf_splitter.bypassOut;
    tf_fanNozzle.compute();
    printStation("Station 19 — Fan Nozzle Exit", tf_fanNozzle.flowOut);

    // --- Shaft balance ---
    tf_hpShaft.computeBalance();
    tf_lpShaft.computeBalance();

    // --- Key performance parameters ---
    double tf_Fg_hot  = tf_nozzle.Fg;
    double tf_Fg_cold = tf_fanNozzle.Fg_cold;
    double tf_Fg      = tf_Fg_hot + tf_Fg_cold;
    double tf_FAR     = tf_combustor.flowOut.FAR;
    double tf_FAR_ab  = tf_afterburner.FAR_ab;
    double tf_W_core  = tf_splitter.flowOut.W;          // core air mass flow [kg/s]
    double tf_Wf_core = tf_W_core * tf_FAR;             // core combustor fuel flow
    double tf_Wf_ab   = tf_W_core * tf_FAR_ab;          // afterburner fuel flow
    double tf_Wf      = tf_Wf_core + tf_Wf_ab;          // total fuel flow
    double tf_TSFC    = tf_Wf / tf_Fg;

    std::cout << "========================================\n";
    std::cout << "  Key Performance Parameters — Turbofan\n";
    std::cout << "========================================\n";
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "  Hot Thrust  (Fg_hot)   = " << tf_Fg_hot  << " N\n";
    std::cout << "  Cold Thrust (Fg_cold)  = " << tf_Fg_cold << " N\n";
    std::cout << "  Total Thrust (Fg)      = " << tf_Fg      << " N\n";
    std::cout << "  Core Fuel Flow         = " << tf_Wf_core << " kg/s\n";
    std::cout << "  AB Fuel Flow           = " << tf_Wf_ab   << " kg/s\n";
    std::cout << "  Total Fuel Flow        = " << tf_Wf      << " kg/s\n";
    std::cout << "  TSFC                   = "
              << std::setprecision(6) << tf_TSFC << " kg/N/s\n\n";

    tf_hpShaft.printBalance();

    // LP shaft — manual power balance (fan and LP turbine on different mass flow bases)
    // Physical power [W] = specific work [J/kg] × mass flow [kg/s]
    double LP_driver_power = tf_turbine_lp.dHt * (1.0 + tf_FAR)
                             * tf_splitter.flowOut.W;
    double LP_driven_power = tf_fan.dHt * tf_inlet.flowIn.W;
    double LP_balance_pct  = (LP_driver_power - LP_driven_power)
                             / LP_driven_power * 100.0;

    std::cout << "--- Shaft: LP Shaft ---\n";
    std::cout << "  Driver power (LP turbine) = "
              << std::fixed << std::setprecision(2)
              << LP_driver_power << " W\n";
    std::cout << "  Driven power (fan)        = " << LP_driven_power << " W\n";
    std::cout << "  Balance error             = "
              << LP_driver_power - LP_driven_power << " W"
              << "  (" << LP_balance_pct << " %)\n\n";

    std::cout << "========================================\n";

    return 0;
}