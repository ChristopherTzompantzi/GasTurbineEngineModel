#include <iostream>
#include <iomanip>
#include <cmath>
#include <string>
#include <vector>

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
 *   Section 1 — single-spool turbojet  (design point, solver-balanced)
 *   Section 2 — two-spool turbofan with afterburner (design point, solver-balanced)
 *
 * USAGE:
 *   ./GasTurbineEngineModel              — runs both cycles
 *   ./GasTurbineEngineModel turbojet     — runs turbojet only
 *   ./GasTurbineEngineModel turbofan     — runs turbofan only
 *
 * SECTION 1 — TURBOJET DESIGN POINT:
 * Parameters are hardcoded for validation against Mattingly
 * "Elements of Gas Turbine Propulsion" turbojet example.
 * PR_t is solved by the Newton-Raphson solver (N=1 system).
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
 *   PR_t      : solved by Newton-Raphson solver (initial guess: 2.286)
 *   eff_t     : 0.89                 — turbine isentropic efficiency
 *
 * SECTION 2 — TURBOFAN DESIGN POINT:
 * Representative parameters for a low-bypass military turbofan.
 * PR_t_hp, PR_t_lp, and W solved by the Newton-Raphson solver (N=3 system).
 *
 * UNITS — SI INTERNALLY:
 *   Pressures    [Pa]
 *   Temperatures [K]
 *   Mass flow    [kg/s]
 *   Thrust       [N]
 *   TSFC         [kg/N/s]
 */

// =========================================================================
// printStation — prints full FlowStation conditions at each engine station
// =========================================================================
// Static conditions (Ps, Ts) are derived from total conditions and Mach
// number using isentropic relations with real gas gamma from Thermo.
//   Ps = Pt / (1 + (γ-1)/2 × MN²)^(γ/(γ-1))
//   Ts = Tt / (1 + (γ-1)/2 × MN²)
void printStation(const std::string& name,
                  const FlowStation& fs) noexcept
{
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

int main(int argc, char* argv[])
{
    // =========================================================================
    // COMMAND LINE ARGUMENT HANDLING
    // =========================================================================
    // Default: run both cycles.
    // Pass "turbojet" or "turbofan" to run one cycle only.
    bool run_turbojet = true;
    bool run_turbofan = true;

    if (argc >= 2)
    {
        const std::string mode = argv[1];
        if      (mode == "turbojet") { run_turbofan = false; }
        else if (mode == "turbofan") { run_turbojet = false; }
        // "all" or unrecognised argument runs both
    }

    // =========================================================================
    // MAP FILE PATHS
    // =========================================================================
    // Relative to build directory — maps/synthetic/ is at ../maps/synthetic/
    const std::string MAP_DIR = "../maps/synthetic/";

    // =========================================================================
    // SHARED FLIGHT CONDITIONS
    // =========================================================================
    constexpr double altitude_ft = 35000.0;
    const double     altitude_m  = ISA::feetToMeters(altitude_ft);
    constexpr double MN          = 0.85;

    // =========================================================================
    // SECTION 1 — SINGLE SPOOL TURBOJET
    // =========================================================================

    if (run_turbojet)
    {
        std::cout << "========================================\n";
        std::cout << "  GasTurbineEngineModel — Section 1\n";
        std::cout << "  Single Spool Turbojet — Design Point\n";
        std::cout << "========================================\n\n";

        std::cout << "Flight Conditions:\n";
        std::cout << "  Altitude = " << altitude_ft << " ft ("
                  << std::fixed << std::setprecision(1)
                  << altitude_m << " m)\n";
        std::cout << "  Mach     = " << MN << "\n\n";

        // --- Assemble engine elements ---
        Inlet      inlet(altitude_m, MN, 0.97);
        Compressor compressor(10.0, 0.87);
        Combustor  combustor(CombustorMode::Tt4, 1400.0, 0.0, 0.99, 0.04);

        // Turbine — initial guess PR_t = 2.286 (from perfect gas derivation)
        // The Newton-Raphson solver will find the real gas balanced value
        // clearance_corr=0.005 — 0.5% penalty, HP turbine cruise with ACC
        // Reference: Walsh & Fletcher Ch.5
        Turbine    turbine(2.286, 0.89, 0.005);
        Nozzle     nozzle(altitude_m, 0.98);

        // Load performance maps — NPSS S_map subelement pattern
        inlet.loadMap     (MAP_DIR + "inlet.map");
        compressor.loadMap(MAP_DIR + "hp_compressor.map");
        turbine.loadMap   (MAP_DIR + "hp_turbine.map");
        nozzle.loadMap    (MAP_DIR + "nozzle.map");

        // HP shaft — design speed 15,000 RPM physical
        // Solver will find operating shaft speed from map shaft balance
        Shaft hpShaft("HP Shaft", 15000.0);
        hpShaft.addElement(&compressor);
        hpShaft.addElement(&turbine);

        // Connect elements to shaft — enables corrected speed computation
        compressor.connectShaft(&hpShaft);
        turbine.connectShaft   (&hpShaft);

        // --- Set inlet mass flow ---
        constexpr double W_target = 20.0;   // kg/s — fixed for turbojet
        inlet.flowIn.W = W_target;

        // =====================================================================
        // NEWTON-RAPHSON SOLVER — TURBOJET CYCLE EVALUATOR (N=1)
        // =====================================================================
        // Independent variable : x[0] = PR_t
        // Residual             : F[0] = HP shaft balance error [J/kg]
        //
        // The turbojet has one shaft constraint and one unknown. The solver
        // uses the same N×N infrastructure as the turbofan — x and F are
        // 1-element vectors. W is fixed at 20 kg/s and is not iterated.
        //
        // Scale factor: 1e5 J/kg — typical magnitude of shaft balance residual
        // =====================================================================

        auto tj_cycleEvaluator = [&](const std::vector<double>& x)
            -> std::vector<double>
        {
            // Inject independent variable — shaft speed [RPM]
            hpShaft.setSpeed(x[0]);

            // Run turbojet cycle
            inlet.compute();

            compressor.flowIn = inlet.flowOut;
            compressor.compute();

            combustor.flowIn = compressor.flowOut;
            combustor.compute();

            turbine.flowIn = combustor.flowOut;
            turbine.compute();

            nozzle.flowIn = turbine.flowOut;
            nozzle.compute();

            // Compute shaft balance
            hpShaft.computeBalance();

            // F[0] — HP shaft balance error [J/kg]
            return { hpShaft.getBalanceError() };
        };

        // Construct and run solver
        // Scale: {1e5} — shaft balance in J/kg
        // Scale: {1e4} — shaft speed in RPM, O(10000)
        Solver tj_solver(tj_cycleEvaluator, {1.0e4});
        const std::vector<double> tj_x0 = { 14000.0 };   // initial guess near operating point
        const SolverResult tj_result = tj_solver.solve(tj_x0);
        Solver::printResult(tj_result, {"N_hp [RPM]"});

        // --- Final print pass — re-run cycle with converged shaft speed ---
        hpShaft.setSpeed(tj_result.x[0]);

        inlet.compute();
        printStation("Station 2 — Inlet Exit", inlet.flowOut);

        compressor.flowIn = inlet.flowOut;
        compressor.compute();
        printStation("Station 3 — Compressor Exit", compressor.flowOut);

        combustor.flowIn = compressor.flowOut;
        combustor.compute();
        printStation("Station 4 — Turbine Inlet (Tt4)", combustor.flowOut);

        turbine.flowIn = combustor.flowOut;
        turbine.compute();
        printStation("Station 5 — Turbine Exit", turbine.flowOut);

        nozzle.flowIn = turbine.flowOut;
        nozzle.compute();
        printStation("Station 9 — Nozzle Exit", nozzle.flowOut);

        // --- Key performance parameters ---
        const double Fg   = nozzle.Fg;
        const double W    = inlet.flowIn.W;
        const double FAR  = combustor.flowOut.FAR;
        const double Wf   = W * FAR;
        const double TSFC = Wf / Fg;

        hpShaft.computeBalance();

        std::cout << "========================================\n";
        std::cout << "  Key Performance Parameters\n";
        std::cout << "========================================\n";
        std::cout << std::fixed << std::setprecision(2);
        std::cout << "  Gross Thrust (Fg)  = " << Fg          << " N\n";
        std::cout << "  Fuel Flow   (Wf)   = " << Wf          << " kg/s\n";
        std::cout << "  FAR                = " << FAR          << "\n";
        std::cout << "  TSFC               = "
                  << std::setprecision(6) << TSFC             << " kg/N/s\n";
        std::cout << std::setprecision(2);
        std::cout << "  Jet Velocity       = " << nozzle.Vjet << " m/s\n";
        std::cout << "  Nozzle PR (NPR)    = " << nozzle.NPR  << "\n";
        std::cout << "  Throat Area (Ath)  = "
                  << std::setprecision(4) << nozzle.Ath       << " m²\n";
        std::cout << "\n";
        hpShaft.printBalance();
        std::cout << std::fixed << std::setprecision(2);
        std::cout << "  Compressor surge margin : "
                  << compressor.surge_margin << " %\n";
        std::cout << std::setprecision(4);
        std::cout << "  Compressor eta_is       : "
                  << compressor.eff_is   << "\n";
        std::cout << "  Compressor eta_poly     : "
                  << compressor.eta_poly  << "\n";
        std::cout << "  Turbine    eta_is       : "
                  << turbine.eff_is      << "\n";
        std::cout << "  Turbine    eta_poly     : "
                  << turbine.eta_poly     << "\n";
        std::cout << "  Turbine    clearance    : "
                  << std::setprecision(3) << 0.5 << " %\n";
        std::cout << "========================================\n";
    }

    // =========================================================================
    // SECTION 2 — TWO SPOOL TURBOFAN WITH AFTERBURNER
    // =========================================================================
    //
    // Configuration:
    //   Inlet → Fan → Splitter
    //                   ├→ FanNozzle         (bypass / cold stream)
    //                   └→ Compressor → Combustor → Turbine (HP)
    //                                             → Turbine (LP)
    //                                             → Afterburner
    //                                             → Nozzle   (core / hot stream)
    //
    // Shafts:
    //   LP Shaft: Fan ←→ LP Turbine
    //   HP Shaft: Compressor ←→ HP Turbine
    //
    // Source: Representative parameters for a low-bypass military turbofan.
    // Not matched to a specific engine — for architecture validation only.
    //
    // Design Point Conditions:
    //   Altitude  : 10,668 m (35,000 ft)
    //   Mach      : 0.85
    //   W         : 60.0 kg/s
    //   BPR       : 0.8
    //   PR_f      : 3.5
    //   eff_f     : 0.89
    //   PR_c      : 8.0
    //   eff_c     : 0.87
    //   Tt4       : 1700.0 K
    //   PR_t_hp   : solved by Newton-Raphson solver (initial guess: 2.4186)
    //   PR_t_lp   : solved by Newton-Raphson solver (initial guess: 1.9663)
    //   eff_t     : 0.89
    //   Tt7       : 2100.0 K

    if (run_turbofan)
    {
        std::cout << "\n\n";
        std::cout << "========================================\n";
        std::cout << "  GasTurbineEngineModel — Section 2\n";
        std::cout << "  Two Spool Turbofan — Design Point\n";
        std::cout << "========================================\n\n";

        std::cout << "Flight Conditions:\n";
        std::cout << "  Altitude = " << altitude_ft << " ft ("
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
        // Initial guesses are from the Phase 1b analytical derivation.
        Turbine    tf_turbine_hp(2.4186, 0.89, 0.005);  // 0.5% clearance penalty — HP cruise
        Turbine    tf_turbine_lp(1.9663, 0.89, 0.003);  // 0.3% clearance penalty — LP cruise

        Afterburner tf_afterburner(2100.0);
        Nozzle      tf_nozzle(altitude_m, 0.98);
        FanNozzle   tf_fanNozzle(altitude_m, 0.98);

        // Load performance maps — NPSS S_map subelement pattern
        tf_inlet.loadMap      (MAP_DIR + "inlet.map");
        tf_fan.loadMap        (MAP_DIR + "fan.map");
        tf_compressor.loadMap (MAP_DIR + "hp_compressor.map");
        tf_turbine_hp.loadMap (MAP_DIR + "hp_turbine.map");
        tf_turbine_lp.loadMap (MAP_DIR + "lp_turbine.map");
        tf_nozzle.loadMap     (MAP_DIR + "nozzle.map");
        tf_fanNozzle.loadMap  (MAP_DIR + "fan_nozzle.map");

        // --- Shaft connections ---
        // HP shaft — design speed 15,000 RPM
        Shaft tf_hpShaft("HP Shaft", 15000.0);
        tf_hpShaft.addElement(&tf_compressor);
        tf_hpShaft.addElement(&tf_turbine_hp);

        // LP shaft — design speed 8,000 RPM
        Shaft tf_lpShaft("LP Shaft", 8000.0);
        // NOTE: Fan is NOT added via getWork() — mixed mass flow bases.
        // LP balance verified manually in physical power [W].
        tf_lpShaft.addElement(&tf_turbine_lp);

        // Connect elements to shafts
        tf_compressor.connectShaft(&tf_hpShaft);
        tf_turbine_hp.connectShaft(&tf_hpShaft);
        tf_fan.connectShaft       (&tf_lpShaft);
        tf_turbine_lp.connectShaft(&tf_lpShaft);

        // --- Target mass flow ---
        constexpr double tf_W_target = 60.0;   // kg/s — solver will enforce this

        // =====================================================================
        // NEWTON-RAPHSON SOLVER — TURBOFAN CYCLE EVALUATOR (N=3)
        // =====================================================================
        // Independent variables : x = [PR_t_hp, PR_t_lp, W]
        // Residuals             : F = [F_hp, F_lp, F_W]
        //
        //   F[0] = HP shaft balance error [J/kg]
        //          = driver work (HP turbine) + driven work (HP compressor)
        //          Target: 0.0
        //
        //   F[1] = LP shaft balance error [W] (physical power basis)
        //          = LP turbine power - fan power
        //          Target: 0.0
        //
        //   F[2] = mass flow residual [kg/s]
        //          = actual inlet W - target W
        //          Target: 0.0
        //
        // Scale factors: {1e5, 1e6, 1.0}
        //   1e5  — typical HP shaft balance magnitude [J/kg]
        //   1e6  — typical LP shaft power magnitude [W]
        //   1.0  — mass flow residual [kg/s] already O(1)
        //
        // The lambda captures all element objects by reference. After the
        // solver converges, the elements already hold the correct converged
        // station data — no second solve needed for the print pass.
        // =====================================================================

        auto tf_cycleEvaluator = [&](const std::vector<double>& x)
            -> std::vector<double>
        {
            // Inject independent variables — shaft speeds [RPM]
            tf_hpShaft.setSpeed(x[0]);
            tf_lpShaft.setSpeed(x[1]);
            tf_inlet.flowIn.W = x[2];

            // Run full turbofan cycle
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

            // F[0] — HP shaft balance error [J/kg]
            const double F_hp = tf_hpShaft.getBalanceError();

            // F[1] — LP shaft balance error [W] (physical power basis)
            const double tf_FAR_lp = tf_combustor.flowOut.FAR;
            const double F_lp      = tf_turbine_lp.dHt * (1.0 + tf_FAR_lp)
                                   * tf_splitter.flowOut.W
                                   - tf_fan.dHt * tf_inlet.flowIn.W;

            // F[2] — mass flow residual [kg/s]
            const double F_W = tf_inlet.flowIn.W - tf_W_target;

            return { F_hp, F_lp, F_W };
        };

        // --- Construct and run solver ---
        // Scale factors: RPM O(10000), RPM O(10000), kg/s O(1)
        Solver tf_solver(tf_cycleEvaluator, {1.0e4, 1.0e4, 1.0});
        const std::vector<double> tf_x0 = { 15000.0,   // HP design shaft speed [RPM]
                                             8000.0,    // LP design shaft speed [RPM]
                                             tf_W_target };
        const SolverResult tf_result = tf_solver.solve(tf_x0);
        Solver::printResult(tf_result, {"N_hp [RPM]", "N_lp [RPM]", "W"});

        // --- Final print pass — re-run cycle with converged solution ---
        tf_hpShaft.setSpeed(tf_result.x[0]);
        tf_lpShaft.setSpeed(tf_result.x[1]);
        tf_inlet.flowIn.W = tf_result.x[2];

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
        const double tf_Fg_hot  = tf_nozzle.Fg;
        const double tf_Fg_cold = tf_fanNozzle.Fg_cold;
        const double tf_Fg      = tf_Fg_hot + tf_Fg_cold;
        const double tf_FAR     = tf_combustor.flowOut.FAR;
        const double tf_FAR_ab  = tf_afterburner.FAR_ab;
        const double tf_W_core  = tf_splitter.flowOut.W;
        const double tf_Wf_core = tf_W_core * tf_FAR;
        const double tf_Wf_ab   = tf_W_core * tf_FAR_ab;
        const double tf_Wf      = tf_Wf_core + tf_Wf_ab;
        const double tf_TSFC    = tf_Wf / tf_Fg;

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
        std::cout << std::fixed << std::setprecision(2);
        std::cout << "  HP Compressor surge margin : "
                  << tf_compressor.surge_margin << " %\n";
        std::cout << "  Fan surge margin           : "
                  << tf_fan.surge_margin        << " %\n";
        std::cout << std::setprecision(4);
        std::cout << "  HP Compressor eta_is       : "
                  << tf_compressor.eff_is  << "\n";
        std::cout << "  HP Compressor eta_poly     : "
                  << tf_compressor.eta_poly << "\n";
        std::cout << "  Fan eta_is                 : "
                  << tf_fan.eff_is         << "\n";
        std::cout << "  Fan eta_poly               : "
                  << tf_fan.eta_poly        << "\n";
        std::cout << "  HP Turbine eta_is          : "
                  << tf_turbine_hp.eff_is   << "\n";
        std::cout << "  HP Turbine eta_poly        : "
                  << tf_turbine_hp.eta_poly << "\n";
        std::cout << "  HP Turbine clearance       : "
                  << std::setprecision(3) << 0.5 << " %\n";
        std::cout << "  LP Turbine eta_is          : "
                  << std::setprecision(4)
                  << tf_turbine_lp.eff_is   << "\n";
        std::cout << "  LP Turbine eta_poly        : "
                  << tf_turbine_lp.eta_poly << "\n";
        std::cout << "  LP Turbine clearance       : "
                  << std::setprecision(3) << 0.3 << " %\n\n";

        // LP shaft — manual power balance (mixed mass flow bases)
        // Physical power [W] = specific work [J/kg] × mass flow [kg/s]
        const double LP_driver_power = tf_turbine_lp.dHt * (1.0 + tf_FAR)
                                     * tf_splitter.flowOut.W;
        const double LP_driven_power = tf_fan.dHt * tf_inlet.flowIn.W;
        const double LP_balance_pct  = (LP_driver_power - LP_driven_power)
                                     / LP_driven_power * 100.0;

        std::cout << "--- Shaft: LP Shaft ---\n";
        std::cout << "  Driver power (LP turbine) = "
                  << std::fixed << std::setprecision(2)
                  << LP_driver_power << " W\n";
        std::cout << "  Driven power (fan)        = "
                  << LP_driven_power << " W\n";
        std::cout << "  Balance error             = "
                  << LP_driver_power - LP_driven_power << " W"
                  << "  (" << LP_balance_pct << " %)\n\n";

        std::cout << "========================================\n";
    }

    return 0;
}