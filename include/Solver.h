// =============================================================================
// Solver.h
// -----------------------------------------------------------------------------
// Newton-Raphson solver for the GasTurbineEngineModel two-spool turbofan cycle.
//
// =============================================================================
// NUMERICAL METHOD — NEWTON-RAPHSON
// =============================================================================
//
// WHAT PROBLEM ARE WE SOLVING?
// -----------------------------
// A gas turbine cycle is a system of coupled nonlinear equations. At any
// operating condition, several constraints must be satisfied simultaneously:
//
//   (1) HP shaft power balance  — HP turbine work = HP compressor work
//   (2) LP shaft power balance  — LP turbine power = fan power
//   (3) Mass flow consistency   — inlet mass flow propagates correctly
//
// Each constraint is expressed as a residual:
//
//   F_i(x) = 0
//
// where x is a vector of independent variables (unknowns) and F is a vector
// of residuals (constraint violations). The goal of the solver is to find the
// x that makes all F_i simultaneously zero.
//
// For our two-spool turbofan, the three unknowns and residuals are:
//
//   x[0] = PR_t_hp    →  F[0] = HP shaft balance error  [J/kg]
//   x[1] = PR_t_lp    →  F[1] = LP shaft balance error  [W]
//   x[2] = W          →  F[2] = mass flow residual       [kg/s]
//                                (target W minus actual W at inlet)
//
// WHY NEWTON-RAPHSON?
// -------------------
// Newton-Raphson (NR) is selected because:
//
//   1. QUADRATIC CONVERGENCE — once near the solution, the number of correct
//      digits roughly doubles each iteration. In practice, engine cycle balance
//      converges in 4-8 iterations from a reasonable starting guess.
//
//   2. WELL-SUITED TO SMOOTH SYSTEMS — gas turbine thermodynamics are smooth
//      and continuous (away from the choked/unchoked nozzle transition), which
//      is exactly the regime where NR excels.
//
//   3. SMALL SYSTEM SIZE — with only 3 unknowns, the Jacobian is a 3x3 matrix.
//      Solving a 3x3 linear system per iteration is computationally trivial.
//
//   4. INDUSTRY PRECEDENT — this is the same architecture used by NPSS
//      (Numerical Propulsion System Simulation), the NASA/industry standard
//      engine cycle tool that this project is inspired by.
//
// WHY NOT OTHER METHODS?
// ----------------------
//   Successive substitution — only linearly convergent, can fail to converge
//     for stiff systems. Ruled out.
//   Broyden's method — quasi-Newton, approximates the Jacobian rather than
//     recomputing it. Useful when cycle evaluations are expensive. Overkill
//     here since our cycle evaluates in microseconds.
//   Gradient-free methods (Nelder-Mead, genetic algorithms) — global
//     optimizers for design problems with many local minima. Far too slow
//     for cycle balance. Ruled out.
//
// HOW THE METHOD WORKS
// --------------------
// The NR iteration for a system of equations is:
//
//   x_{n+1} = x_n - J^{-1} * F(x_n)
//
// where J is the Jacobian matrix of partial derivatives:
//
//   J_ij = dF_i / dx_j
//
// In practice, J is never inverted directly. Instead, the linear system:
//
//   J * delta_x = -F
//
// is solved for the update step delta_x, then applied:
//
//   x_{n+1} = x_n + delta_x
//
// This is numerically more stable than explicit inversion and avoids
// accumulation of floating-point errors.
//
// JACOBIAN BY FINITE DIFFERENCES
// --------------------------------
// Analytical Jacobians require differentiating the entire engine cycle — every
// isentropic relation, the combustor FAR equation, the nozzle thrust equation.
// This is error-prone to derive and maintain as the code evolves.
//
// Instead, we use numerical finite differences. For each independent variable
// x_j, we perturb it by a small amount h and re-run the full cycle:
//
//   J_ij ≈ ( F_i(x + h*e_j) - F_i(x) ) / h
//
// where e_j is the unit vector in the j-th direction. This requires N+1 cycle
// evaluations per Newton step (1 baseline + N perturbations). For N=3:
//   - 4 cycle evaluations per Newton step
//   - ~5 Newton steps to converge
//   - ~20 total cycle evaluations
//   - Each cycle evaluation takes ~microseconds
//   - Total solve time: milliseconds
//
// WHAT THIS MEANS FOR OUR CODE
// ==============================
// The solver is a standalone class in src/solver/Solver.cpp. It wraps the
// engine cycle (the sequence of element compute() calls in main.cpp) inside
// a lambda function called the "cycle evaluator". The solver treats the cycle
// as a black box: given a vector of independent variables x, it runs the
// cycle and returns the residual vector F.
//
// The engine element classes (Fan, Compressor, Combustor, Turbine, Nozzle,
// etc.) are NOT modified. They remain exactly as implemented in Phase 1b.
// The solver operates by:
//
//   Step 1 — Receive initial guess for x = [PR_t_hp, PR_t_lp, W]
//   Step 2 — Evaluate F(x) by running the full engine cycle
//   Step 3 — Check convergence: if ||F|| < tolerance, done
//   Step 4 — Perturb each x_j by h, re-run cycle, compute column j of J
//   Step 5 — Solve J * delta_x = -F using Gaussian elimination (3x3)
//   Step 6 — Update x = x + delta_x, go to Step 2
//
// The cycle evaluator is passed into the solver as a std::function. This keeps
// the solver generic — it has no knowledge of Fan, Compressor, Turbine, etc.
// It only knows: "given x, evaluate F". This separation of concerns means the
// solver can be reused for any engine configuration in Phase 2 and beyond.
//
// CONVERGENCE CRITERION
// ----------------------
// The solver converges when the L2 norm of the residual vector falls below
// a specified absolute tolerance:
//
//   ||F|| = sqrt(F[0]^2 + F[1]^2 + F[2]^2) < tolerance
//
// Default tolerance is 1e-4 (dimensionless after normalisation — see Solver.cpp
// for how residuals are normalised before the norm is computed).
//
// PERTURBATION STEP SIZE
// -----------------------
// The finite-difference step h must be:
//   - Large enough that the perturbed residual differs meaningfully from the
//     baseline (avoids cancellation error in floating-point subtraction)
//   - Small enough that the linear approximation (Taylor expansion) is valid
//
// Default values:
//   h_PR  = 1e-4  (relative perturbation on pressure ratio)
//   h_W   = 1e-3  (absolute perturbation on mass flow [kg/s])
//
// SOURCE REFERENCES
// -----------------
// Numerical method:
//   Press, W.H. et al., "Numerical Recipes in C++", 3rd ed., Cambridge, 2007.
//   Chapter 9 — Root Finding and Nonlinear Sets of Equations.
//
// Engine cycle solver architecture:
//   Lytle, J.K., "The Numerical Propulsion System Simulation: An Overview",
//   NASA/TM-2000-209915, 2000.
//
// Finite-difference Jacobians for engine cycles:
//   Mattingly, J.D., "Elements of Gas Turbine Propulsion",
//   McGraw-Hill, 1996. Chapter 5.
//
// =============================================================================

#ifndef SOLVER_H
#define SOLVER_H

#include <array>
#include <functional>
#include <string>

// -----------------------------------------------------------------------------
// SolverResult — returned by Solver::solve()
// -----------------------------------------------------------------------------
struct SolverResult
{
    bool   converged    = false;  // true if ||F|| < tolerance at exit
    int    iterations   = 0;      // number of Newton steps taken
    double residualNorm = 0.0;    // ||F|| at exit
    std::array<double, 3> x = {0.0, 0.0, 0.0};  // final [PR_t_hp, PR_t_lp, W]
};

// -----------------------------------------------------------------------------
// Solver
//
// Newton-Raphson solver for the two-spool turbofan cycle.
//
// Usage:
//   1. Construct with a cycle evaluator function and convergence parameters.
//   2. Call solve() with an initial guess.
//   3. Inspect the returned SolverResult.
//
// The cycle evaluator has the signature:
//   std::array<double,3> evaluator(const std::array<double,3>& x)
//
// It receives x = [PR_t_hp, PR_t_lp, W] and returns F = [F_hp, F_lp, F_W].
// The evaluator is responsible for running the full engine cycle and computing
// the residuals. The solver has no knowledge of engine elements.
// -----------------------------------------------------------------------------
class Solver
{
public:
    // CycleEvaluator type alias — takes x, returns F
    using CycleEvaluator = std::function<std::array<double,3>
                                        (const std::array<double,3>&)>;

    // Constructor
    // evaluator    — cycle evaluator function (see above)
    // maxIter      — maximum Newton iterations before declaring non-convergence
    // tolerance    — convergence criterion on ||F|| (normalised)
    // h_PR         — finite-difference step for pressure ratio unknowns
    // h_W          — finite-difference step for mass flow unknown [kg/s]
    explicit Solver(CycleEvaluator  evaluator,
                    int             maxIter   = 50,
                    double          tolerance = 1.0e-4,
                    double          h_PR      = 1.0e-4,
                    double          h_W       = 1.0e-3);

    // solve — runs the Newton-Raphson iteration from initial guess x0
    // Returns a SolverResult describing convergence, iteration count, and
    // the final solution vector.
    SolverResult solve(const std::array<double,3>& x0) const;

    // printResult — prints a formatted summary of a SolverResult to stdout
    static void printResult(const SolverResult& result) noexcept;

private:
    // Members always initialized via constructor — no in-class defaults needed.
    CycleEvaluator  m_evaluator;   // cycle evaluator black box
    int             m_maxIter;     // maximum iterations
    double          m_tolerance;   // convergence tolerance on ||F||
    double          m_h_PR;        // finite-difference step for PR unknowns
    double          m_h_W;         // finite-difference step for W unknown

    // computeJacobian — approximates the 3x3 Jacobian by forward finite
    // differences. Requires 3 additional cycle evaluations per call.
    // J[i][j] = (F_i(x + h*e_j) - F_i(x)) / h
    std::array<std::array<double,3>,3>
    computeJacobian(const std::array<double,3>& x,
                    const std::array<double,3>& F) const;

    // solveLinearSystem — solves J * delta = -F by Gaussian elimination
    // with partial pivoting. Returns the update vector delta_x.
    // Returns a zero vector if J is singular (degenerate cycle state).
    static std::array<double,3>
    solveLinearSystem(std::array<std::array<double,3>,3> J,
                      std::array<double,3> F) noexcept;

    // norm — computes the L2 norm of a 3-vector
    static double norm(const std::array<double,3>& v) noexcept;
};

#endif // SOLVER_H
