#ifndef SOLVER_H
#define SOLVER_H

#include <functional>
#include <vector>
#include <iostream>
#include <iomanip>
#include <cmath>

/*
 * Solver.h
 * --------
 * Declares the Newton-Raphson solver for engine cycle balancing.
 * Solves a system of N nonlinear equations in N unknowns:
 *
 *   F(x) = 0
 *
 * where x is the vector of independent variables and F is the
 * vector of residuals returned by the cycle evaluator.
 *
 * GENERALISED N×N DESIGN:
 * The solver operates on std::vector<double> for both independent
 * variables and residuals. N is determined at runtime from the
 * size of the initial guess vector x0 passed to solve(). This
 * allows the same solver infrastructure to handle any engine
 * configuration:
 *   - Turbojet      : N=1  (PR_t only)
 *   - Two-spool TF  : N=3  (PR_t_hp, PR_t_lp, W)
 *   - Three-spool   : N=4+ (future)
 *
 * RESIDUAL NORMALISATION (caller-provided scales):
 * Each residual F[i] is divided by a caller-supplied scale factor
 * scales[i] before computing the convergence norm. This prevents
 * large-magnitude residuals (shaft power in Watts) from dominating
 * over small-magnitude ones (mass flow in kg/s).
 *
 * The caller chooses scales based on the physical units of each
 * residual. Typical values:
 *   HP shaft balance [J/kg]   → scale = 1e5
 *   LP shaft balance [W]      → scale = 1e6
 *   Mass flow [kg/s]          → scale = 1.0
 *   Single shaft [J/kg]       → scale = 1e5
 *
 * ALGORITHM:
 * 1. Evaluate F(x) — residuals at current x
 * 2. Check convergence: ||F_normalised||_2 < tol
 * 3. Compute Jacobian J via forward finite differences
 * 4. Build augmented matrix [J | -F]
 * 5. Solve J·dx = -F via Gaussian elimination with partial pivoting
 * 6. Apply physical guards: PR > 1.001, W > 1.0
 * 7. Update x = x + dx
 * 8. Repeat from step 1
 *
 * PHYSICAL GUARDS:
 * After each Newton step, independent variables are clamped to
 * prevent non-physical states during iteration:
 *   PR values > 1.001  — pressure ratio must be expansive
 *   W values  > 1.0    — mass flow must be positive and non-trivial
 *
 * The solver applies guards based on variable name conventions —
 * the caller documents which indices are PR values and which are W.
 * Guards are applied to all variables uniformly at the PR threshold
 * unless the variable is identified as a mass flow by the scales vector.
 *
 * CONVERGENCE:
 *   Tolerance : ||F_normalised||_2 < 1e-4
 *   Max iter  : 50
 *
 * JACOBIAN PERTURBATION STEPS:
 *   h_rel = 1e-4   — relative step for pressure ratio variables
 *   h_abs = 1e-3   — absolute step for mass flow variables [kg/s]
 *   Variables with scale >= 1e4 are treated as pressure ratios (h_rel)
 *   Variables with scale <  1e4 are treated as mass flows (h_abs)
 *
 * UNITS — SI throughout:
 *   Pressures    [Pa]
 *   Temperatures [K]
 *   Mass flow    [kg/s]
 *   Shaft work   [J/kg] or [W]
 *
 * NPSS ALIGNMENT:
 * This solver mirrors the Newton-Raphson solver used internally
 * by NPSS for cycle balancing. Independent variables and residuals
 * are defined by the cycle evaluator lambda in main.cpp — the
 * solver itself has no knowledge of engine elements.
 */

// =========================================================================
// RESULT STRUCT
// =========================================================================

/*
 * SolverResult — returned by solve() after convergence or max iterations.
 *
 * x            : converged independent variable vector [size N]
 * converged    : true if ||F_normalised||_2 < tol before max iterations
 * iterations   : number of Newton steps taken
 * residualNorm : final ||F_normalised||_2
 * N            : number of independent variables — inferred from x.size()
 */
struct SolverResult {
    std::vector<double> x;                  // Converged solution [size N]
    bool                converged    = false;
    int                 iterations   = 0;
    double              residualNorm = 0.0;
};

// =========================================================================
// SOLVER CLASS
// =========================================================================

class Solver {
public:

    /*
     * CycleEvaluator — callable that runs the engine cycle.
     *
     * Input:  x — independent variable vector [size N]
     * Output: F — residual vector [size N]
     *
     * The evaluator is provided by main.cpp as a lambda that captures
     * engine element objects by reference. The solver calls it on each
     * Newton iteration and for each column of the finite-difference
     * Jacobian. N is inferred from x0.size() at solve() time.
     */
    using CycleEvaluator = std::function<std::vector<double>(const std::vector<double>&)>;

    /*
     * Constructor
     *
     * @param evaluator  Cycle evaluator lambda — runs the engine cycle
     *                   and returns residuals for given independent vars.
     * @param scales     Residual scale factors [size N] — one per equation.
     *                   F[i] is divided by scales[i] before norm evaluation.
     *                   Caller provides based on physical units of residuals.
     *                   Example (two-spool turbofan): {1e5, 1e6, 1.0}
     *                   Example (turbojet):           {1e5}
     * @param maxIter    Maximum Newton iterations (default: 50)
     * @param tolerance  Convergence tolerance on normalised norm (default: 1e-4)
     * @param h_rel      Relative finite-difference step for PR variables (default: 1e-4)
     * @param h_abs      Absolute finite-difference step for W variables (default: 1e-3)
     */
    Solver(CycleEvaluator      evaluator,
           std::vector<double> scales,
           int                 maxIter   = 50,
           double              tolerance = 1.0e-4,
           double              h_rel     = 1.0e-4,
           double              h_abs     = 1.0e-3);

    /*
     * solve — runs Newton-Raphson iteration until convergence or max iter.
     *
     * @param x0  Initial guess vector [size N]
     * @return    SolverResult with converged x, convergence flag,
     *            iteration count, and final residual norm.
     *
     * N is inferred from x0.size(). scales must match N.
     * Physical guards applied after each step — see class comment.
     */
    SolverResult solve(const std::vector<double>& x0);

    /*
     * printResult — prints solver convergence summary to stdout.
     *
     * Prints converged/diverged status, iteration count, residual norm,
     * and all N converged independent variable values generically as
     * x[0], x[1], ... x[N-1]. Static — callable without a Solver instance.
     *
     * @param result  SolverResult returned by solve()
     * @param labels  Optional variable labels [size N] for readable output.
     *                If empty, variables are printed as x[0], x[1], etc.
     */
    static void printResult(const SolverResult&              result,
                            const std::vector<std::string>&  labels = {}) noexcept;

private:

    // =====================================================================
    // PRIVATE MEMBERS
    // =====================================================================

    CycleEvaluator      evaluator_;   // Cycle evaluator lambda
    std::vector<double> scales_;      // Residual normalisation factors [N]
    int                 maxIter_;     // Maximum Newton iterations
    double              tolerance_;   // Convergence tolerance on norm
    double              h_rel_;       // Finite-difference step — PR variables
    double              h_abs_;       // Finite-difference step — W variables

    // =====================================================================
    // PRIVATE METHODS
    // =====================================================================

    /*
     * computeJacobian — builds N×N finite-difference Jacobian matrix.
     *
     * For each independent variable x[j], perturbs by step h[j] and
     * re-runs the full cycle. Column j of J is:
     *
     *   J[:,j] = (F(x + h[j]*e_j) - F0) / h[j]
     *
     * The step h[j] is chosen based on the residual scale:
     *   scales[j] >= 1e4 → h = h_rel_ (pressure ratio variable)
     *   scales[j] <  1e4 → h = h_abs_ (mass flow variable)
     *
     * The baseline F0 is passed in to avoid a redundant evaluation.
     * This requires exactly N cycle evaluations for an N×N Jacobian.
     *
     * @param x   Current independent variable vector [size N]
     * @param F0  Residuals at x [size N] — reused to avoid redundant call
     * @return    J[N][N] — Jacobian matrix as vector of row vectors
     */
    std::vector<std::vector<double>>
    computeJacobian(const std::vector<double>& x,
                    const std::vector<double>& F0);

    /*
     * solveLinearSystem — solves J·dx = -F via Gaussian elimination
     * with partial pivoting.
     *
     * Takes the N×N Jacobian J and residual vector F, forms the
     * augmented matrix [J | -F], and applies forward elimination
     * followed by back substitution.
     *
     * Singular pivot threshold: 1e-14. Returns zero vector if singular
     * to prevent divide-by-zero — stalls convergence rather than crashing.
     * A singular Jacobian indicates a degenerate cycle state, typically
     * caused by a non-physical operating point or a very poor initial guess.
     *
     * @param J   N×N Jacobian matrix (passed by value — modified in place)
     * @param F   Residual vector [size N]
     * @return    Solution vector dx [size N]
     */
    std::vector<double>
    solveLinearSystem(std::vector<std::vector<double>> J,
                      const std::vector<double>&       F);

    /*
     * norm — computes L2 norm of a vector.
     * Used for convergence check on normalised residuals.
     *
     * @param v   Input vector [size N]
     * @return    ||v||_2 = sqrt(sum(v[i]^2))
     */
    double norm(const std::vector<double>& v) noexcept;
};

#endif // SOLVER_H