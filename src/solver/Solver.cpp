// =============================================================================
// Solver.cpp
// -----------------------------------------------------------------------------
// Newton-Raphson solver implementation for the GasTurbineEngineModel
// two-spool turbofan cycle.
//
// See Solver.h for the full description of the numerical method, design
// rationale, Jacobian strategy, and source references.
// =============================================================================

#include "Solver.h"

#include <algorithm>   // std::swap
#include <cmath>       // std::sqrt, std::fabs
#include <iostream>
#include <iomanip>

// =============================================================================
// Constructor
// =============================================================================

Solver::Solver(CycleEvaluator evaluator,
               int            maxIter,
               double         tolerance,
               double         h_PR,
               double         h_W)
    : m_evaluator(std::move(evaluator))
    , m_maxIter  (maxIter)
    , m_tolerance(tolerance)
    , m_h_PR     (h_PR)
    , m_h_W      (h_W)
{}

// =============================================================================
// solve
//
// Runs the Newton-Raphson iteration starting from x0.
//
// Algorithm per iteration:
//   1. Evaluate residual vector F at current x
//   2. Check convergence — if ||F|| < tolerance, return
//   3. Compute Jacobian J by finite differences (3 perturbed cycle runs)
//   4. Solve J * delta_x = -F by Gaussian elimination with partial pivoting
//   5. Apply update: x = x + delta_x
//   6. Repeat
//
// Residual normalisation:
//   Before computing ||F||, each residual is normalised so that all three
//   are dimensionless and comparable in magnitude:
//
//     F_norm[0] = F_hp  / F_hp_ref    where F_hp_ref  = 1.0e5  [J/kg]
//     F_norm[1] = F_lp  / F_lp_ref    where F_lp_ref  = 1.0e6  [W]
//     F_norm[2] = F_W   / F_W_ref     where F_W_ref   = 1.0    [kg/s]
//
//   These reference values reflect the typical magnitude of each residual
//   at the start of iteration. They prevent the mass flow residual (small
//   in absolute terms) from being swamped by the shaft work residuals
//   (large in absolute terms) when computing the norm.
// =============================================================================

SolverResult Solver::solve(const std::array<double,3>& x0) const
{
    // Reference scales for residual normalisation (see header comment)
    constexpr double F_hp_ref = 1.0e5;   // J/kg — HP shaft work scale
    constexpr double F_lp_ref = 1.0e6;   // W    — LP shaft power scale
    constexpr double F_W_ref  = 1.0;     // kg/s — mass flow residual scale

    SolverResult result;
    result.x = x0;

    for (int iter = 0; iter < m_maxIter; ++iter)
    {
        // --- Step 1: Evaluate residuals at current x ---
        const std::array<double,3> F = m_evaluator(result.x);

        // --- Step 2: Check convergence on normalised residual norm ---
        const std::array<double,3> F_norm = {
            F[0] / F_hp_ref,
            F[1] / F_lp_ref,
            F[2] / F_W_ref
        };

        result.residualNorm = norm(F_norm);
        result.iterations   = iter + 1;

        if (result.residualNorm < m_tolerance)
        {
            result.converged = true;
            return result;
        }

        // --- Step 3: Compute Jacobian by finite differences ---
        const auto J = computeJacobian(result.x, F);

        // --- Step 4: Solve J * delta_x = -F ---
        // Note: solveLinearSystem works on raw (un-normalised) F and J
        // so that the update delta_x is in physical units (PR, kg/s).
        const std::array<double,3> delta_x = solveLinearSystem(J, F);

        // --- Step 5: Apply update ---
        result.x[0] += delta_x[0];
        result.x[1] += delta_x[1];
        result.x[2] += delta_x[2];

        // Guard against non-physical values
        // PR must remain > 1.0; W must remain positive
        if (result.x[0] < 1.001) result.x[0] = 1.001;
        if (result.x[1] < 1.001) result.x[1] = 1.001;
        if (result.x[2] < 1.0)   result.x[2] = 1.0;
    }

    // Maximum iterations reached without convergence
    result.converged = false;
    return result;
}

// =============================================================================
// computeJacobian
//
// Approximates the 3x3 Jacobian matrix by forward finite differences.
//
// For each independent variable x_j, perturb by step h_j and re-run the
// full engine cycle. The j-th column of J is then:
//
//   J[:,j] = ( F(x + h_j * e_j) - F(x) ) / h_j
//
// This requires 3 additional cycle evaluations (one per unknown).
// The baseline F(x) is passed in to avoid a redundant 4th evaluation.
//
// Perturbation steps:
//   x[0] = PR_t_hp  →  h = m_h_PR  (relative — absolute on a ~2.4 value)
//   x[1] = PR_t_lp  →  h = m_h_PR
//   x[2] = W        →  h = m_h_W   (absolute [kg/s])
// =============================================================================

std::array<std::array<double,3>,3>
Solver::computeJacobian(const std::array<double,3>& x,
                         const std::array<double,3>& F) const
{
    // Step sizes for each unknown
    const std::array<double,3> h = { m_h_PR, m_h_PR, m_h_W };

    std::array<std::array<double,3>,3> J = {};

    for (int j = 0; j < 3; ++j)
    {
        // Perturb x_j
        std::array<double,3> x_pert = x;
        x_pert[j] += h[j];

        // Evaluate perturbed residual
        const std::array<double,3> F_pert = m_evaluator(x_pert);

        // Forward finite difference — column j of J
        for (int i = 0; i < 3; ++i)
        {
            J[i][j] = (F_pert[i] - F[i]) / h[j];
        }
    }

    return J;
}

// =============================================================================
// solveLinearSystem
//
// Solves the 3x3 linear system  J * delta = -F  using Gaussian elimination
// with partial pivoting (row pivoting for numerical stability).
//
// The system being solved is:
//
//   [ J_00  J_01  J_02 ] [ delta_0 ]   [ -F_0 ]
//   [ J_10  J_11  J_12 ] [ delta_1 ] = [ -F_1 ]
//   [ J_20  J_21  J_22 ] [ delta_2 ]   [ -F_2 ]
//
// Returns a zero update vector if J is singular (pivot < 1e-14), which
// prevents a divide-by-zero crash at the cost of stalling convergence.
// A singular Jacobian indicates a degenerate cycle state — typically caused
// by a very poor initial guess or a non-physical operating condition.
// =============================================================================

std::array<double,3>
Solver::solveLinearSystem(std::array<std::array<double,3>,3> J,
                           std::array<double,3>               F) noexcept
{
    // Form augmented matrix [J | -F]
    std::array<std::array<double,4>,3> aug;
    for (int i = 0; i < 3; ++i)
    {
        aug[i][0] = J[i][0];
        aug[i][1] = J[i][1];
        aug[i][2] = J[i][2];
        aug[i][3] = -F[i];      // RHS = -F
    }

    // Forward elimination with partial pivoting
    for (int col = 0; col < 3; ++col)
    {
        // Find pivot row — row with largest absolute value in this column
        int pivotRow = col;
        double pivotVal = std::fabs(aug[col][col]);
        for (int row = col + 1; row < 3; ++row)
        {
            if (std::fabs(aug[row][col]) > pivotVal)
            {
                pivotVal = std::fabs(aug[row][col]);
                pivotRow = row;
            }
        }

        // Singular or near-singular — return zero update
        if (pivotVal < 1.0e-14)
        {
            return {0.0, 0.0, 0.0};
        }

        // Swap rows if needed
        if (pivotRow != col)
        {
            std::swap(aug[col], aug[pivotRow]);
        }

        // Eliminate below pivot
        for (int row = col + 1; row < 3; ++row)
        {
            double factor = aug[row][col] / aug[col][col];
            for (int k = col; k < 4; ++k)
            {
                aug[row][k] -= factor * aug[col][k];
            }
        }
    }

    // Back substitution
    std::array<double,3> delta = {};
    for (int i = 2; i >= 0; --i)
    {
        delta[i] = aug[i][3];
        for (int j = i + 1; j < 3; ++j)
        {
            delta[i] -= aug[i][j] * delta[j];
        }
        delta[i] /= aug[i][i];
    }

    return delta;
}

// =============================================================================
// norm — L2 norm of a 3-vector
// =============================================================================

double Solver::norm(const std::array<double,3>& v) noexcept
{
    return std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

// =============================================================================
// printResult
//
// Prints a formatted summary of a SolverResult to stdout.
// Useful for debugging and for displaying solver performance in main.cpp.
// =============================================================================

void Solver::printResult(const SolverResult& result) noexcept
{
    std::cout << std::fixed;
    std::cout << "--- Newton-Raphson Solver Result ---\n";
    std::cout << "  Converged      : " << (result.converged ? "YES" : "NO") << "\n";
    std::cout << "  Iterations     : " << result.iterations << "\n";
    std::cout << "  Residual norm  : "
              << std::setprecision(6) << result.residualNorm << "\n";
    std::cout << "  PR_t_hp        : "
              << std::setprecision(6) << result.x[0] << "\n";
    std::cout << "  PR_t_lp        : "
              << std::setprecision(6) << result.x[1] << "\n";
    std::cout << "  W              : "
              << std::setprecision(4) << result.x[2] << " kg/s\n";
    std::cout << "\n";
}
