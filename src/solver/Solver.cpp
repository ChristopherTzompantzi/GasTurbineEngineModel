// =============================================================================
// Solver.cpp
// -----------------------------------------------------------------------------
// Newton-Raphson solver implementation for the GasTurbineEngineModel.
// Generalised N×N system — handles any engine configuration at runtime.
//
// See Solver.h for the full description of the numerical method, design
// rationale, Jacobian strategy, physical guards, and source references.
// =============================================================================

#include "Solver.h"

#include <algorithm>   // std::swap
#include <cmath>       // std::sqrt, std::fabs
#include <iostream>
#include <iomanip>

// =============================================================================
// Constructor
//
// Stores the cycle evaluator lambda, residual scale factors, and solver
// tuning parameters. scales_ must have the same size N as the x0 vector
// passed to solve() — N is not validated here but mismatches will produce
// incorrect normalisation at runtime.
// =============================================================================

Solver::Solver(CycleEvaluator      evaluator,
               std::vector<double> scales,
               int                 maxIter,
               double              tolerance,
               double              h_rel,
               double              h_abs)
    : evaluator_ (std::move(evaluator))
    , scales_    (std::move(scales))
    , maxIter_   (maxIter)
    , tolerance_ (tolerance)
    , h_rel_     (h_rel)
    , h_abs_     (h_abs)
{}

// =============================================================================
// solve
//
// Runs the Newton-Raphson iteration starting from x0.
// N is inferred from x0.size() — scales_ must have been initialised to
// the same N in the constructor.
//
// Algorithm per iteration:
//   1. Evaluate residual vector F at current x  (1 cycle run)
//   2. Normalise: F_norm[i] = F[i] / scales_[i]
//   3. Check convergence: ||F_norm||_2 < tolerance_
//   4. Compute N×N Jacobian J by forward finite differences  (N cycle runs)
//   5. Solve J·dx = -F by Gaussian elimination with partial pivoting
//   6. Apply physical guards and update x = x + dx
//   7. Repeat
//
// Total cycle evaluations per iteration: N+1
//   (1 for residual + N for Jacobian columns)
//
// Physical guards (applied after each Newton step):
//   scales_[i] >= 1e4  →  pressure ratio variable  →  clamped to > 1.001
//   scales_[i] <  1e4  →  mass flow variable        →  clamped to > 1.0
// =============================================================================

SolverResult Solver::solve(const std::vector<double>& x0)
{
    const int N = static_cast<int>(x0.size());

    SolverResult result;
    result.x = x0;

    for (int iter = 0; iter < maxIter_; ++iter)
    {
        // Step 1 — Evaluate residuals at current x
        const std::vector<double> F = evaluator_(result.x);

        // Step 2 — Normalise residuals by caller-supplied scale factors
        std::vector<double> F_norm(N);
        for (int i = 0; i < N; ++i)
            F_norm[i] = F[i] / scales_[i];

        // Step 3 — Check convergence on normalised L2 norm
        result.residualNorm = norm(F_norm);
        result.iterations   = iter + 1;

        if (result.residualNorm < tolerance_)
        {
            result.converged = true;
            return result;
        }

        // Step 4 — Compute N×N Jacobian by forward finite differences
        // Passes F as baseline to avoid a redundant cycle evaluation
        const auto J = computeJacobian(result.x, F);

        // Step 5 — Solve J·dx = -F (raw, un-normalised)
        // dx is in physical units — PR [-] or W [kg/s]
        const std::vector<double> dx = solveLinearSystem(J, F);

        // Step 6 — Apply update with physical guards
        for (int i = 0; i < N; ++i)
        {
            result.x[i] += dx[i];

            // Clamp to physical bounds based on variable type
            // Pressure ratio: scales_[i] >= 1e4
            // Mass flow:      scales_[i] <  1e4
            if (scales_[i] >= 1.0e4)
            {
                if (result.x[i] < 1.001) result.x[i] = 1.001;
            }
            else
            {
                if (result.x[i] < 1.0) result.x[i] = 1.0;
            }
        }
    }

    // Maximum iterations reached without convergence
    result.converged = false;
    return result;
}

// =============================================================================
// computeJacobian
//
// Builds the N×N Jacobian matrix by forward finite differences.
//
// For each independent variable x[j], perturbs by step h[j] and re-runs
// the full engine cycle. Column j of J:
//
//   J[:,j] = (F(x + h[j]*e_j) - F0) / h[j]
//
// Step selection per variable (based on residual scale):
//   scales_[j] >= 1e4  →  h = h_rel_   (pressure ratio — absolute step on ~2.0 value)
//   scales_[j] <  1e4  →  h = h_abs_   (mass flow — absolute step [kg/s])
//
// The baseline F0 is passed in from solve() to avoid a redundant evaluation.
// This requires exactly N additional cycle evaluations per Newton step.
// =============================================================================

std::vector<std::vector<double>>
Solver::computeJacobian(const std::vector<double>& x,
                         const std::vector<double>& F0)
{
    const int N = static_cast<int>(x.size());

    // Initialise N×N Jacobian to zero
    std::vector<std::vector<double>> J(N, std::vector<double>(N, 0.0));

    for (int j = 0; j < N; ++j)
    {
        // Select perturbation step based on variable type
        const double h = (scales_[j] >= 1.0e4) ? h_rel_ : h_abs_;

        // Perturb x[j] forward
        std::vector<double> x_pert = x;
        x_pert[j] += h;

        // Evaluate perturbed residual
        const std::vector<double> F_pert = evaluator_(x_pert);

        // Forward finite difference — column j of J
        for (int i = 0; i < N; ++i)
            J[i][j] = (F_pert[i] - F0[i]) / h;
    }

    return J;
}

// =============================================================================
// solveLinearSystem
//
// Solves the N×N linear system J·dx = -F using Gaussian elimination
// with partial pivoting (row pivoting for numerical stability).
//
// Forms the augmented matrix [J | -F] and applies:
//   1. Forward elimination with partial pivoting
//   2. Back substitution
//
// Singular pivot threshold: 1e-14. Returns a zero vector if the system
// is singular — stalls convergence rather than crashing. A singular
// Jacobian typically indicates a non-physical engine operating state
// or a very poor initial guess.
//
// @param J   N×N Jacobian (passed by value — modified during elimination)
// @param F   Residual vector [size N]
// @return    Solution vector dx [size N]
// =============================================================================

std::vector<double>
Solver::solveLinearSystem(std::vector<std::vector<double>> J,
                           const std::vector<double>&       F)
{
    const int N = static_cast<int>(F.size());

    // Form augmented matrix [J | -F]  (N rows, N+1 columns)
    std::vector<std::vector<double>> aug(N, std::vector<double>(N + 1, 0.0));
    for (int i = 0; i < N; ++i)
    {
        for (int j = 0; j < N; ++j)
            aug[i][j] = J[i][j];
        aug[i][N] = -F[i];   // RHS = -F
    }

    // Forward elimination with partial pivoting
    for (int col = 0; col < N; ++col)
    {
        // Find pivot row — maximum absolute value in this column
        int    pivotRow = col;
        double pivotVal = std::fabs(aug[col][col]);
        for (int row = col + 1; row < N; ++row)
        {
            if (std::fabs(aug[row][col]) > pivotVal)
            {
                pivotVal = std::fabs(aug[row][col]);
                pivotRow = row;
            }
        }

        // Singular or near-singular — return zero update vector
        if (pivotVal < 1.0e-14)
            return std::vector<double>(N, 0.0);

        // Swap rows if pivot is not already on the diagonal
        if (pivotRow != col)
            std::swap(aug[col], aug[pivotRow]);

        // Eliminate below pivot
        for (int row = col + 1; row < N; ++row)
        {
            const double factor = aug[row][col] / aug[col][col];
            for (int k = col; k <= N; ++k)
                aug[row][k] -= factor * aug[col][k];
        }
    }

    // Back substitution
    std::vector<double> dx(N, 0.0);
    for (int i = N - 1; i >= 0; --i)
    {
        dx[i] = aug[i][N];
        for (int j = i + 1; j < N; ++j)
            dx[i] -= aug[i][j] * dx[j];
        dx[i] /= aug[i][i];
    }

    return dx;
}

// =============================================================================
// norm
//
// Computes the L2 norm of a vector: ||v||_2 = sqrt(sum(v[i]^2))
// Used for convergence check on the normalised residual vector F_norm.
// =============================================================================

double Solver::norm(const std::vector<double>& v) noexcept
{
    double sum = 0.0;
    for (const double val : v)
        sum += val * val;
    return std::sqrt(sum);
}

// =============================================================================
// printResult
//
// Prints a formatted solver convergence summary to stdout.
//
// Outputs convergence status, iteration count, residual norm, and all N
// converged independent variable values. If labels are provided they are
// used as variable names — otherwise variables print as x[0], x[1], etc.
//
// Example output (turbojet, N=1, labels={"PR_t"}):
//   --- Newton-Raphson Solver Result ---
//     Converged      : YES
//     Iterations     : 3
//     Residual norm  : 0.000042
//     PR_t           : 2.312456
//
// Example output (turbofan, N=3, labels={"PR_t_hp","PR_t_lp","W"}):
//   --- Newton-Raphson Solver Result ---
//     Converged      : YES
//     Iterations     : 3
//     Residual norm  : 0.000016
//     PR_t_hp        : 2.362703
//     PR_t_lp        : 1.879607
//     W              : 60.000000
// =============================================================================

void Solver::printResult(const SolverResult&             result,
                          const std::vector<std::string>& labels) noexcept
{
    const int N = static_cast<int>(result.x.size());

    std::cout << std::fixed;
    std::cout << "--- Newton-Raphson Solver Result ---\n";
    std::cout << "  Converged      : "
              << (result.converged ? "YES" : "NO") << "\n";
    std::cout << "  Iterations     : " << result.iterations << "\n";
    std::cout << "  Residual norm  : "
              << std::setprecision(6) << result.residualNorm << "\n";

    for (int i = 0; i < N; ++i)
    {
        // Use provided label if available, otherwise generic x[i]
        std::string label = (i < static_cast<int>(labels.size()))
                          ? labels[i]
                          : ("x[" + std::to_string(i) + "]");

        // Pad label to 14 characters for column alignment
        while (static_cast<int>(label.size()) < 14) label += " ";

        std::cout << "  " << label << " : "
                  << std::setprecision(6) << result.x[i] << "\n";
    }
    std::cout << "\n";
}