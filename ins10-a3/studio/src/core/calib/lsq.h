// Non-linear least squares for the calibration fits: Levenberg-Marquardt with a
// forward-difference Jacobian, and the soft_l1 robust loss by iterative reweighting.
//
// It minimises the same function as scipy.optimize.least_squares:
//   linear:  0.5 * sum r^2
//   soft_l1: fScale^2 * sum (sqrt(1 + z) - 1),  z = (r / fScale)^2
// For soft_l1 each residual and Jacobian row is weighted by (1 + z)^(-1/4), which gives
// the robust cost's gradient exactly; steps are accepted or rejected on the true
// robust cost, not on the weighted sum of squares.
#pragma once

#include "core/calib/errors.h"

#include <Eigen/Core>

#include <atomic>
#include <functional>

namespace ins10::calib {

enum class Loss { Linear, SoftL1 };

struct LsqOptions {
    Loss loss = Loss::Linear;
    double fScale = 1.0;
    int maxIterations = 200;
    double tolerance = 1e-10;               // relative change of cost and of x
    const std::atomic<bool> *cancel = nullptr;
};

struct LsqResult {
    Eigen::VectorXd x;
    Eigen::VectorXd residuals;  // at x, unweighted (scipy: fun)
    Eigen::MatrixXd jacobian;   // at x, unweighted (scipy: jac for the linear loss)
    double cost = 0.0;
    int iterations = 0;
    int evaluations = 0;
    bool converged = false;
};

using ResidualFunction = std::function<Eigen::VectorXd(const Eigen::VectorXd &)>;

LsqResult leastSquares(const ResidualFunction &residual, const Eigen::VectorXd &x0, const LsqOptions &options = {});

// Forward differences, step sqrt(eps) * max(1, |x_j|), as scipy's '2-point'.
Eigen::MatrixXd numericJacobian(const ResidualFunction &residual, const Eigen::VectorXd &x, const Eigen::VectorXd &r,
                                int *evaluations = nullptr);

} // namespace ins10::calib
