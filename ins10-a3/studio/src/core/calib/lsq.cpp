#include "core/calib/lsq.h"

#include <QCoreApplication>

#include <Eigen/Cholesky>
#include <Eigen/SVD>

#include <cmath>
#include <limits>

namespace ins10::calib {

namespace {
struct Text {
    Q_DECLARE_TR_FUNCTIONS(ins10::Calibration)
};

constexpr double kEps = std::numeric_limits<double>::epsilon();

double robustCost(const Eigen::VectorXd &r, const LsqOptions &o) {
    if (o.loss == Loss::Linear) return 0.5 * r.squaredNorm();
    const double f2 = o.fScale * o.fScale;
    double sum = 0.0;
    for (Eigen::Index i = 0; i < r.size(); ++i) sum += std::sqrt(1.0 + r[i] * r[i] / f2) - 1.0;
    return f2 * sum;
}

// Per-residual weights of the reweighted problem.
Eigen::VectorXd weights(const Eigen::VectorXd &r, const LsqOptions &o) {
    if (o.loss == Loss::Linear) return Eigen::VectorXd::Ones(r.size());
    const double f2 = o.fScale * o.fScale;
    Eigen::VectorXd w(r.size());
    for (Eigen::Index i = 0; i < r.size(); ++i) w[i] = std::pow(1.0 + r[i] * r[i] / f2, -0.25);
    return w;
}

void checkCancel(const LsqOptions &o) {
    if (o.cancel && o.cancel->load(std::memory_order_relaxed))
        throw CalibrationError(CalibrationError::Code::Cancelled, Text::tr("cancelled"));
}
} // namespace

Eigen::MatrixXd numericJacobian(const ResidualFunction &residual, const Eigen::VectorXd &x, const Eigen::VectorXd &r,
                                int *evaluations) {
    const double rstep = std::sqrt(kEps);
    Eigen::MatrixXd jac(r.size(), x.size());
    Eigen::VectorXd xh = x;
    for (Eigen::Index j = 0; j < x.size(); ++j) {
        const double sign = x[j] >= 0.0 ? 1.0 : -1.0;
        const double h0 = rstep * sign * std::max(1.0, std::abs(x[j]));
        xh[j] = x[j] + h0;
        const double h = xh[j] - x[j]; // the step actually taken
        const Eigen::VectorXd rh = residual(xh);
        if (evaluations) ++*evaluations;
        jac.col(j) = (rh - r) / h;
        xh[j] = x[j];
    }
    return jac;
}

LsqResult leastSquares(const ResidualFunction &residual, const Eigen::VectorXd &x0, const LsqOptions &options) {
    LsqResult out;
    Eigen::VectorXd x = x0;
    Eigen::VectorXd r = residual(x);
    out.evaluations = 1;
    const Eigen::Index n = x.size();
    double cost = robustCost(r, options);
    Eigen::MatrixXd jac = numericJacobian(residual, x, r, &out.evaluations);

    Eigen::MatrixXd a;
    Eigen::VectorXd g;
    auto normalEquations = [&] {
        const Eigen::VectorXd w = weights(r, options);
        const Eigen::MatrixXd jw = w.asDiagonal() * jac;
        a = jw.transpose() * jw;
        g = jw.transpose() * (w.asDiagonal() * r);
    };
    normalEquations();

    double lambda = 1e-3 * std::max(a.diagonal().maxCoeff(), kEps);
    double nu = 2.0;
    int iteration = 0;
    while (iteration < options.maxIterations) {
        checkCancel(options);
        ++iteration;
        if (!(cost > 0.0) || g.lpNorm<Eigen::Infinity>() <= 1e-15 * std::max(1.0, cost)) {
            out.converged = true;
            break;
        }
        Eigen::MatrixXd damped = a;
        for (Eigen::Index i = 0; i < n; ++i) damped(i, i) += lambda * std::max(a(i, i), 1e-12 * a.diagonal().maxCoeff() + kEps);
        Eigen::VectorXd step = damped.ldlt().solve(-g);
        if (!step.allFinite()) step = damped.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(-g);
        const double stepNorm = step.norm();
        const bool tinyStep = stepNorm <= options.tolerance * (x.norm() + options.tolerance);
        if (!step.allFinite()) {
            lambda *= nu;
            nu *= 2.0;
            continue;
        }
        const Eigen::VectorXd xNew = x + step;
        const Eigen::VectorXd rNew = residual(xNew);
        ++out.evaluations;
        const double costNew = rNew.allFinite() ? robustCost(rNew, options) : std::numeric_limits<double>::infinity();
        if (costNew < cost) {
            // Gain ratio against the local quadratic model (Nielsen's update of lambda).
            const double modelDecrease = -(g.dot(step) + 0.5 * step.dot(a * step));
            const double rho = modelDecrease > 0.0 ? (cost - costNew) / modelDecrease : 0.0;
            const bool smallDecrease = cost - costNew <= options.tolerance * cost;
            x = xNew;
            r = rNew;
            cost = costNew;
            jac = numericJacobian(residual, x, r, &out.evaluations);
            normalEquations();
            lambda *= std::max(1.0 / 3.0, 1.0 - std::pow(2.0 * rho - 1.0, 3));
            lambda = std::max(lambda, 1e-300);
            nu = 2.0;
            if (smallDecrease || tinyStep) {
                out.converged = true;
                break;
            }
        } else {
            lambda *= nu;
            nu *= 2.0;
            if (tinyStep || lambda > 1e300) {
                out.converged = tinyStep;
                break;
            }
        }
    }
    out.x = x;
    out.residuals = r;
    out.jacobian = jac;
    out.cost = cost;
    out.iterations = iteration;
    return out;
}

} // namespace ins10::calib
