#include "core/calib/stats.h"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace ins10::calib {

double median(std::vector<double> v) {
    if (v.empty()) return std::numeric_limits<double>::quiet_NaN();
    const size_t n = v.size();
    const size_t mid = n / 2;
    std::nth_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(mid), v.end());
    const double upper = v[mid];
    if (n % 2) return upper;
    const double lower = *std::max_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(mid));
    return 0.5 * (lower + upper);
}

double median(const Eigen::VectorXd &values) {
    return median(std::vector<double>(values.data(), values.data() + values.size()));
}

double robustRms(const Eigen::VectorXd &residuals) {
    return std::sqrt(median(Eigen::VectorXd(residuals.array().square())) * 2.198);
}

double ptp(const Eigen::VectorXd &values) {
    if (values.size() == 0) return 0.0;
    return values.maxCoeff() - values.minCoeff();
}

double roundHalfEven(double value) {
    const double r = std::round(value);
    if (std::abs(value - std::trunc(value)) == 0.5) return 2.0 * std::round(value / 2.0);
    return r;
}

Eigen::RowVector3d columnMedian(const Eigen::MatrixX3d &values) {
    Eigen::RowVector3d out;
    for (int c = 0; c < 3; ++c) out[c] = median(Eigen::VectorXd(values.col(c)));
    return out;
}

Eigen::RowVector3d columnMean(const Eigen::MatrixX3d &values, Eigen::Index i0, Eigen::Index i1) {
    Eigen::RowVector3d sum = Eigen::RowVector3d::Zero();
    for (Eigen::Index i = i0; i < i1; ++i) sum += values.row(i);
    return sum / static_cast<double>(i1 - i0);
}

Eigen::RowVector3d columnStd(const Eigen::MatrixX3d &values, Eigen::Index i0, Eigen::Index i1) {
    const Eigen::RowVector3d mean = columnMean(values, i0, i1);
    Eigen::RowVector3d sum = Eigen::RowVector3d::Zero();
    for (Eigen::Index i = i0; i < i1; ++i) {
        const Eigen::RowVector3d d = values.row(i) - mean;
        sum += d.cwiseProduct(d);
    }
    return (sum / static_cast<double>(i1 - i0)).cwiseSqrt();
}

Eigen::VectorXd rowNorms(const Eigen::MatrixX3d &values) { return values.rowwise().norm(); }

double radians(double degreesValue) { return degreesValue * (std::numbers::pi / 180.0); }
double degrees(double radiansValue) { return radiansValue * (180.0 / std::numbers::pi); }

} // namespace ins10::calib
