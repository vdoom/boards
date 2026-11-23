// Small numpy equivalents used by the calibration code, with numpy's conventions
// (median of an even count averages the middle two, std divides by n, round() rounds
// half to even).
#pragma once

#include <Eigen/Core>

#include <vector>

namespace ins10::calib {

double median(std::vector<double> values);
double median(const Eigen::VectorXd &values);
// Robust RMS: sqrt(median(r^2) * 2.198).
double robustRms(const Eigen::VectorXd &residuals);
// Peak to peak.
double ptp(const Eigen::VectorXd &values);
double roundHalfEven(double value);
// Column-wise median of an (n, 3) array.
Eigen::RowVector3d columnMedian(const Eigen::MatrixX3d &values);
// Column-wise mean and standard deviation (divisor n) of rows [i0, i1).
Eigen::RowVector3d columnMean(const Eigen::MatrixX3d &values, Eigen::Index i0, Eigen::Index i1);
Eigen::RowVector3d columnStd(const Eigen::MatrixX3d &values, Eigen::Index i0, Eigen::Index i1);
// Row norms.
Eigen::VectorXd rowNorms(const Eigen::MatrixX3d &values);
// Degrees <-> radians as numpy does it (x * (pi / 180)).
double radians(double degrees);
double degrees(double radians);

} // namespace ins10::calib
