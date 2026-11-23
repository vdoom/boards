// Temperature: bias drift from a stationary warm-up or cool-down (calib.py, last part).
#pragma once

#include "core/calib/errors.h"

#include <Eigen/Core>

#include <optional>

namespace ins10::calib {

struct ThermalCalibration {
    double tRef = 25.0;
    Eigen::Vector3d gyroBias = Eigen::Vector3d::Zero();  // dps at tRef
    Eigen::Matrix3d gyroTc = Eigen::Matrix3d::Zero();    // [axis][k]: dps per degC^(k+1)
    Eigen::Matrix3d accelTc = Eigen::Matrix3d::Zero();   // g per degC^(k+1)
    double accelTRef = 25.0;
    int order = 1;
    double tMin = 0.0, tMax = 0.0;
    double gyroResidualDps = 0.0;
    double gyroDriftDps = 0.0;    // peak-to-peak over the recording, before compensation
    double accelResidualMg = 0.0;
    double accelDriftMg = 0.0;
    int samples = 0;
    // Full polynomial coefficients (order + 1 rows, 3 columns) for plotting the fit.
    Eigen::MatrixXd gyroCoef, accelCoef;
    Warnings warnings;
};

// Least-squares polynomial in d per column of y, with one round of 4-sigma clipping.
// Returns the coefficients ((order + 1) x 3) and the rows kept at the end.
struct ClippedFit {
    Eigen::MatrixXd coef;
    Eigen::MatrixX3d keptResiduals;
};
ClippedFit polyfitClipped(const Eigen::VectorXd &d, const Eigen::MatrixX3d &y, int order);

// Polynomial order from the temperature span: 1 below 8 C, 2 below 20 C, else 3.
int thermalOrder(double spanC);

ThermalCalibration calibrateThermal(const Eigen::VectorXd &temp, const Eigen::MatrixX3d &gyro,
                                    const Eigen::MatrixX3d &accel, std::optional<double> accelTRef = std::nullopt);

} // namespace ins10::calib
