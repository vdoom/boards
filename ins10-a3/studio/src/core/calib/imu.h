// Accelerometer and gyroscope: multi-position calibration (calib.py, second part).
#pragma once

#include "core/calib/errors.h"
#include "core/calib/lsq.h"

#include <Eigen/Core>

#include <optional>
#include <vector>

namespace ins10::calib {

struct StaticInterval {
    Eigen::Index start = 0;   // first IMU row
    Eigen::Index stop = 0;    // one past the last row
    double duration = 0.0;    // s
    Eigen::Vector3d accel;    // mean, g
    Eigen::Vector3d gyro;     // mean, dps
    double temp = 0.0;
};

struct StaticOptions {
    double windowS = 0.5;
    double gyroStdDps = 0.3;
    double accelStdG = 0.006;
    double minDurationS = 1.5;
};

// Periods during which the board lay still, with the mean readings of each.
std::vector<StaticInterval> findStaticIntervals(const Eigen::VectorXd &t, const Eigen::VectorXd &dt,
                                                const Eigen::MatrixX3d &gyro, const Eigen::MatrixX3d &accel,
                                                const Eigen::VectorXd &temp, const StaticOptions &options = {});

// Number of mutually distinct directions (more than minAngleDeg apart).
int countOrientations(const Eigen::MatrixX3d &directions, double minAngleDeg = 25.0);

struct AccelCalibration {
    Eigen::Vector3d bias;
    Eigen::Matrix3d matrix;
    double residualMg = 0.0;
    double residualBeforeMg = 0.0;
    int orientations = 0;
};

// Bias and scale/cross-axis matrix from the mean readings in several rest positions.
AccelCalibration calibrateAccel(const Eigen::MatrixX3d &means, const std::atomic<bool> *cancel = nullptr);

struct GyroCalibration {
    Eigen::Vector3d bias;
    Eigen::Matrix3d matrix;
    std::optional<double> residualDeg;
    std::optional<double> residualBeforeDeg;
    QString mode; // "full", "scale" or "none"
    int turns = 0;
};

// Gyro bias from the rest positions; scale and misalignment from the turns between
// them (Tedaldi, Pretto and Menegatti, ICRA 2014). contiguous[i] is false where rows
// i and i+1 are not consecutive; turns containing such a break are skipped.
GyroCalibration calibrateGyro(const Eigen::VectorXd &dt, const Eigen::MatrixX3d &gyro,
                              const std::vector<StaticInterval> &intervals, const Eigen::Vector3d &accelBias,
                              const Eigen::Matrix3d &accelMatrix, const std::vector<bool> *contiguous = nullptr,
                              const std::atomic<bool> *cancel = nullptr);

struct ImuCalibration {
    Eigen::Vector3d accelBias;
    Eigen::Matrix3d accelMatrix;
    double accelResidualMg = 0.0;        // 1-sigma of |a| - 1 g over the positions, after
    double accelResidualBeforeMg = 0.0;
    Eigen::Vector3d gyroBias;            // dps
    Eigen::Matrix3d gyroMatrix;
    std::optional<double> gyroResidualDeg;        // gravity-direction mismatch after the turns
    std::optional<double> gyroResidualBeforeDeg;
    QString gyroMatrixFitted;            // "full", "scale" or "none"
    int positions = 0;
    int orientations = 0;
    double tempC = 0.0;
    std::vector<StaticInterval> intervals;
    Warnings warnings;
};

ImuCalibration calibrateImu(const Eigen::VectorXd &t, const Eigen::VectorXd &dt, const Eigen::MatrixX3d &gyro,
                            const Eigen::MatrixX3d &accel, const Eigen::VectorXd &temp,
                            const std::vector<bool> *contiguous = nullptr, const std::atomic<bool> *cancel = nullptr);

} // namespace ins10::calib
