// Magnetometer calibration: ellipsoid fit (hard and soft iron), then alignment to the
// IMU frame. Port of the first part of tools/ins10tool/calib.py; names and results
// follow the Python code so the two can be compared.
//
// One deliberate difference: calib.py always aligns with a proper rotation, which
// assumes both sensors report right-handed axes. On the INS10-A3 the magnetometer's
// axes, as the firmware reports them, are a mirror image of the IMU's (NOTES.md
// item 17), and a rotation then turns the field upside down. Here the alignment may
// be a reflection; which of the two it is follows from the hemisphere (the field
// points down in the north). When both sensors are right-handed the results agree.
//
// Results are in the firmware's model: corrected = matrix * (raw - bias), raw in uT.
#pragma once

#include "core/calib/errors.h"
#include "core/calib/lsq.h"

#include <Eigen/Core>

#include <optional>

namespace ins10::calib {

// Where the board is used: whether the field points down (north) or up (south).
enum class Hemisphere { North, South };

struct MagCalibration {
    Eigen::Vector3d bias = Eigen::Vector3d::Zero();      // uT, hard iron
    Eigen::Matrix3d matrix = Eigen::Matrix3d::Identity(); // soft iron and rotation into the IMU frame
    double fieldUt = 0.0;                    // local field magnitude implied by the fit
    std::optional<double> dipDeg;            // positive when the field points down; none if not aligned
    double normResidualPct = 0.0;            // 1-sigma of |B| after calibration, % of the field
    std::optional<double> dipResidualDeg;
    double coverage = 0.0;                   // fraction of the sphere of directions sampled
    int samples = 0;
    bool aligned = false;
    bool mirrored = false;                   // aligned with a reflection: det(matrix) < 0
    Warnings warnings;
};

struct EllipsoidFit {
    Eigen::Vector3d center;
    Eigen::Matrix3d a; // symmetric positive definite, |a (x - center)| = 1
};

// _sym(p): parameters [xx, yy, zz, xy, xz, yz] to a symmetric matrix.
Eigen::Matrix3d symmetric(const double *p);

// Algebraic least-squares ellipsoid through the points (n, 3).
EllipsoidFit fitEllipsoid(const Eigen::MatrixX3d &x);

// Fraction of equal-area cells (bands of equal z x sectors of equal longitude) of the
// unit sphere hit by the given directions. Also gives the hit cells if asked.
double sphereCoverage(const Eigen::MatrixX3d &directions, int bands = 6, int sectors = 12,
                      std::vector<bool> *cells = nullptr);
// Cell index (band * sectors + sector) of one direction, -1 for a zero vector.
int sphereCell(const Eigen::Vector3d &direction, int bands = 6, int sectors = 12);

struct Alignment {
    Eigen::Matrix3d r;          // calibrated field directions -> accelerometer frame; orthogonal
    double sinDip = 0.0;        // positive when the field points down
    double residualDeg = 0.0;
    double conditioning = 0.0;
    bool mirrored = false;      // det(r) = -1
};

// Below this |sin(dip)| (about 10 degrees, near the magnetic equator) the hemisphere
// cannot tell the two signs apart; the alignment is then taken as a rotation.
constexpr double kMinSinDipForSign = 0.17;

// Orthogonal R with a_i . (R m_i) = const over the recording (see calib.py). The fit
// gives R and the constant up to a common sign; the sign is the one that makes the
// field point down in the north and up in the south, so R may be a reflection.
Alignment alignToAccel(const Eigen::MatrixX3d &magUnit, const Eigen::MatrixX3d &accel, const LsqOptions &options = {},
                       Hemisphere hemisphere = Hemisphere::North);

// mag (n, 3) uT; accel (n, 3) g and gyro (n, 3) dps at the same moments, optional.
MagCalibration calibrateMag(const Eigen::MatrixX3d &mag, const Eigen::MatrixX3d *accel = nullptr,
                            const Eigen::MatrixX3d *gyro = nullptr, const std::atomic<bool> *cancel = nullptr,
                            Hemisphere hemisphere = Hemisphere::North);

} // namespace ins10::calib
