// Attitude conventions of the ins10_ahrs firmware (lib/ins10/src/math3.c, fusion.c).
//
// The quaternion {w, x, y, z} is Hamilton and rotates body vectors into the
// East-North-Up navigation frame. Identity attitude: body X points east, heading 90 deg.
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ins10 {

struct EulerAngles {
    double rollDeg = 0.0;
    double pitchDeg = 0.0;
    double yawDeg = 0.0;
};

// Body-to-ENU rotation matrix of q = {w, x, y, z} (ins10_quat_to_dcm). q need not be
// normalised exactly; the formula is the firmware's.
Eigen::Matrix3d quatToDcm(const Eigen::Vector4d &q);

// ZYX Euler angles, as ins10_quat_to_euler.
EulerAngles quatToEuler(const Eigen::Vector4d &q);

// Compass heading of the body X axis, 0..360 deg clockwise from north:
// atan2(R00, R10), as ins10_fusion_get.
double quatToHeading(const Eigen::Vector4d &q);

// Hamilton product a * b, both {w, x, y, z}.
Eigen::Vector4d quatMultiply(const Eigen::Vector4d &a, const Eigen::Vector4d &b);

// Unit quaternion {w, x, y, z} of the rotation vector v (rad), with the small-angle
// series used by SciPy's Rotation.from_rotvec.
Eigen::Vector4d quatFromRotvec(const Eigen::Vector3d &v);

// Rotation matrix of a rotation vector (rad): exp of its skew matrix.
Eigen::Matrix3d rotvecToMatrix(const Eigen::Vector3d &v);

// Normalised copy; identity for a zero or non-finite input.
Eigen::Vector4d quatNormalized(const Eigen::Vector4d &q);

} // namespace ins10
