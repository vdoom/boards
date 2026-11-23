#include "core/attitude.h"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace ins10 {

namespace {
constexpr double kRad2Deg = 180.0 / std::numbers::pi;
}

Eigen::Matrix3d quatToDcm(const Eigen::Vector4d &q) {
    const double w = q[0], x = q[1], y = q[2], z = q[3];
    Eigen::Matrix3d r;
    r(0, 0) = 1.0 - 2.0 * (y * y + z * z);
    r(0, 1) = 2.0 * (x * y - w * z);
    r(0, 2) = 2.0 * (x * z + w * y);
    r(1, 0) = 2.0 * (x * y + w * z);
    r(1, 1) = 1.0 - 2.0 * (x * x + z * z);
    r(1, 2) = 2.0 * (y * z - w * x);
    r(2, 0) = 2.0 * (x * z - w * y);
    r(2, 1) = 2.0 * (y * z + w * x);
    r(2, 2) = 1.0 - 2.0 * (x * x + y * y);
    return r;
}

EulerAngles quatToEuler(const Eigen::Vector4d &q) {
    const double w = q[0], x = q[1], y = q[2], z = q[3];
    EulerAngles e;
    e.rollDeg = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)) * kRad2Deg;
    e.pitchDeg = std::asin(std::clamp(2.0 * (w * y - x * z), -1.0, 1.0)) * kRad2Deg;
    e.yawDeg = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)) * kRad2Deg;
    return e;
}

double quatToHeading(const Eigen::Vector4d &q) {
    const Eigen::Matrix3d r = quatToDcm(q);
    const double heading = std::atan2(r(0, 0), r(1, 0)) * kRad2Deg;
    return heading < 0.0 ? heading + 360.0 : heading;
}

Eigen::Vector4d quatMultiply(const Eigen::Vector4d &a, const Eigen::Vector4d &b) {
    return {a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
            a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
            a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
            a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]};
}

Eigen::Vector4d quatFromRotvec(const Eigen::Vector3d &v) {
    const double angle = v.norm();
    double scale; // sin(angle / 2) / angle
    if (angle <= 1e-3) {
        const double a2 = angle * angle;
        scale = 0.5 - a2 / 48.0 + a2 * a2 / 3840.0;
    } else {
        scale = std::sin(0.5 * angle) / angle;
    }
    return {std::cos(0.5 * angle), scale * v[0], scale * v[1], scale * v[2]};
}

Eigen::Matrix3d rotvecToMatrix(const Eigen::Vector3d &v) {
    return quatToDcm(quatFromRotvec(v));
}

Eigen::Vector4d quatNormalized(const Eigen::Vector4d &q) {
    const double n = q.norm();
    if (!(n > 1e-12) || !std::isfinite(n)) return {1.0, 0.0, 0.0, 0.0};
    return q / n;
}

} // namespace ins10
