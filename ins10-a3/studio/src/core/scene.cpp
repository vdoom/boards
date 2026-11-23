#include "core/scene.h"

#include <cmath>

namespace ins10 {

SceneAttitude sceneAttitude(const AttRecord &record) {
    SceneAttitude s;
    const Eigen::Vector4d q(record.q[0], record.q[1], record.q[2], record.q[3]);
    const double n = q.norm();
    if (!(n > 0.5) || !std::isfinite(n)) return s;
    s.q = q / n;
    s.euler = quatToEuler(s.q);
    s.headingDeg = quatToHeading(s.q);
    s.valid = true;
    return s;
}

Eigen::Matrix4d bodyToWorld(const Eigen::Vector4d &q) {
    Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
    m.topLeftCorner<3, 3>() = quatToDcm(q);
    return m;
}

const std::vector<BoardPart> &boardParts() {
    // KiCad places the outline's top-left corner at (100, 100) mm with y pointing
    // down the drawing, J1 at the bottom; the body frame has its origin at the
    // outline's centre and y pointing up the drawing.
    static const std::vector<BoardPart> parts = [] {
        using K = BoardPart::Kind;
        std::vector<BoardPart> p;
        const auto add = [&p](K kind, const char *ref, double kx, double ky, double sx, double sy, double h,
                              int pin1 = -1) {
            p.push_back({kind, ref, kx - 100.0 - kBoardLengthMm / 2, 100.0 + kBoardWidthMm / 2 - ky, sx, sy, h, pin1});
        };
        add(K::Chip, "U1", 111.3, 106.8, 3.0, 2.5, 0.91, 3);      // ICM-42688-P, LGA-14 2.5 x 3
        add(K::Chip, "U2", 104.0, 106.9, 3.0, 3.0, 1.0, 0);       // MMC5983MA, LGA-16 3 x 3, turned 180 deg
        add(K::Chip, "U3", 119.0, 105.9, 2.0, 2.0, 0.73, 3);      // LPS22DF, HLGA-10 2 x 2
        add(K::Crystal, "Y1", 108.175, 102.25, 3.2, 2.5, 0.9, 0); // ECS-327MVATX-3, 3.2 x 2.5
        add(K::Capacitor, "C1", 114.2, 107.4, 0.5, 1.0, 0.5);     // 0402, turned 90 deg
        add(K::Capacitor, "C2", 114.3, 109.3, 1.6, 0.8, 0.8);     // 0603
        add(K::Capacitor, "C3", 110.9, 109.1, 1.0, 0.5, 0.5);
        add(K::Capacitor, "C4", 104.0, 109.45, 1.0, 0.5, 0.5);
        add(K::Capacitor, "C5", 104.375, 103.65, 2.0, 1.25, 1.25); // 0805
        add(K::Capacitor, "C6", 101.1, 106.4, 0.5, 1.0, 0.5);
        add(K::Capacitor, "C7", 116.55, 105.2, 0.5, 1.0, 0.5);
        add(K::Capacitor, "C8", 110.8, 102.4, 0.5, 1.0, 0.5);
        for (int pin = 0; pin < 8; ++pin) add(K::Pad, "J1", 102.36 + 2.54 * pin, 111.95, 1.8, 1.0, 0.0);
        add(K::Pad, "J2", 102.36, 101.25, 1.8, 1.0, 0.0); // MMC_INT
        add(K::Pad, "J3", 112.52, 101.25, 1.8, 1.0, 0.0); // ICM_INT
        add(K::Pad, "J4", 120.14, 101.25, 1.8, 1.0, 0.0); // LPS_INT
        return p;
    }();
    return parts;
}

double angleDifference(double a, double b) {
    double d = std::fmod(a - b, 360.0);
    if (d <= -180.0) d += 360.0;
    if (d > 180.0) d -= 360.0;
    return d;
}

} // namespace ins10
