// Attitude conventions against values worked out from the firmware's formulas.
#include "core/attitude.h"

#include <QTest>

#include <cmath>
#include <numbers>

using namespace ins10;

class TestAttitude : public QObject {
    Q_OBJECT
private slots:
    void identityPointsEast() {
        const Eigen::Vector4d q(1, 0, 0, 0);
        const EulerAngles e = quatToEuler(q);
        QCOMPARE(e.rollDeg, 0.0);
        QCOMPARE(e.pitchDeg, 0.0);
        QCOMPARE(e.yawDeg, 0.0);
        QCOMPARE(quatToHeading(q), 90.0);
    }

    void yawTurnsCounterClockwise() {
        // +90 deg about Up: body X now points north, heading 0, yaw +90.
        const double h = std::sqrt(0.5);
        const Eigen::Vector4d q(h, 0, 0, h);
        QVERIFY(std::abs(quatToEuler(q).yawDeg - 90.0) < 1e-9);
        const double heading = quatToHeading(q);
        QVERIFY(heading < 1e-9 || heading > 360.0 - 1e-9);
        // +180 deg: body X points west.
        QVERIFY(std::abs(quatToHeading(Eigen::Vector4d(0, 0, 0, 1)) - 270.0) < 1e-9);
    }

    void rollAndPitch() {
        const double a = 30.0 * std::numbers::pi / 180.0;
        const Eigen::Vector4d roll(std::cos(a / 2), std::sin(a / 2), 0, 0);
        QVERIFY(std::abs(quatToEuler(roll).rollDeg - 30.0) < 1e-9);
        const Eigen::Vector4d pitch(std::cos(a / 2), 0, std::sin(a / 2), 0);
        QVERIFY(std::abs(quatToEuler(pitch).pitchDeg - 30.0) < 1e-9);
    }

    void rotvecMatchesQuaternion() {
        const Eigen::Vector3d v(0.3, -0.2, 0.5);
        const Eigen::Matrix3d r = rotvecToMatrix(v);
        const Eigen::Matrix3d reference = Eigen::AngleAxisd(v.norm(), v.normalized()).toRotationMatrix();
        QVERIFY((r - reference).cwiseAbs().maxCoeff() < 1e-12);
        // Small angles use the series and stay accurate.
        const Eigen::Vector3d tiny(1e-9, -2e-9, 3e-9);
        QVERIFY((rotvecToMatrix(tiny) - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff() < 1e-8);
    }

    void productComposes() {
        const Eigen::Vector4d a = quatFromRotvec(Eigen::Vector3d(0.1, 0.2, -0.3));
        const Eigen::Vector4d b = quatFromRotvec(Eigen::Vector3d(-0.4, 0.05, 0.2));
        const Eigen::Matrix3d ab = quatToDcm(quatMultiply(a, b));
        QVERIFY((ab - quatToDcm(a) * quatToDcm(b)).cwiseAbs().maxCoeff() < 1e-12);
    }
};

QTEST_GUILESS_MAIN(TestAttitude)
#include "tst_attitude.moc"
