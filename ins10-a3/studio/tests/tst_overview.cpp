// Overview scene: in --demo the board in the scene follows the simulated trajectory,
// and the angles and heading computed from the scene's quaternion agree with the
// fields of $ATT to 0.02 deg.
#include "core/scene.h"
#include "device/fakeboard.h"
#include "device/ports.h"
#include "device/session.h"

#include <QTest>

#include <cmath>
#include <cstring>

using namespace ins10;

class TestOverview : public QObject {
    Q_OBJECT
private slots:
    void sceneFollowsTheDemoTrajectory() {
        FakeBoardConfig config;
        config.realTime = false;
        config.motion = FakeBoardConfig::MotionKind::Demo;
        FakeBoard board(config);
        FakeTransportFactory factory(&board);
        SessionConfig sc;
        sc.settleMs = 5;
        sc.batchIntervalMs = 5;
        DeviceSession session(&factory, sc);
        QList<AttRecord> records;
        connect(&session, &DeviceSession::streamBatch, this, [&](const StreamBatch &batch) {
            for (const ParsedLine &line : batch) {
                if (line.kind == LineKind::Att) records.append(line.as<AttRecord>());
            }
        });
        session.connectTo(kDemoPortName);
        QTRY_VERIFY(session.state() == ConnectionState::Connected && session.queuedCommands() == 0);

        // Two 40 s cycles of the choreography at 50 Hz, ending at 81 s: one second into
        // a rest, so that the last record and the board's attitude now are the same.
        for (int i = 0; i < 162; ++i) {
            board.advance(0.5);
            QCoreApplication::processEvents();
        }
        QTRY_VERIFY(records.size() > 4000);
        double worstAngle = 0.0, worstHeading = 0.0, maxPitch = 0.0;
        int checked = 0;
        for (const AttRecord &r : records) {
            const SceneAttitude s = sceneAttitude(r);
            QVERIFY(s.valid);
            maxPitch = std::max(maxPitch, std::abs(s.euler.pitchDeg));
            worstAngle = std::max({worstAngle, std::abs(angleDifference(s.euler.rollDeg, r.rollDeg)),
                                   std::abs(s.euler.pitchDeg - r.pitchDeg), std::abs(angleDifference(s.euler.yawDeg, r.yawDeg))});
            worstHeading = std::max(worstHeading, std::abs(angleDifference(s.headingDeg, r.headingDeg)));
            ++checked;
        }
        qInfo("%d records, worst angle difference %.4f deg, heading %.4f deg, |pitch| up to %.1f deg", checked,
              worstAngle, worstHeading, maxPitch);
        QVERIFY(worstAngle <= 0.02);
        QVERIFY(worstHeading <= 0.02);
        QVERIFY(maxPitch > 15.0); // the trajectory really moved

        // The last attitude shown is the simulated board's true attitude.
        const Eigen::Vector4d truth = board.trueAttitude().normalized();
        const SceneAttitude last = sceneAttitude(records.last());
        const double dot = std::abs(truth.dot(last.q));
        QVERIFY2(dot > 1.0 - 1e-6, qPrintable(QString::number(dot, 'g', 12)));
    }

    void bodyXPointsAlongTheHeading() {
        // Arbitrary attitudes: the scene's body X axis, seen from above, has the
        // azimuth the heading formula gives.
        for (double yaw : {0.0, 37.0, 90.0, 181.0, -75.0}) {
            for (double pitch : {-30.0, 0.0, 20.0}) {
                const double y = yaw * M_PI / 180.0, p = pitch * M_PI / 180.0, r = 0.3;
                const Eigen::Quaterniond q = Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()) *
                                             Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
                                             Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX());
                const Eigen::Vector4d v(q.w(), q.x(), q.y(), q.z());
                const Eigen::Vector3d x = bodyToWorld(v).topLeftCorner<3, 3>() * Eigen::Vector3d::UnitX();
                double azimuth = std::atan2(x[0], x[1]) * 180.0 / M_PI;
                if (azimuth < 0) azimuth += 360.0;
                QVERIFY(std::abs(angleDifference(azimuth, quatToHeading(v))) < 1e-9);
                QVERIFY(std::abs(quatToEuler(v).yawDeg - yaw) < 1e-9 || std::abs(angleDifference(quatToEuler(v).yawDeg, yaw)) < 1e-9);
            }
        }
    }

    // The drawing is the real INS10-A3 in the IMU's axes: J1 along the long edge at -Y
    // (pin 1 at -X), the INT pads along the long edge at +Y, the magnetometer at -X and
    // the barometer at +X of the IMU. (An earlier drawing had the header on a short edge.)
    void boardDrawingMatchesTheRealBoard() {
        const auto &parts = boardParts();
        const auto find = [&](const char *ref) -> BoardPart {
            for (const BoardPart &p : parts)
                if (std::strcmp(p.reference, ref) == 0) return p;
            return {BoardPart::Kind::Chip, "missing", 1e9, 1e9, 0, 0, 0};
        };
        QList<BoardPart> j1;
        for (const BoardPart &p : parts) {
            // Everything inside the outline.
            QVERIFY2(std::abs(p.x) + p.sizeX / 2 <= kBoardLengthMm / 2 && std::abs(p.y) + p.sizeY / 2 <= kBoardWidthMm / 2,
                     p.reference);
            if (std::strcmp(p.reference, "J1") == 0) j1.append(p);
        }
        QCOMPARE(j1.size(), 8);
        for (int i = 0; i < 8; ++i) {
            QVERIFY(std::abs(j1[i].y - (-5.35)) < 1e-9);
            QVERIFY(std::abs(j1[i].x - (-8.89 + 2.54 * i)) < 1e-9); // 2.54 mm pitch, pin 1 at -X
        }
        for (const char *ref : {"J2", "J3", "J4"}) QVERIFY(std::abs(find(ref).y - 5.35) < 1e-9);
        QVERIFY(find("J2").x < find("J3").x && find("J3").x < find("J4").x);
        QVERIFY(std::abs(find("J2").x - j1.first().x) < 1e-9 && std::abs(find("J4").x - j1.last().x) < 1e-9);
        const BoardPart imu = find("U1"), mag = find("U2"), baro = find("U3");
        QVERIFY(mag.x < imu.x && imu.x < baro.x);
        QVERIFY(std::abs(imu.x) < 0.5 && std::abs(imu.y) < 0.5); // the IMU sits near the middle
    }

    void angleDifferenceWraps() {
        QCOMPARE(angleDifference(359.99, 0.01), -0.02 + 0.0 * 0);
        QVERIFY(std::abs(angleDifference(359.99, 0.01) + 0.02) < 1e-9);
        QVERIFY(std::abs(angleDifference(-179.99, 179.99) - 0.02) < 1e-9);
        QCOMPARE(angleDifference(10.0, 10.0), 0.0);
    }
};

QTEST_GUILESS_MAIN(TestOverview)
#include "tst_overview.moc"
