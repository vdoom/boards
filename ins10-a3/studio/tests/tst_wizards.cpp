// The calibration wizards end to end against the simulated board with built-in sensor
// errors: each run records through the real session, computes, and writes; the board
// ends up with the right fields and flag bits, cancelling restores the snapshot, and a
// saved recording gives the same result offline.
#include "device/boardmodel.h"
#include "device/calibrationrun.h"
#include "device/fakeboard.h"
#include "device/ports.h"

#include <QSignalSpy>
#include <QTemporaryDir>
#include <QTest>

#include <Eigen/LU>

#include <cmath>

using namespace ins10;
using calib::Procedure;

namespace {

struct Bench {
    std::unique_ptr<FakeBoard> board;
    DeviceController device;
    BoardModel model;

    static SessionConfig sessionConfig() {
        SessionConfig c;
        c.settleMs = 5;
        c.batchIntervalMs = 5;
        c.commandTimeoutMs = 3000;
        return c;
    }

    explicit Bench(const FakeBoardConfig &config)
        : board(std::make_unique<FakeBoard>(config)),
          device(std::make_unique<FakeTransportFactory>(board.get()), DeviceController::Threading::SameThread, sessionConfig()),
          model(&device) {}

    bool connect() {
        device.connectTo(kDemoPortName);
        return QTest::qWaitFor([this] { return device.isConnected() && idle(); }, 5000);
    }
    bool idle() const { return device.session()->queuedCommands() == 0; }
    bool settle() { return QTest::qWaitFor([this] { return idle(); }, 5000); }

    // Board time passes in small steps so the session sees a steady stream.
    void run(double seconds, double step = 0.5) {
        for (double t = 0.0; t < seconds; t += step) {
            board->advance(step);
            QCoreApplication::processEvents();
        }
        // The session hands stream records over on a timer: give it real time.
        QTest::qWait(30);
    }
};

FakeBoardConfig boardConfig(FakeBoardConfig::MotionKind motion) {
    FakeBoardConfig c;
    c.realTime = false;
    c.bootAttRateHz = 10;
    c.motion = motion;
    c.sensors = sim::SensorModel::typical();
    return c;
}

Eigen::Vector3d field3(const CalibrationSet &set, const char *name) {
    const QList<double> v = set.value(QLatin1String(name));
    return v.size() == 3 ? Eigen::Vector3d(v[0], v[1], v[2]) : Eigen::Vector3d::Constant(NAN);
}

Eigen::Matrix3d field9(const CalibrationSet &set, const char *name) {
    const QList<double> v = set.value(QLatin1String(name));
    Eigen::Matrix3d m = Eigen::Matrix3d::Constant(NAN);
    if (v.size() == 9) {
        for (int i = 0; i < 9; ++i) m(i / 3, i % 3) = v[i];
    }
    return m;
}

QList<double> list(const Eigen::Vector3d &v) { return {v[0], v[1], v[2]}; }
QList<double> list(const Eigen::Matrix3d &m) {
    QList<double> out;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) out.append(m(r, c));
    }
    return out;
}

bool waitState(CalibrationRun &run, CalibrationRun::State state, int ms = 60000) {
    return QTest::qWaitFor([&] { return run.state() == state; }, ms);
}

bool writeAndWait(BoardModel &model, const CalibrationSet &fields, quint32 flags) {
    bool done = false, ok = false;
    model.writeCalibration(fields, flags, false, [&](bool success, const QString &) {
        done = true;
        ok = success;
    });
    return QTest::qWaitFor([&] { return done; }, 5000) && ok;
}

} // namespace

class TestWizards : public QObject {
    Q_OBJECT
private slots:
    void accelerometerAndGyro() {
        FakeBoardConfig config = boardConfig(FakeBoardConfig::MotionKind::MultiPosition);
        config.positions = 16;
        config.tempC = 31.0;
        Bench bench(config);
        QVERIFY(bench.connect());
        const sim::SensorModel &s = bench.board->sensors();

        CalibrationRun run(&bench.model, Procedure::Imu);
        run.start();
        QVERIFY(waitState(run, CalibrationRun::State::Recording, 5000));
        QCOMPARE(bench.board->rawDiv(), 2);
        bench.run(5.0 * config.positions + 4.0);
        run.stop();
        QVERIFY(bench.settle());
        QCOMPARE(bench.board->rawDiv(), 0); // streams handed back
        QCOMPARE(run.imuProgress().positions, config.positions + 1);

        run.compute();
        QVERIFY(waitState(run, CalibrationRun::State::Computed));
        const calib::WorkflowResult &r = run.result();
        QVERIFY2(r.ok, qPrintable(r.error));
        QVERIFY(r.writable && r.imu);
        const calib::ImuCalibration &c = *r.imu;
        // The truth at this temperature: the drift terms are part of the bias.
        const Eigen::Vector3d accelBias = s.accelBias + s.accelDrift(31.0);
        const Eigen::Vector3d gyroBias = s.gyroBias + s.gyroDrift(31.0);
        qInfo("imu: %d positions, %d orientations, accel bias error %.3f mg, matrix %.5f; gyro bias %.4f dps, matrix %.5f (%s)",
              c.positions, c.orientations, (c.accelBias - accelBias).cwiseAbs().maxCoeff() * 1000,
              (c.accelMatrix - s.accelMatrix).cwiseAbs().maxCoeff(), (c.gyroBias - gyroBias).cwiseAbs().maxCoeff(),
              (c.gyroMatrix - s.gyroMatrix).cwiseAbs().maxCoeff(), qPrintable(c.gyroMatrixFitted));
        QVERIFY(c.orientations >= 9);
        QVERIFY((c.accelBias - accelBias).cwiseAbs().maxCoeff() < 0.0005);
        QVERIFY((c.accelMatrix - s.accelMatrix).cwiseAbs().maxCoeff() < 0.0005);
        QVERIFY((c.gyroBias - gyroBias).cwiseAbs().maxCoeff() < 0.005);
        QCOMPARE(c.gyroMatrixFitted, QStringLiteral("full"));
        QVERIFY((c.gyroMatrix - s.gyroMatrix).cwiseAbs().maxCoeff() < 0.0015);

        QSignalSpy written(&run, &CalibrationRun::written);
        run.saveToFlash();
        QTRY_VERIFY_WITH_TIMEOUT(!written.isEmpty(), 10000);
        QVERIFY2(written.first().at(0).toBool(), qPrintable(written.first().at(1).toString()));
        QVERIFY(run.saved());
        QVERIFY(bench.settle());
        const auto stored = bench.board->storedCalibration();
        QVERIFY(stored.has_value());
        QCOMPARE(calFlags(*stored), quint32(CalAccel | CalGyro));
        QVERIFY((field3(*stored, "accel_bias") - c.accelBias).cwiseAbs().maxCoeff() < 1e-6);
        QVERIFY((field9(*stored, "gyro_matrix") - c.gyroMatrix).cwiseAbs().maxCoeff() < 1e-6);
        QVERIFY(std::abs(stored->value(QStringLiteral("accel_tc_tref")).front() - 31.0) < 0.1);
        // The save kept the board's own power-up rate, and ours is back.
        QCOMPARE(bench.board->storedAttRate(), 10);
        QCOMPARE(bench.board->attRate(), 50);
    }

    void magnetometerTryAndCancel() {
        FakeBoardConfig config = boardConfig(FakeBoardConfig::MotionKind::Tumble);
        config.tumbleRateDps = 55.0;
        config.motionSeed = 13;
        Bench bench(config);
        QVERIFY(bench.connect());
        const sim::SensorModel &s = bench.board->sensors();
        // An accelerometer calibration is already on the board (as recommended).
        CalibrationSet accel;
        accel.insert(QStringLiteral("accel_bias"), list(Eigen::Vector3d(s.accelBias + s.accelDrift(30.0))));
        accel.insert(QStringLiteral("accel_matrix"), list(s.accelMatrix));
        QVERIFY(writeAndWait(bench.model, accel, CalAccel));
        const CalibrationSet before = bench.board->ramCalibration();
        QCOMPARE(calFlags(before), quint32(CalAccel));

        CalibrationRun run(&bench.model, Procedure::Mag);
        run.start();
        QVERIFY(waitState(run, CalibrationRun::State::Recording, 5000));
        QVERIFY(bench.board->magStream());
        QCOMPARE(bench.board->rawDiv(), 8);
        bench.run(150.0);
        run.stop();
        QVERIFY(bench.settle());
        QVERIFY(!bench.board->magStream());
        QVERIFY(run.magProgress().coverage > 0.8);
        run.compute();
        QVERIFY(waitState(run, CalibrationRun::State::Computed));
        const calib::WorkflowResult &r = run.result();
        QVERIFY2(r.ok, qPrintable(r.error));
        QVERIFY2(r.writable, qPrintable(r.refusal));
        const calib::MagCalibration &c = *r.mag;
        // corrected = M (raw - b) and raw - hard = soft mount f, so M soft mount = I.
        const Eigen::Matrix3d chain = c.matrix * s.magSoft * s.magMount;
        qInfo("mag: bias error %.3f uT, |M soft mount - I| %.4f, dip %.2f deg, field %.2f uT, coverage %.0f %%",
              (c.bias - s.magHard).cwiseAbs().maxCoeff(), (chain - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff(),
              *c.dipDeg, c.fieldUt, c.coverage * 100);
        QVERIFY(!calib::hasWarning(r.warnings, calib::Warning::Code::AccelUncalibrated));
        QVERIFY((c.bias - s.magHard).cwiseAbs().maxCoeff() < 0.3);
        // Unit average gain: the fitted field is det(soft)^(1/3) times the true one, so
        // the chain is that scale times the identity.
        const double scale = std::cbrt(s.magSoft.determinant());
        QVERIFY((chain / scale - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff() < 0.003);
        QVERIFY(std::abs(*c.dipDeg - sim::kDipDeg) < 0.5);

        // Try it: in RAM, flags OR-ed.
        QSignalSpy written(&run, &CalibrationRun::written);
        run.tryOnBoard();
        QTRY_VERIFY_WITH_TIMEOUT(written.size() == 1, 10000);
        QVERIFY(written.last().at(0).toBool());
        QVERIFY(run.tried());
        QCOMPARE(calFlags(bench.board->ramCalibration()), quint32(CalAccel | CalMag));
        QVERIFY((field3(bench.board->ramCalibration(), "mag_bias") - c.bias).cwiseAbs().maxCoeff() < 1e-4);
        QVERIFY(!bench.board->hasStored()); // nothing went to flash

        // Cancel: the snapshot comes back.
        run.revert();
        QTRY_VERIFY_WITH_TIMEOUT(written.size() == 2, 10000);
        QVERIFY(written.last().at(0).toBool());
        QVERIFY(bench.settle());
        QVERIFY(changedCalFields(before, bench.board->ramCalibration()).isEmpty());
        QVERIFY(!bench.board->hasStored());

        // The recording saved to a file gives the same answer offline.
        QTemporaryDir dir;
        const QString path = dir.filePath(QStringLiteral("mag.log"));
        QVERIFY(run.saveRecording(path));
        CalibrationRun offline(&bench.model, Procedure::Mag);
        QVERIFY(offline.loadRecording(path));
        QVERIFY(offline.isOffline());
        QCOMPARE(calFlags(offline.snapshot()), quint32(CalAccel)); // the log carries the calibration
        offline.compute();
        QVERIFY(waitState(offline, CalibrationRun::State::Computed));
        QVERIFY(offline.result().ok && offline.result().mag);
        QVERIFY((offline.result().mag->bias - c.bias).cwiseAbs().maxCoeff() < 1e-9);
        QVERIFY((offline.result().mag->matrix - c.matrix).cwiseAbs().maxCoeff() < 1e-9);
    }

    void temperature() {
        FakeBoardConfig config = boardConfig(FakeBoardConfig::MotionKind::Still);
        config.stillT0C = 20.0;
        config.stillT1C = 45.0;
        config.stillTauS = 900.0;
        Bench bench(config);
        QVERIFY(bench.connect());
        const sim::SensorModel &s = bench.board->sensors();
        CalibrationRun run(&bench.model, Procedure::Thermal);
        run.start();
        QVERIFY(waitState(run, CalibrationRun::State::Recording, 5000));
        QCOMPARE(bench.board->rawDiv(), 256);
        bench.run(1200.0, 5.0);
        run.stop();
        QVERIFY(bench.settle());
        QVERIFY(run.thermalProgress().span() > 15.0);
        run.compute();
        QVERIFY(waitState(run, CalibrationRun::State::Computed));
        const calib::WorkflowResult &r = run.result();
        QVERIFY2(r.ok, qPrintable(r.error));
        const calib::ThermalCalibration &c = *r.thermal;
        double worst = 0.0;
        for (double t = c.tMin; t <= c.tMax; t += 0.5) {
            const Eigen::Vector3d truth = s.gyroBias + s.gyroDrift(t);
            const double u = t - c.tRef;
            const Eigen::Vector3d model = c.gyroBias + c.gyroTc.col(0) * u + c.gyroTc.col(1) * u * u + c.gyroTc.col(2) * u * u * u;
            worst = std::max(worst, (model - truth).cwiseAbs().maxCoeff());
        }
        qInfo("thermal: %.1f..%.1f C, order %d, model error %.4f dps", c.tMin, c.tMax, c.order, worst);
        QVERIFY(worst < 0.005);
        QVERIFY(std::abs(c.accelTc(2, 0) - s.accelTc(2, 0)) < 2e-5);

        QSignalSpy written(&run, &CalibrationRun::written);
        run.saveToFlash();
        QTRY_VERIFY_WITH_TIMEOUT(!written.isEmpty(), 10000);
        QVERIFY(written.first().at(0).toBool());
        QVERIFY(bench.settle());
        const auto stored = bench.board->storedCalibration();
        QVERIFY(stored.has_value());
        QCOMPARE(calFlags(*stored), quint32(CalGyroTc | CalAccelTc));
        QVERIFY(std::abs(stored->value(QStringLiteral("gyro_tc_tref")).front() - c.tRef) < 1e-6);
        QVERIFY(std::abs(stored->value(QStringLiteral("gyro_tc_x")).front() - c.gyroTc(0, 0)) < 1e-6);
    }

    void cableOutDuringARecording() {
        FakeBoardConfig config = boardConfig(FakeBoardConfig::MotionKind::Tumble);
        Bench bench(config);
        QVERIFY(bench.connect());
        auto run = std::make_unique<CalibrationRun>(&bench.model, Procedure::Mag);
        QSignalSpy problems(run.get(), &CalibrationRun::problem);
        run->start();
        QVERIFY(waitState(*run, CalibrationRun::State::Recording, 5000));
        bench.run(3.0);
        const qsizetype rows = run->recording().imuRows();
        QVERIFY(rows > 100);
        bench.board->unplug();
        QTRY_COMPARE(run->state(), CalibrationRun::State::Recorded);
        QCOMPARE(problems.size(), 1);
        QCOMPARE(run->recording().imuRows(), rows);
        run.reset(); // no crash, nothing sent to a board that is not there
        QCOMPARE(bench.device.state(), ConnectionState::Lost);
    }

    void refusesWithoutABoard() {
        FakeBoardConfig config = boardConfig(FakeBoardConfig::MotionKind::Still);
        Bench bench(config);
        CalibrationRun run(&bench.model, Procedure::Imu);
        QSignalSpy problems(&run, &CalibrationRun::problem);
        run.start();
        QCOMPARE(problems.size(), 1);
        QCOMPARE(run.state(), CalibrationRun::State::Idle);
        QString error;
        QVERIFY(!run.loadRecording(QStringLiteral("no/such.log"), &error));
    }
};

QTEST_GUILESS_MAIN(TestWizards)
#include "tst_wizards.moc"
