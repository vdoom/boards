// The C++ calibration against the Python tool on the same data (fixtures written by
// tools/gen_fixtures.py), with the tolerances of PLAN.md section 8, and against the
// true sensor errors with the thresholds of tools/tests/test_calib.py.
#include "core/calib/stats.h"
#include "core/calib/workflow.h"
#include "core/logfile.h"
#include "fixture_dir.h"

#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTest>

#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <Eigen/LU>

#include <algorithm>
#include <cmath>
#include <random>
#include <utility>

using namespace ins10;
using namespace ins10::calib;

namespace {

QString fixture(const QString &name) { return QString::fromUtf8(INS10_FIXTURE_DIR) + u'/' + name; }

QJsonObject json(const QString &name) {
    QFile f(fixture(name + QStringLiteral(".json")));
    if (!f.open(QIODevice::ReadOnly)) qFatal("missing fixture %s", qPrintable(name));
    return QJsonDocument::fromJson(f.readAll()).object();
}

Recording log(const QString &name) {
    QString error;
    const auto rec = loadLog(fixture(name + QStringLiteral(".log")), &error);
    if (!rec) qFatal("cannot read %s: %s", qPrintable(name), qPrintable(error));
    return *rec;
}

Eigen::VectorXd vec(const QJsonValue &v) {
    const QJsonArray a = v.toArray();
    Eigen::VectorXd out(a.size());
    for (qsizetype i = 0; i < a.size(); ++i) out[i] = a[i].toDouble();
    return out;
}

Eigen::Matrix3d mat3(const QJsonValue &v) {
    Eigen::Matrix3d m;
    const QJsonArray rows = v.toArray();
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) m(r, c) = rows[r].toArray()[c].toDouble();
    }
    return m;
}

Eigen::MatrixX3d rows3(const QJsonValue &v) {
    const QJsonArray rows = v.toArray();
    Eigen::MatrixX3d m(rows.size(), 3);
    for (qsizetype i = 0; i < rows.size(); ++i) {
        for (int c = 0; c < 3; ++c) m(i, c) = rows[i].toArray()[c].toDouble();
    }
    return m;
}

Eigen::MatrixXd rows(const QJsonValue &v) {
    const QJsonArray r = v.toArray();
    const qsizetype cols = r.isEmpty() ? 0 : r[0].toArray().size();
    Eigen::MatrixXd m(r.size(), cols);
    for (qsizetype i = 0; i < r.size(); ++i) {
        for (qsizetype c = 0; c < cols; ++c) m(i, c) = r[i].toArray()[c].toDouble();
    }
    return m;
}

double maxDiff(const Eigen::MatrixXd &a, const Eigen::MatrixXd &b) { return (a - b).cwiseAbs().maxCoeff(); }

// Relative 1e-6, but never stricter than 1e-9 absolute.
bool coefficientsAgree(const Eigen::MatrixXd &cpp, const Eigen::MatrixXd &python, QString *worst) {
    bool ok = true;
    for (Eigen::Index i = 0; i < cpp.size(); ++i) {
        const double ref = python.data()[i], got = cpp.data()[i];
        const double allowed = std::max(1e-6 * std::abs(ref), 1e-9);
        if (std::abs(got - ref) > allowed) {
            ok = false;
            *worst += QStringLiteral(" [%1: %2 vs %3]").arg(i).arg(got, 0, 'g', 12).arg(ref, 0, 'g', 12);
        }
    }
    return ok;
}

Eigen::MatrixXd fieldMatrix(const CalibrationSet &set, const QString &name) {
    const QList<double> v = set.value(name);
    Eigen::MatrixXd m(1, v.size());
    for (qsizetype i = 0; i < v.size(); ++i) m(0, i) = v[i];
    return m;
}

Eigen::MatrixXd cliField(const QJsonObject &cli, const QString &name) {
    const QJsonArray a = cli.value(QStringLiteral("fields")).toObject().value(name).toArray();
    Eigen::MatrixXd m(1, a.size());
    for (qsizetype i = 0; i < a.size(); ++i) m(0, i) = a[i].toDouble();
    return m;
}

// The board at rest in orientations spread over the whole sphere: what the
// accelerometer reads, and a magnetometer mounted through `mount`, with soft and hard
// iron. Field 50 uT, east-north-up, dip positive when it points down.
struct MagScene {
    Eigen::MatrixX3d mag, accel, gyro, fieldImu;
};

MagScene magScene(const Eigen::Matrix3d &mount, double dipDeg, unsigned seed) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> uniform(0.0, 1.0);
    std::normal_distribution<double> noise(0.0, 1.0);
    // An explicit return type: 'auto' would return an Eigen expression that refers to
    // the temporary vector after it is gone.
    const auto jitter = [&](double sigma) -> Eigen::Vector3d {
        const double x = noise(rng), y = noise(rng), z = noise(rng);
        return Eigen::Vector3d(x, y, z) * sigma;
    };
    const double dip = dipDeg * M_PI / 180.0;
    const Eigen::Vector3d fieldNav(0.0, 50.0 * std::cos(dip), -50.0 * std::sin(dip));
    Eigen::Matrix3d soft;
    soft << 1.08, 0.03, -0.02, 0.03, 0.94, 0.04, -0.02, 0.04, 1.01;
    const Eigen::Vector3d hard(23.0, -41.0, 12.0);
    const Eigen::Index n = 3000;
    MagScene s{Eigen::MatrixX3d(n, 3), Eigen::MatrixX3d(n, 3), Eigen::MatrixX3d::Zero(n, 3), Eigen::MatrixX3d(n, 3)};
    for (Eigen::Index i = 0; i < n; ++i) {
        // A uniformly random rotation (Shoemake).
        const double u1 = uniform(rng), u2 = 2.0 * M_PI * uniform(rng), u3 = 2.0 * M_PI * uniform(rng);
        const Eigen::Quaterniond q(std::sqrt(u1) * std::cos(u3), std::sqrt(1.0 - u1) * std::sin(u2),
                                   std::sqrt(1.0 - u1) * std::cos(u2), std::sqrt(u1) * std::sin(u3));
        const Eigen::Matrix3d navToBody = q.toRotationMatrix().transpose();
        const Eigen::Vector3d field = navToBody * fieldNav;
        s.fieldImu.row(i) = field.transpose();
        s.mag.row(i) = (soft * mount * field + hard + jitter(0.05)).transpose();
        s.accel.row(i) = (navToBody * Eigen::Vector3d::UnitZ() + jitter(0.002)).transpose();
    }
    return s;
}

// Smallest and largest angle between the calibrated field and the true one, degrees.
std::pair<double, double> directionErrorsDeg(const MagCalibration &c, const MagScene &s) {
    const Eigen::MatrixX3d corrected = (s.mag.rowwise() - c.bias.transpose()) * c.matrix.transpose();
    double best = 180.0, worst = 0.0;
    for (Eigen::Index i = 0; i < corrected.rows(); ++i) {
        const double cosang = corrected.row(i).normalized().dot(s.fieldImu.row(i).normalized());
        const double angle = std::acos(std::clamp(cosang, -1.0, 1.0)) * 180.0 / M_PI;
        best = std::min(best, angle);
        worst = std::max(worst, angle);
    }
    return {best, worst};
}

// The mounting of tools/gen_fixtures.py: turned 178 deg about Z and a little off square.
Eigen::Matrix3d mountRotation() {
    return (Eigen::AngleAxisd(178.0 * M_PI / 180.0, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(2.0 * M_PI / 180.0, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(-1.5 * M_PI / 180.0, Eigen::Vector3d::UnitX()))
        .toRotationMatrix();
}

} // namespace

class TestCalib : public QObject {
    Q_OBJECT
private slots:
    void solverMatchesClosedForm() {
        // A linear problem: the solution is the normal-equations one.
        Eigen::MatrixXd a(6, 2);
        a << 1, 0, 1, 1, 1, 2, 1, 3, 1, 4, 1, 5;
        Eigen::VectorXd y(6);
        y << 0.1, 1.9, 4.2, 5.8, 8.1, 9.9;
        const auto fit = leastSquares([&](const Eigen::VectorXd &p) { return Eigen::VectorXd(a * p - y); }, Eigen::Vector2d(0, 0));
        const Eigen::VectorXd exact = (a.transpose() * a).ldlt().solve(a.transpose() * y);
        QVERIFY(fit.converged);
        QVERIFY(maxDiff(fit.x, exact) < 1e-9);
        // soft_l1 ignores an outlier that pulls the plain fit.
        y[3] = 50.0;
        LsqOptions robust;
        robust.loss = Loss::SoftL1;
        robust.fScale = 0.2;
        const auto plain = leastSquares([&](const Eigen::VectorXd &p) { return Eigen::VectorXd(a * p - y); }, Eigen::Vector2d(0, 0));
        const auto soft = leastSquares([&](const Eigen::VectorXd &p) { return Eigen::VectorXd(a * p - y); }, Eigen::Vector2d(0, 0), robust);
        QVERIFY(std::abs(soft.x[1] - 2.0) < 0.1);
        QVERIFY(std::abs(plain.x[1] - 2.0) > 0.5);
        // Cancellation.
        std::atomic<bool> cancel{true};
        robust.cancel = &cancel;
        QVERIFY_THROWS_EXCEPTION(CalibrationError, leastSquares([&](const Eigen::VectorXd &p) { return Eigen::VectorXd(a * p - y); },
                                                                Eigen::Vector2d(0, 0), robust));
    }

    void numpyConventions() {
        QCOMPARE(median(std::vector<double>{3, 1, 2}), 2.0);
        QCOMPARE(median(std::vector<double>{4, 1, 3, 2}), 2.5);
        QCOMPARE(roundHalfEven(30.5), 30.0);
        QCOMPARE(roundHalfEven(31.5), 32.0);
        QCOMPARE(roundHalfEven(-0.5), -0.0);
        QCOMPARE(roundHalfEven(29.51), 30.0);
        QCOMPARE(sphereCell(Eigen::Vector3d(0, 0, 1)), 5 * 12 + 6);
        QCOMPARE(sphereCell(Eigen::Vector3d(-1, -1e-12, 0)), 3 * 12 + 0);
        QCOMPARE(sphereCell(Eigen::Vector3d(0, 0, 0)), -1);
    }

    void magnetometer() {
        const QJsonObject j = json(QStringLiteral("mag"));
        const QJsonObject py = j[QStringLiteral("python")].toObject();
        const QJsonObject truth = j[QStringLiteral("truth")].toObject();
        const Recording rec = log(QStringLiteral("mag"));
        const MagWithImu m = rec.magWithImu();
        const MagCalibration c = calibrateMag(m.mag, &m.accel, &m.gyro);

        // Against Python.
        const double biasDiff = maxDiff(c.bias, vec(py[QStringLiteral("bias")]));
        const double matrixDiff = maxDiff(c.matrix, mat3(py[QStringLiteral("matrix")]));
        qInfo("mag vs Python: bias %.2e uT, matrix %.2e, field %.2e uT, dip %.2e deg", biasDiff, matrixDiff,
              std::abs(c.fieldUt - py[QStringLiteral("field_ut")].toDouble()),
              std::abs(*c.dipDeg - py[QStringLiteral("dip_deg")].toDouble()));
        QVERIFY(biasDiff < 0.02);
        QVERIFY(matrixDiff < 1e-3);
        QCOMPARE(c.coverage, py[QStringLiteral("coverage")].toDouble());
        QCOMPARE(c.samples, py[QStringLiteral("samples")].toInt());
        QCOMPARE(c.aligned, py[QStringLiteral("aligned")].toBool());
        QCOMPARE(c.warnings.size(), py[QStringLiteral("warnings")].toArray().size());

        // Against the truth, as test_magnetometer.
        QVERIFY(c.aligned);
        QVERIFY(c.warnings.isEmpty());
        const Eigen::MatrixX3d fieldImu = rows3(truth[QStringLiteral("field_imu_at_mag")]);
        const Eigen::MatrixX3d raw = [&] {
            Eigen::MatrixX3d r(rec.magRows(), 3);
            for (Eigen::Index i = 0; i < r.rows(); ++i) r.row(i) = rec.mag()[static_cast<size_t>(i)].transpose();
            return r;
        }();
        QCOMPARE(raw.rows(), fieldImu.rows());
        const Eigen::MatrixX3d corrected = (raw.rowwise() - c.bias.transpose()) * c.matrix.transpose();
        const Eigen::VectorXd norms = corrected.rowwise().norm();
        double worstAngle = 0.0;
        for (Eigen::Index i = 0; i < raw.rows(); ++i) {
            const double cosang = std::clamp(corrected.row(i).dot(fieldImu.row(i)) / (norms[i] * 50.0), -1.0, 1.0);
            worstAngle = std::max(worstAngle, std::acos(cosang) * 180.0 / M_PI);
        }
        const double spread = std::sqrt((norms.array() - norms.mean()).square().mean()) / norms.mean();
        const Eigen::Matrix3d soft = mat3(truth[QStringLiteral("soft")]);
        qInfo("mag vs truth: bias %.3f uT, direction max %.3f deg, spread %.4f %%, dip %.2f deg",
              maxDiff(c.bias, vec(truth[QStringLiteral("hard")])), worstAngle, spread * 100, *c.dipDeg);
        QVERIFY(maxDiff(c.bias, vec(truth[QStringLiteral("hard")])) < 0.3);
        QVERIFY(worstAngle < 0.6);
        QVERIFY(spread < 0.004);
        QVERIFY(std::abs(*c.dipDeg - 66.0) < 0.5);
        QVERIFY(std::abs(c.fieldUt / (50.0 * std::cbrt(soft.determinant())) - 1.0) < 0.01);

        // The tool's magcal, offline: the same fields.
        const WorkflowResult w = runMagCal(rec, {}, MagOptions{false, false});
        QVERIFY(w.ok && w.writable);
        QCOMPARE(w.setFlags, quint32(CalMag));
        const QJsonObject cli = j[QStringLiteral("cli")].toObject();
        QCOMPARE(cli[QStringLiteral("exit_code")].toInt(), 0);
        QVERIFY(maxDiff(fieldMatrix(w.fields, QStringLiteral("mag_bias")), cliField(cli, QStringLiteral("mag_bias"))) < 0.02);
        QVERIFY(maxDiff(fieldMatrix(w.fields, QStringLiteral("mag_matrix")), cliField(cli, QStringLiteral("mag_matrix"))) < 1e-3);
        QVERIFY(maxDiff(fieldMatrix(w.fields, QStringLiteral("mag_field")), cliField(cli, QStringLiteral("mag_field"))) < 0.01);
        QVERIFY(maxDiff(fieldMatrix(w.fields, QStringLiteral("mag_dip")), cliField(cli, QStringLiteral("mag_dip"))) < 0.05);
        QCOMPARE(w.fields.keys(), QStringList({"mag_bias", "mag_dip", "mag_field", "mag_matrix"}));
    }

    void magnetometerRejectsARing() {
        const QJsonObject j = json(QStringLiteral("mag_ring"));
        const Recording rec = log(QStringLiteral("mag_ring"));
        Eigen::MatrixX3d mag(rec.magRows(), 3);
        for (Eigen::Index i = 0; i < mag.rows(); ++i) mag.row(i) = rec.mag()[static_cast<size_t>(i)].transpose();
        if (j.contains(QStringLiteral("error"))) {
            QVERIFY_THROWS_EXCEPTION(CalibrationError, calibrateMag(mag));
            qInfo("Python and C++ both refuse: %s", qPrintable(j[QStringLiteral("error")].toString()));
        } else {
            const MagCalibration c = calibrateMag(mag);
            const QJsonObject py = j[QStringLiteral("result")].toObject();
            QCOMPARE(c.coverage, py[QStringLiteral("coverage")].toDouble());
            QVERIFY(c.coverage < 0.6 && !c.warnings.isEmpty());
        }
    }

    // A magnetometer whose axes are a mirror image of the IMU's, as the INS10-A3
    // firmware reports them (NOTES.md item 17). calib.py aligns it with a rotation and
    // turns the field upside down; here it is aligned with a reflection.
    void magnetometerWithMirroredAxes() {
        const Eigen::Matrix3d mirror = Eigen::Vector3d(1.0, 1.0, -1.0).asDiagonal();
        const MagScene s = magScene(mirror * mountRotation(), 66.0, 7);
        const MagCalibration c = calibrateMag(s.mag, &s.accel, &s.gyro);
        const auto [best, worst] = directionErrorsDeg(c, s);
        qInfo("mirrored, north: dip %.2f deg, direction error %.3f..%.3f deg", *c.dipDeg, best, worst);
        QVERIFY(c.aligned && c.mirrored);
        QVERIFY(c.matrix.determinant() < 0.0);
        QVERIFY(hasWarning(c.warnings, Warning::Code::Mirrored));
        QVERIFY(std::abs(*c.dipDeg - 66.0) < 0.5);
        QVERIFY(worst < 0.6);

        // Told it is in the south, the same data gets a rotation: the field then points
        // up and every direction comes out reversed. The setting decides.
        const MagCalibration south = calibrateMag(s.mag, &s.accel, &s.gyro, nullptr, Hemisphere::South);
        QVERIFY(south.aligned && !south.mirrored);
        QVERIFY(std::abs(*south.dipDeg + 66.0) < 0.5);
        QVERIFY(directionErrorsDeg(south, s).first > 179.0);
    }

    // Right-handed axes come out as calib.py has them, in both hemispheres.
    void magnetometerHemispheres() {
        for (const double dip : {66.0, -66.0}) {
            const MagScene s = magScene(mountRotation(), dip, 11);
            const MagCalibration c =
                calibrateMag(s.mag, &s.accel, &s.gyro, nullptr, dip > 0 ? Hemisphere::North : Hemisphere::South);
            QVERIFY(c.aligned && !c.mirrored);
            QVERIFY(c.matrix.determinant() > 0.0);
            QVERIFY(!hasWarning(c.warnings, Warning::Code::Mirrored));
            QVERIFY(std::abs(*c.dipDeg - dip) < 0.5);
            QVERIFY(directionErrorsDeg(c, s).second < 0.6);
        }
    }

    // The board itself, in the printed jig on 26 faces next to two PCs (2026-11-18):
    // every 5th magnetometer sample of 492 s, with the IMU rows on either side. The
    // magnetometer's axes come out mirrored (about diag(1, 1, -1)) and the field points
    // down, as it does where this was recorded (about 74-75 deg of dip there).
    void realBoardMirroredMagnetometer() {
        const Recording rec = log(QStringLiteral("real_board_jig"));
        QVERIFY(rec.magRows() > 4000);
        const WorkflowResult w = runMagCal(rec, rec.calibration(), MagOptions{false, false});
        QVERIFY(w.ok && w.mag);
        const MagCalibration &c = *w.mag;
        qInfo("real board: aligned %d, mirrored %d, dip %.1f deg, field %.1f uT, spread %.2f %%, coverage %.0f %%; %s",
              c.aligned, c.mirrored, c.dipDeg.value_or(NAN), c.fieldUt, c.normResidualPct, c.coverage * 100.0,
              qPrintable(warningTexts(c.warnings).join(QStringLiteral("; "))));
        QVERIFY(c.aligned && c.mirrored && w.writable);
        QVERIFY(*c.dipDeg > 55.0 && *c.dipDeg < 80.0);
        const Eigen::Matrix3d m = c.matrix;
        QVERIFY2(m(0, 0) > 0.8 && m(1, 1) > 0.8 && m(2, 2) < -0.8, qPrintable(QStringLiteral("%1 %2 %3").arg(m(0, 0)).arg(m(1, 1)).arg(m(2, 2))));
        QVERIFY(w.fields.value(QStringLiteral("mag_dip")).value(0) > 0.0);
    }

    void imu() {
        const QJsonObject j = json(QStringLiteral("imu"));
        const QJsonObject py = j[QStringLiteral("python")].toObject();
        const QJsonObject truth = j[QStringLiteral("truth")].toObject();
        const Recording rec = log(QStringLiteral("imu"));
        const ImuArrays a = rec.imuArrays();
        const std::vector<bool> contiguous = rec.contiguity();
        const ImuCalibration c = calibrateImu(a.t, a.dt, a.gyro, a.accel, a.temp, &contiguous);

        // The rest positions themselves are the same rows.
        const QJsonArray intervals = j[QStringLiteral("intervals")].toArray();
        QCOMPARE(static_cast<qsizetype>(c.intervals.size()), intervals.size());
        for (qsizetype i = 0; i < intervals.size(); ++i) {
            QCOMPARE(c.intervals[static_cast<size_t>(i)].start, static_cast<Eigen::Index>(intervals[i].toArray()[0].toInteger()));
            QCOMPARE(c.intervals[static_cast<size_t>(i)].stop, static_cast<Eigen::Index>(intervals[i].toArray()[1].toInteger()));
        }
        QCOMPARE(c.positions, py[QStringLiteral("positions")].toInt());
        QCOMPARE(c.orientations, py[QStringLiteral("orientations")].toInt());
        QCOMPARE(c.gyroMatrixFitted, py[QStringLiteral("gyro_matrix_fitted")].toString());
        const double ab = maxDiff(c.accelBias, vec(py[QStringLiteral("accel_bias")]));
        const double am = maxDiff(c.accelMatrix, mat3(py[QStringLiteral("accel_matrix")]));
        const double gb = maxDiff(c.gyroBias, vec(py[QStringLiteral("gyro_bias")]));
        const double gm = maxDiff(c.gyroMatrix, mat3(py[QStringLiteral("gyro_matrix")]));
        qInfo("imu vs Python: accel bias %.2e mg, accel matrix %.2e, gyro bias %.2e dps, gyro matrix %.2e", ab * 1000, am, gb, gm);
        QVERIFY(ab < 0.05e-3);
        QVERIFY(am < 1e-4);
        QVERIFY(gb < 1e-4);
        QVERIFY(gm < 2e-4);

        // Against the truth, as test_imu.
        QCOMPARE(c.positions, 17);
        QVERIFY(c.orientations >= 9);
        QVERIFY(maxDiff(c.accelBias, vec(truth[QStringLiteral("accel_bias")])) < 0.0005);
        QVERIFY(maxDiff(c.accelMatrix, mat3(truth[QStringLiteral("accel_matrix")])) < 0.0005);
        QVERIFY(maxDiff(c.gyroBias, vec(truth[QStringLiteral("gyro_bias")])) < 0.005);
        QCOMPARE(c.gyroMatrixFitted, QStringLiteral("full"));
        QVERIFY(maxDiff(c.gyroMatrix, mat3(truth[QStringLiteral("gyro_matrix")])) < 0.0015);
        QVERIFY(*c.gyroResidualDeg < 0.25 * *c.gyroResidualBeforeDeg);
        QVERIFY(std::abs(c.tempC - 31.0) < 0.05);

        // The tool's imucal, offline.
        const WorkflowResult w = runImuCal(rec, {});
        QVERIFY(w.ok && w.writable);
        QCOMPARE(w.setFlags, quint32(CalAccel | CalGyro));
        const QJsonObject cli = j[QStringLiteral("cli")].toObject();
        QStringList expected = cli[QStringLiteral("fields")].toObject().keys();
        QStringList got = w.fields.keys();
        expected.sort();
        got.sort();
        QCOMPARE(got, expected);
        QVERIFY(maxDiff(fieldMatrix(w.fields, QStringLiteral("accel_bias")), cliField(cli, QStringLiteral("accel_bias"))) < 0.05e-3);
        QVERIFY(maxDiff(fieldMatrix(w.fields, QStringLiteral("gyro_matrix")), cliField(cli, QStringLiteral("gyro_matrix"))) < 2e-4);
    }

    void accelSixFaces() {
        const QJsonObject j = json(QStringLiteral("accel_faces"));
        const Eigen::MatrixX3d means = rows3(j[QStringLiteral("means")]);
        const AccelCalibration c = calibrateAccel(means);
        const QJsonObject py = j[QStringLiteral("python")].toObject();
        QCOMPARE(c.orientations, 6);
        QCOMPARE(c.orientations, py[QStringLiteral("orientations")].toInt());
        QVERIFY(maxDiff(c.bias, vec(py[QStringLiteral("bias")])) < 0.05e-3);
        QVERIFY(maxDiff(c.matrix, mat3(py[QStringLiteral("matrix")])) < 1e-4);
        QVERIFY(maxDiff(c.bias, vec(j[QStringLiteral("truth")].toObject()[QStringLiteral("bias")])) < 1e-6);
        QVERIFY(maxDiff(c.matrix.diagonal(), vec(j[QStringLiteral("truth")].toObject()[QStringLiteral("scale")])) < 1e-6);
        QVERIFY(c.residualMg < 1e-3 && 1e-3 < c.residualBeforeMg);
        QVERIFY(j[QStringLiteral("four")].toObject().contains(QStringLiteral("error")));
        QVERIFY_THROWS_EXCEPTION(CalibrationError, calibrateAccel(means.topRows(4)));
    }

    void imuWithLostLines() {
        const QJsonObject j = json(QStringLiteral("imu_lost"));
        const QJsonObject py = j[QStringLiteral("python")].toObject();
        const Recording rec = log(QStringLiteral("imu_lost"));
        QCOMPARE(rec.lostSamples(), static_cast<qint64>(j[QStringLiteral("lost_samples")].toInteger()));
        const ImuArrays a = rec.imuArrays();
        const std::vector<bool> contiguous = rec.contiguity();
        const ImuCalibration c = calibrateImu(a.t, a.dt, a.gyro, a.accel, a.temp, &contiguous);
        QCOMPARE(c.gyroMatrixFitted, py[QStringLiteral("gyro_matrix_fitted")].toString());
        QVERIFY(c.gyroMatrixFitted != QLatin1String("none"));
        QVERIFY(maxDiff(c.gyroMatrix, mat3(py[QStringLiteral("gyro_matrix")])) < 2e-4);
        QVERIFY(maxDiff(c.gyroBias, vec(py[QStringLiteral("gyro_bias")])) < 1e-4);
        const double scale = j[QStringLiteral("truth")].toObject()[QStringLiteral("gyro_scale")].toDouble();
        QVERIFY((c.gyroMatrix.diagonal().array() - scale).abs().maxCoeff() < 0.002);
        // Without the contiguity information the hole would be integrated across.
        QVERIFY(!rec.isContiguous());
    }

    void thermal() {
        const QJsonObject j = json(QStringLiteral("thermal"));
        const QJsonObject py = j[QStringLiteral("python")].toObject();
        const Eigen::VectorXd temp = vec(j[QStringLiteral("temp")]);
        const Eigen::MatrixX3d gyro = rows3(j[QStringLiteral("gyro")]);
        const Eigen::MatrixX3d accel = rows3(j[QStringLiteral("accel")]);
        const ThermalCalibration c = calibrateThermal(temp, gyro, accel, j[QStringLiteral("accel_t_ref")].toDouble());

        QCOMPARE(c.order, py[QStringLiteral("order")].toInt());
        QCOMPARE(c.tRef, py[QStringLiteral("t_ref")].toDouble());
        QString worst;
        QVERIFY2(coefficientsAgree(c.gyroBias, vec(py[QStringLiteral("gyro_bias")]), &worst), qPrintable(worst));
        QVERIFY2(coefficientsAgree(c.gyroTc, rows(py[QStringLiteral("gyro_tc")]), &worst), qPrintable(worst));
        QVERIFY2(coefficientsAgree(c.accelTc, rows(py[QStringLiteral("accel_tc")]), &worst), qPrintable(worst));
        QVERIFY(std::abs(c.gyroDriftDps - py[QStringLiteral("gyro_drift_dps")].toDouble()) < 1e-9);
        QCOMPARE(c.warnings.size(), py[QStringLiteral("warnings")].toArray().size());
        QVERIFY(hasWarning(c.warnings, Warning::Code::ThermalMoved));

        // Against the truth, as test_thermal.
        const QJsonObject truth = j[QStringLiteral("truth")].toObject();
        const Eigen::MatrixXd g = rows(truth[QStringLiteral("g_true")]);
        double worstModel = 0.0;
        for (Eigen::Index i = 0; i < temp.size(); ++i) {
            const double d = temp[i] - 35.0, u = temp[i] - c.tRef;
            for (int a = 0; a < 3; ++a) {
                const double trueBias = g(a, 0) + g(a, 1) * d + g(a, 2) * d * d;
                const double model = c.gyroBias[a] + c.gyroTc(a, 0) * u + c.gyroTc(a, 1) * u * u + c.gyroTc(a, 2) * u * u * u;
                worstModel = std::max(worstModel, std::abs(model - trueBias));
            }
        }
        qInfo("thermal: order %d, model error %.4f dps", c.order, worstModel);
        QVERIFY(worstModel < 0.003);
        QVERIFY(maxDiff(c.accelTc.col(0), vec(truth[QStringLiteral("a_tc")])) < 2e-5);
        QVERIFY(c.gyroDriftDps > 0.3 && c.gyroResidualDps < 0.005);

        // No temperature change: refused.
        const QJsonObject flat = j[QStringLiteral("flat")].toObject();
        QVERIFY(flat.contains(QStringLiteral("error")));
        QVERIFY_THROWS_EXCEPTION(CalibrationError, calibrateThermal(vec(flat[QStringLiteral("temp")]),
                                                                    rows3(flat[QStringLiteral("gyro")]),
                                                                    rows3(flat[QStringLiteral("accel")])));
    }

    void thermalFromALog() {
        const QJsonObject j = json(QStringLiteral("thermal_tool"));
        const QJsonObject py = j[QStringLiteral("python")].toObject();
        const Recording rec = log(QStringLiteral("thermal_tool"));
        const WorkflowResult w = runThermalCal(rec, {});
        QVERIFY(w.ok && w.writable && w.thermal);
        const ThermalCalibration &c = *w.thermal;
        QCOMPARE(c.order, py[QStringLiteral("order")].toInt());
        QCOMPARE(c.tRef, py[QStringLiteral("t_ref")].toDouble());
        QString worst;
        QVERIFY2(coefficientsAgree(c.gyroTc, rows(py[QStringLiteral("gyro_tc")]), &worst), qPrintable(worst));
        QVERIFY2(coefficientsAgree(c.accelTc, rows(py[QStringLiteral("accel_tc")]), &worst), qPrintable(worst));
        QVERIFY2(coefficientsAgree(c.gyroBias, vec(py[QStringLiteral("gyro_bias")]), &worst), qPrintable(worst));
        const Eigen::VectorXd slopes = vec(j[QStringLiteral("truth")].toObject()[QStringLiteral("slopes")]);
        QVERIFY(maxDiff(c.gyroTc.col(0), slopes) < 0.002);
        QCOMPARE(w.setFlags, quint32(CalGyroTc | CalAccelTc));
        const QJsonObject cli = j[QStringLiteral("cli")].toObject();
        QStringList expected = cli[QStringLiteral("fields")].toObject().keys();
        QStringList got = w.fields.keys();
        expected.sort();
        got.sort();
        QCOMPARE(got, expected);
        for (const QString &name : got) {
            const Eigen::MatrixXd a = fieldMatrix(w.fields, name), b = cliField(cli, name);
            // The tool prints %.9g.
            QVERIFY2(maxDiff(a, b) <= 1e-8 * std::max(1.0, b.cwiseAbs().maxCoeff()) + 1e-12, qPrintable(name));
        }
    }

    void imuCalSubtractsAStoredTemperatureModel() {
        const Recording rec = log(QStringLiteral("imu"));
        CalibrationSet current = defaultCalibration();
        current[QStringLiteral("flags")] = {double(CalAccelTc | CalGyroTc)};
        current[QStringLiteral("accel_tc_tref")] = {30.0};
        current[QStringLiteral("accel_tc_z")] = {0.01, 0, 0};
        current[QStringLiteral("gyro_tc_tref")] = {30.0};
        current[QStringLiteral("gyro_tc_x")] = {0.05, 0, 0};
        // The recording is at about 31 C: 1 C above t_ref, so the stored polynomial
        // moves the z accelerometer reading by 0.01 g and the x gyro by 0.05 dps, and
        // the biases follow; the reference temperatures are left alone.
        const WorkflowResult w = runImuCal(rec, current);
        QVERIFY(w.ok);
        QVERIFY(!w.fields.contains(QStringLiteral("accel_tc_tref")));
        QVERIFY(!w.fields.contains(QStringLiteral("gyro_tc_tref")));
        const WorkflowResult plain = runImuCal(rec, {});
        QVERIFY(plain.fields.contains(QStringLiteral("accel_tc_tref")));
        const double t = w.imu->tempC;
        const Eigen::MatrixXd da = fieldMatrix(w.fields, QStringLiteral("accel_bias")) - fieldMatrix(plain.fields, QStringLiteral("accel_bias"));
        const Eigen::MatrixXd dg = fieldMatrix(w.fields, QStringLiteral("gyro_bias")) - fieldMatrix(plain.fields, QStringLiteral("gyro_bias"));
        QVERIFY(std::abs(da(0, 2) + 0.01 * (t - 30.0)) < 2e-5);
        QVERIFY(std::abs(dg(0, 0) + 0.05 * (t - 30.0)) < 1e-6);
        const Eigen::MatrixX3d tc = tempco(current, QStringLiteral("accel"), Eigen::Vector2d(30.0, 40.0));
        QCOMPARE(tc(0, 2), 0.0);
        QVERIFY(std::abs(tc(1, 2) - 0.1) < 1e-15);
    }

    void magCalRefusalsAndWarnings() {
        const Recording rec = log(QStringLiteral("mag"));
        // With a board attached and the accelerometer uncalibrated, the tool warns.
        const WorkflowResult w = runMagCal(rec, defaultCalibration(), MagOptions{true, false});
        QVERIFY(w.ok && w.writable);
        QVERIFY(hasWarning(w.warnings, Warning::Code::AccelUncalibrated));
        // Too little data.
        Recording small;
        for (int i = 0; i < 50; ++i) small.feed(QStringLiteral("$MAG,%1,1,2,3").arg(i));
        const WorkflowResult none = runMagCal(small, {});
        QVERIFY(!none.ok && !none.error.isEmpty());
    }

    void progressFigures() {
        const Recording rec = log(QStringLiteral("imu"));
        const ImuProgress p = imuProgress(rec);
        QCOMPARE(p.positions, 17);
        QVERIFY(p.orientations >= 9);
        const MagProgress m = magProgress(log(QStringLiteral("mag")));
        QVERIFY(m.coverage > 0.8);
        QCOMPARE(static_cast<int>(m.cells.size()), 72);
        QVERIFY(m.radius > 30.0);
        const ThermalProgress t = thermalProgress(log(QStringLiteral("thermal_tool")));
        QVERIFY(t.span() > 24.0 && t.span() < 25.5);
    }
};

QTEST_GUILESS_MAIN(TestCalib)
#include "tst_calib.moc"
