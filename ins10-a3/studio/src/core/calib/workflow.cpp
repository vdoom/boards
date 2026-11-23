#include "core/calib/workflow.h"

#include "core/calib/stats.h"

#include <QCoreApplication>

#include <cmath>

namespace ins10::calib {

namespace {
struct Text {
    Q_DECLARE_TR_FUNCTIONS(ins10::Calibration)
};

quint32 currentFlags(const CalibrationSet &cal) { return calFlags(cal); }

QList<double> vec(const Eigen::Vector3d &v) { return {v[0], v[1], v[2]}; }

QList<double> mat(const Eigen::Matrix3d &m) {
    QList<double> out;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) out.append(m(r, c));
    }
    return out;
}

Eigen::Vector3d calVector(const CalibrationSet &cal, const char *name) {
    const QList<double> v = cal.value(QLatin1String(name));
    return v.size() == 3 ? Eigen::Vector3d(v[0], v[1], v[2]) : Eigen::Vector3d::Zero();
}

Eigen::Matrix3d calMatrix(const CalibrationSet &cal, const char *name) {
    const QList<double> v = cal.value(QLatin1String(name));
    Eigen::Matrix3d m = Eigen::Matrix3d::Identity();
    if (v.size() == 9) {
        for (int i = 0; i < 9; ++i) m(i / 3, i % 3) = v[i];
    }
    return m;
}

void fail(WorkflowResult &r, const CalibrationError &e) {
    r.ok = false;
    r.error = e.message();
    r.errorCode = e.code();
}
} // namespace

Eigen::MatrixX3d tempco(const CalibrationSet &cal, const QString &prefix, const Eigen::VectorXd &temp) {
    const QList<double> ref = cal.value(prefix + QStringLiteral("_tc_tref"), {25.0});
    const Eigen::VectorXd d = temp.array() - (ref.isEmpty() ? 25.0 : ref.front());
    Eigen::MatrixX3d out(temp.size(), 3);
    const char *axes[3] = {"_tc_x", "_tc_y", "_tc_z"};
    for (int a = 0; a < 3; ++a) {
        QList<double> c = cal.value(prefix + QLatin1String(axes[a]), {0.0, 0.0, 0.0});
        if (c.size() != 3) c = {0.0, 0.0, 0.0};
        out.col(a) = c[0] * d.array() + c[1] * d.array().square() + c[2] * d.array().cube();
    }
    return out;
}

WorkflowResult runImuCal(const Recording &rec, const CalibrationSet &current, const std::atomic<bool> *cancel) {
    WorkflowResult r;
    r.procedure = Procedure::Imu;
    ImuArrays a = rec.imuArrays();
    const quint32 flags = currentFlags(current);
    // The bias then refers to the reference temperature of an existing temperature
    // calibration.
    if (flags & CalAccelTc) a.accel -= tempco(current, QStringLiteral("accel"), a.temp);
    if (flags & CalGyroTc) a.gyro -= tempco(current, QStringLiteral("gyro"), a.temp);
    const std::vector<bool> contiguous = rec.contiguity();
    try {
        ImuCalibration result = calibrateImu(a.t, a.dt, a.gyro, a.accel, a.temp, &contiguous, cancel);
        r.fields.insert(QStringLiteral("accel_bias"), vec(result.accelBias));
        r.fields.insert(QStringLiteral("accel_matrix"), mat(result.accelMatrix));
        r.fields.insert(QStringLiteral("gyro_bias"), vec(result.gyroBias));
        r.setFlags = CalAccel;
        if (result.gyroMatrixFitted != QLatin1String("none")) {
            r.fields.insert(QStringLiteral("gyro_matrix"), mat(result.gyroMatrix));
            r.setFlags |= CalGyro;
        }
        if (!(flags & CalAccelTc)) r.fields.insert(QStringLiteral("accel_tc_tref"), {result.tempC});
        if (!(flags & CalGyroTc)) r.fields.insert(QStringLiteral("gyro_tc_tref"), {result.tempC});
        r.warnings = result.warnings;
        r.imu = std::move(result);
        r.ok = r.writable = true;
    } catch (const CalibrationError &e) {
        fail(r, e);
    }
    return r;
}

WorkflowResult runMagCal(const Recording &rec, const CalibrationSet &current, const MagOptions &options,
                         const std::atomic<bool> *cancel) {
    WorkflowResult r;
    r.procedure = Procedure::Mag;
    MagWithImu m = rec.magWithImu();
    const quint32 flags = currentFlags(current);
    if (flags & CalAccel) {
        const Eigen::Vector3d bias = calVector(current, "accel_bias");
        const Eigen::Matrix3d matrix = calMatrix(current, "accel_matrix");
        m.accel = (m.accel.rowwise() - bias.transpose()) * matrix.transpose();
    }
    r.magInput = m.mag;
    try {
        MagCalibration result = calibrateMag(m.mag, &m.accel, &m.gyro, cancel,
                                             options.southern ? Hemisphere::South : Hemisphere::North);
        if (options.deviceConnected && !(flags & CalAccel)) {
            result.warnings.append({Warning::Code::AccelUncalibrated,
                                    Text::tr("the accelerometer is not calibrated yet. The magnetometer is aligned to its "
                                             "axes, so its bias (up to about a degree) goes into the alignment. Calibrate "
                                             "the accelerometer first, then the magnetometer again.")});
        }
        if (result.aligned && result.dipDeg && *result.dipDeg < 0.0 && !options.southern) {
            result.warnings.append({Warning::Code::DipNegative,
                                    Text::tr("the dip angle is negative, which is right only in the southern hemisphere. "
                                             "In the north it means one sensor's axes are mirrored; check the wiring and "
                                             "driver before trusting the heading.")});
        }
        r.warnings = result.warnings;
        r.ok = true;
        if (!result.aligned) {
            r.writable = false;
            r.refusal = Text::tr("The result was not aligned to the IMU frame and will not be written.");
        } else {
            r.writable = true;
            r.fields.insert(QStringLiteral("mag_bias"), vec(result.bias));
            r.fields.insert(QStringLiteral("mag_matrix"), mat(result.matrix));
            r.fields.insert(QStringLiteral("mag_field"), {result.fieldUt});
            r.fields.insert(QStringLiteral("mag_dip"), {*result.dipDeg});
            r.setFlags = CalMag;
        }
        r.mag = std::move(result);
    } catch (const CalibrationError &e) {
        fail(r, e);
    }
    return r;
}

WorkflowResult runThermalCal(const Recording &rec, const CalibrationSet &current, const std::atomic<bool> *cancel) {
    WorkflowResult r;
    r.procedure = Procedure::Thermal;
    const ImuArrays a = rec.imuArrays();
    const quint32 flags = currentFlags(current);
    std::optional<double> accelTRef;
    if (flags & CalAccel) {
        const QList<double> ref = current.value(QStringLiteral("accel_tc_tref"));
        if (!ref.isEmpty()) accelTRef = ref.front();
    }
    r.thermalTemp = a.temp;
    r.thermalGyro = a.gyro;
    r.thermalAccel = a.accel;
    try {
        if (cancel && cancel->load()) throw CalibrationError(CalibrationError::Code::Cancelled, Text::tr("cancelled"));
        ThermalCalibration result = calibrateThermal(a.temp, a.gyro, a.accel, accelTRef);
        r.fields.insert(QStringLiteral("gyro_bias"), vec(result.gyroBias));
        r.fields.insert(QStringLiteral("gyro_tc_tref"), {result.tRef});
        r.fields.insert(QStringLiteral("accel_tc_tref"), {result.accelTRef});
        const char *axes[3] = {"x", "y", "z"};
        for (int i = 0; i < 3; ++i) {
            r.fields.insert(QStringLiteral("gyro_tc_") + QLatin1String(axes[i]), vec(result.gyroTc.row(i).transpose()));
            r.fields.insert(QStringLiteral("accel_tc_") + QLatin1String(axes[i]), vec(result.accelTc.row(i).transpose()));
        }
        r.setFlags = CalGyroTc | CalAccelTc;
        r.warnings = result.warnings;
        r.thermal = std::move(result);
        r.ok = r.writable = true;
    } catch (const CalibrationError &e) {
        fail(r, e);
    }
    return r;
}

ImuProgress imuProgress(const Recording &rec) {
    ImuProgress p;
    const ImuArrays a = rec.imuArrays();
    p.intervals = findStaticIntervals(a.t, a.dt, a.gyro, a.accel, a.temp);
    p.positions = static_cast<int>(p.intervals.size());
    if (!p.intervals.empty()) {
        Eigen::MatrixX3d means(static_cast<Eigen::Index>(p.intervals.size()), 3);
        for (size_t i = 0; i < p.intervals.size(); ++i) means.row(static_cast<Eigen::Index>(i)) = p.intervals[i].accel.transpose();
        p.orientations = countOrientations(means);
    }
    return p;
}

MagProgress magProgress(const Recording &rec) {
    MagProgress p;
    p.samples = static_cast<int>(rec.magRows());
    p.cells.assign(72, false);
    if (p.samples < 20) return p;
    Eigen::MatrixX3d m(rec.magRows(), 3);
    for (Eigen::Index i = 0; i < m.rows(); ++i) m.row(i) = rec.mag()[static_cast<size_t>(i)].transpose();
    p.center = 0.5 * (m.colwise().maxCoeff() + m.colwise().minCoeff()).transpose();
    const Eigen::MatrixX3d centered = m.rowwise() - p.center.transpose();
    p.coverage = sphereCoverage(centered, 6, 12, &p.cells);
    p.radius = centered.rowwise().norm().mean();
    return p;
}

ThermalProgress thermalProgress(const Recording &rec) {
    ThermalProgress p;
    p.samples = static_cast<int>(rec.imuRows());
    if (p.samples == 0) return p;
    const auto &t = rec.temp();
    p.now = t.back();
    p.min = *std::min_element(t.begin(), t.end());
    p.max = *std::max_element(t.begin(), t.end());
    if (p.samples >= 3) {
        const ImuArrays a = rec.imuArrays();
        const Eigen::RowVector3d med = columnMedian(a.gyro);
        p.gyroMotionDps = (a.gyro.rowwise() - med).rowwise().norm().maxCoeff();
    }
    return p;
}

} // namespace ins10::calib
