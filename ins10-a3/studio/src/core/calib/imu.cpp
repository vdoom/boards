#include "core/calib/imu.h"

#include "core/attitude.h"
#include "core/calib/stats.h"

#include <QCoreApplication>

#include <Eigen/SVD>

#include <algorithm>
#include <cmath>

namespace ins10::calib {

namespace {
struct Text {
    Q_DECLARE_TR_FUNCTIONS(ins10::Calibration)
};

void checkCancel(const std::atomic<bool> *cancel) {
    if (cancel && cancel->load(std::memory_order_relaxed))
        throw CalibrationError(CalibrationError::Code::Cancelled, Text::tr("cancelled"));
}
} // namespace

std::vector<StaticInterval> findStaticIntervals(const Eigen::VectorXd &t, const Eigen::VectorXd &dt,
                                                const Eigen::MatrixX3d &gyro, const Eigen::MatrixX3d &accel,
                                                const Eigen::VectorXd &temp, const StaticOptions &o) {
    std::vector<StaticInterval> intervals;
    const Eigen::Index n = t.size();
    if (n < 8) return intervals;

    // edges = np.arange(t[0], t[-1] + window, window), element for element as numpy
    // fills it: start, start + window, then start + i * (second - first).
    const double start = t[0];
    const double stop = t[n - 1] + o.windowS;
    const auto length = static_cast<Eigen::Index>(std::max(0.0, std::ceil((stop - start) / o.windowS)));
    std::vector<double> edges(static_cast<size_t>(length));
    if (length >= 1) edges[0] = start;
    if (length >= 2) edges[1] = start + o.windowS;
    const double delta = length >= 2 ? edges[1] - edges[0] : o.windowS;
    for (Eigen::Index i = 2; i < length; ++i) edges[static_cast<size_t>(i)] = start + static_cast<double>(i) * delta;
    // np.searchsorted(t, edges), side "left".
    std::vector<Eigen::Index> index(edges.size());
    for (size_t i = 0; i < edges.size(); ++i)
        index[i] = std::lower_bound(t.data(), t.data() + n, edges[i]) - t.data();

    struct Window {
        Eigen::Index i0, i1;
        bool quiet;
    };
    std::vector<Window> windows;
    for (size_t k = 0; k + 1 < index.size(); ++k) {
        const Eigen::Index i0 = index[k], i1 = index[k + 1];
        if (i1 - i0 < 3) {
            windows.push_back({i0, i1, false});
            continue;
        }
        const bool quiet = columnStd(gyro, i0, i1).maxCoeff() < o.gyroStdDps &&
                           columnStd(accel, i0, i1).maxCoeff() < o.accelStdG &&
                           columnMean(gyro, i0, i1).cwiseAbs().maxCoeff() < 5.0;
        windows.push_back({i0, i1, quiet});
    }

    std::vector<std::pair<Eigen::Index, Eigen::Index>> run;
    const auto closeRun = [&] {
        // Drop the first and last window: they contain the settling and the pick-up.
        if (run.size() >= 4) {
            const Eigen::Index i0 = run[1].first, i1 = run[run.size() - 2].second;
            const double duration = dt.segment(i0, i1 - i0).sum();
            if (duration >= o.minDurationS) {
                StaticInterval iv;
                iv.start = i0;
                iv.stop = i1;
                iv.duration = duration;
                const Eigen::VectorXd w = dt.segment(i0, i1 - i0) / duration;
                iv.accel = (w.transpose() * accel.middleRows(i0, i1 - i0)).transpose();
                iv.gyro = (w.transpose() * gyro.middleRows(i0, i1 - i0)).transpose();
                iv.temp = w.dot(temp.segment(i0, i1 - i0));
                intervals.push_back(iv);
            }
        }
        run.clear();
    };
    for (const Window &w : windows) {
        // Still, but in a new orientation: a slow slide between two rests.
        if (w.quiet && !run.empty() &&
            (columnMean(accel, w.i0, w.i1) - columnMean(accel, run[0].first, run[0].second)).cwiseAbs().maxCoeff() > 0.01)
            closeRun();
        if (w.quiet) run.emplace_back(w.i0, w.i1);
        else closeRun();
    }
    closeRun();
    return intervals;
}

int countOrientations(const Eigen::MatrixX3d &directions, double minAngleDeg) {
    std::vector<Eigen::Vector3d> chosen;
    const double limit = std::cos(radians(minAngleDeg));
    for (Eigen::Index i = 0; i < directions.rows(); ++i) {
        const Eigen::Vector3d d = directions.row(i).transpose() / directions.row(i).norm();
        if (std::all_of(chosen.begin(), chosen.end(), [&](const Eigen::Vector3d &c) { return d.dot(c) < limit; }))
            chosen.push_back(d);
    }
    return static_cast<int>(chosen.size());
}

AccelCalibration calibrateAccel(const Eigen::MatrixX3d &means, const std::atomic<bool> *cancel) {
    const int orientations = countOrientations(means);
    if (orientations < 6) {
        throw CalibrationError(CalibrationError::Code::TooFewOrientations,
                               Text::tr("only %1 distinct orientations found; at least 6 are needed (each face of the "
                                        "board up), 12 or more are better")
                                   .arg(orientations));
    }
    const bool full = orientations >= 9;
    const auto unpack = [full](const Eigen::VectorXd &p, Eigen::Vector3d &b, Eigen::Matrix3d &m) {
        b = p.head<3>();
        m = Eigen::Vector3d(p[3], p[4], p[5]).asDiagonal();
        if (full) {
            m(0, 1) = m(1, 0) = p[6];
            m(0, 2) = m(2, 0) = p[7];
            m(1, 2) = m(2, 1) = p[8];
        }
    };
    const auto residual = [&](const Eigen::VectorXd &p) {
        Eigen::Vector3d b;
        Eigen::Matrix3d m;
        unpack(p, b, m);
        return Eigen::VectorXd(((means.rowwise() - b.transpose()) * m.transpose()).rowwise().norm().array() - 1.0);
    };
    Eigen::VectorXd start(full ? 9 : 6);
    start.setZero();
    start.segment<3>(3).setOnes();
    AccelCalibration out;
    out.residualBeforeMg = std::sqrt(residual(start).array().square().mean()) * 1000.0;
    LsqOptions options;
    options.cancel = cancel;
    const LsqResult fit = leastSquares(residual, start, options);
    unpack(fit.x, out.bias, out.matrix);
    out.residualMg = std::sqrt(fit.residuals.array().square().mean()) * 1000.0;
    out.orientations = orientations;
    if (out.bias.cwiseAbs().maxCoeff() > 0.15 || (out.matrix.diagonal().array() - 1.0).abs().maxCoeff() > 0.1) {
        throw CalibrationError(CalibrationError::Code::AccelOutOfTolerance,
                               Text::tr("accelerometer calibration is far outside the sensor's tolerance; were the rest "
                                        "positions really at rest?"));
    }
    return out;
}

namespace {
struct Turn {
    Eigen::Index i0, i1;
    Eigen::Vector3d u0, u1;
};

double rmsDeg(const Eigen::VectorXd &res) {
    double sum = 0.0;
    const Eigen::Index k = res.size() / 3;
    for (Eigen::Index i = 0; i < k; ++i) sum += res.segment<3>(3 * i).squaredNorm();
    return degrees(std::sqrt(sum / static_cast<double>(k)));
}
} // namespace

GyroCalibration calibrateGyro(const Eigen::VectorXd &dt, const Eigen::MatrixX3d &gyro,
                              const std::vector<StaticInterval> &intervals, const Eigen::Vector3d &accelBias,
                              const Eigen::Matrix3d &accelMatrix, const std::vector<bool> *contiguous,
                              const std::atomic<bool> *cancel) {
    GyroCalibration out;
    double weightSum = 0.0;
    Eigen::Vector3d biasSum = Eigen::Vector3d::Zero();
    for (const StaticInterval &iv : intervals) {
        biasSum += iv.duration * iv.gyro;
        weightSum += iv.duration;
    }
    const Eigen::Vector3d bias = biasSum / weightSum;
    std::vector<Eigen::Vector3d> up;
    for (const StaticInterval &iv : intervals) up.push_back((accelMatrix * (iv.accel - accelBias)).normalized());

    std::vector<Turn> turns;
    for (size_t k = 0; k + 1 < intervals.size(); ++k) {
        // From the middle of one rest to the middle of the next: the gyro reads only its
        // bias there, so the end points do not matter.
        const Eigen::Index i0 = (intervals[k].start + intervals[k].stop) / 2;
        const Eigen::Index i1 = (intervals[k + 1].start + intervals[k + 1].stop) / 2;
        if (contiguous) {
            bool whole = true;
            for (Eigen::Index i = i0; i < i1; ++i) whole = whole && (*contiguous)[static_cast<size_t>(i)];
            if (!whole) continue;
        }
        const double angle = degrees(std::acos(std::clamp(up[k].dot(up[k + 1]), -1.0, 1.0)));
        if (angle < 20.0) continue; // nothing to learn from a turn that barely moved gravity
        turns.push_back({i0, i1, up[k], up[k + 1]});
    }
    out.bias = bias;
    out.matrix = Eigen::Matrix3d::Identity();
    out.mode = QStringLiteral("none");
    out.turns = static_cast<int>(turns.size());
    if (turns.size() < 6) return out;

    const auto residualFor = [&](const Eigen::Matrix3d &matrix) {
        checkCancel(cancel);
        Eigen::VectorXd res(3 * static_cast<Eigen::Index>(turns.size()));
        for (size_t k = 0; k < turns.size(); ++k) {
            const Turn &turn = turns[k];
            Eigen::Vector4d q(1.0, 0.0, 0.0, 0.0);
            for (Eigen::Index i = turn.i0; i < turn.i1; ++i) {
                const Eigen::Vector3d rate = matrix * (gyro.row(i).transpose() - bias);
                const Eigen::Vector3d rotvec(radians(rate[0]) * dt[i], radians(rate[1]) * dt[i], radians(rate[2]) * dt[i]);
                q = quatMultiply(q, quatFromRotvec(rotvec));
                if (((i - turn.i0) & 255) == 255) q.normalize();
            }
            q.normalize();
            res.segment<3>(3 * static_cast<Eigen::Index>(k)) = quatToDcm(q).transpose() * turn.u0 - turn.u1;
        }
        return res;
    };
    const double before = rmsDeg(residualFor(Eigen::Matrix3d::Identity()));
    LsqOptions options;
    options.cancel = cancel;
    const auto fullResidual = [&](const Eigen::VectorXd &p) {
        Eigen::Matrix3d m;
        m << p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8];
        return residualFor(m);
    };
    Eigen::VectorXd start(9);
    start << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    const LsqResult full = leastSquares(fullResidual, start, options);
    Eigen::VectorXd sv = Eigen::JacobiSVD<Eigen::MatrixXd>(full.jacobian).singularValues();
    Eigen::Matrix3d matrix;
    matrix << full.x[0], full.x[1], full.x[2], full.x[3], full.x[4], full.x[5], full.x[6], full.x[7], full.x[8];
    QString mode = QStringLiteral("full");
    Eigen::VectorXd res = full.residuals;
    if (turns.size() < 9 || sv.minCoeff() < 1e-3 * sv.maxCoeff() ||
        (matrix - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff() > 0.05) {
        // Not enough variety of rotation axes for nine parameters: scale factors only.
        const auto diagResidual = [&](const Eigen::VectorXd &p) { return residualFor(Eigen::Vector3d(p).asDiagonal()); };
        const LsqResult diag = leastSquares(diagResidual, Eigen::Vector3d::Ones(), options);
        sv = Eigen::JacobiSVD<Eigen::MatrixXd>(diag.jacobian).singularValues();
        matrix = Eigen::Vector3d(diag.x).asDiagonal();
        mode = QStringLiteral("scale");
        res = diag.residuals;
        if (sv.minCoeff() < 1e-2 * sv.maxCoeff() || (diag.x.array() - 1.0).abs().maxCoeff() > 0.05) {
            out.residualDeg = before;
            out.residualBeforeDeg = before;
            return out;
        }
    }
    out.matrix = matrix;
    out.mode = mode;
    out.residualDeg = rmsDeg(res);
    out.residualBeforeDeg = before;
    return out;
}

ImuCalibration calibrateImu(const Eigen::VectorXd &t, const Eigen::VectorXd &dt, const Eigen::MatrixX3d &gyro,
                            const Eigen::MatrixX3d &accel, const Eigen::VectorXd &temp,
                            const std::vector<bool> *contiguous, const std::atomic<bool> *cancel) {
    ImuCalibration out;
    out.intervals = findStaticIntervals(t, dt, gyro, accel, temp);
    const auto &intervals = out.intervals;
    if (intervals.size() < 6) {
        throw CalibrationError(CalibrationError::Code::TooFewRestPositions,
                               Text::tr("found %1 rest positions; at least 6 are needed. Leave the board untouched for "
                                        "3 s in each position.")
                                   .arg(intervals.size()));
    }
    Eigen::MatrixX3d means(static_cast<Eigen::Index>(intervals.size()), 3);
    double weights = 0.0, tempSum = 0.0;
    for (size_t i = 0; i < intervals.size(); ++i) {
        means.row(static_cast<Eigen::Index>(i)) = intervals[i].accel.transpose();
        weights += intervals[i].duration;
        tempSum += intervals[i].duration * intervals[i].temp;
    }
    const AccelCalibration a = calibrateAccel(means, cancel);
    const GyroCalibration g = calibrateGyro(dt, gyro, intervals, a.bias, a.matrix, contiguous, cancel);
    out.accelBias = a.bias;
    out.accelMatrix = a.matrix;
    out.accelResidualMg = a.residualMg;
    out.accelResidualBeforeMg = a.residualBeforeMg;
    out.gyroBias = g.bias;
    out.gyroMatrix = g.matrix;
    out.gyroResidualDeg = g.residualDeg;
    out.gyroResidualBeforeDeg = g.residualBeforeDeg;
    out.gyroMatrixFitted = g.mode;
    out.positions = static_cast<int>(intervals.size());
    out.orientations = a.orientations;
    out.tempC = tempSum / weights;
    if (a.orientations < 9) {
        out.warnings.append({Warning::Code::FewOrientations,
                             Text::tr("%1 distinct orientations: accelerometer scale and bias were fitted, cross-axis "
                                      "terms were not (needs 9 or more).")
                                 .arg(a.orientations)});
    }
    if (g.mode == QLatin1String("none")) {
        out.warnings.append({Warning::Code::GyroNotFitted,
                             Text::tr("gyro scale factors were not fitted: too few usable turns between rest positions, "
                                      "or too little variety of rotation axes.")});
    } else if (g.mode == QLatin1String("scale")) {
        out.warnings.append({Warning::Code::GyroScaleOnly,
                             Text::tr("gyro scale factors were fitted, misalignment was not (needs more turns about "
                                      "different axes).")});
    }
    if (a.residualMg > 3.0) {
        out.warnings.append({Warning::Code::AccelResidualHigh,
                             Text::tr("accelerometer residual %1 mg is high: was the board moving or vibrating in some "
                                      "positions?")
                                 .arg(a.residualMg, 0, 'f', 1)});
    }
    return out;
}

} // namespace ins10::calib
