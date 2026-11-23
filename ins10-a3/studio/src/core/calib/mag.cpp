#include "core/calib/mag.h"

#include "core/attitude.h"
#include "core/calib/stats.h"

#include <QCoreApplication>

#include <Eigen/Eigenvalues>
#include <Eigen/LU>
#include <Eigen/SVD>

#include <cmath>
#include <numbers>
#include <set>

namespace ins10::calib {

namespace {
struct Text {
    Q_DECLARE_TR_FUNCTIONS(ins10::Calibration)
};
} // namespace

QStringList warningTexts(const Warnings &warnings) {
    QStringList out;
    for (const Warning &w : warnings) out.append(w.text);
    return out;
}

bool hasWarning(const Warnings &warnings, Warning::Code code) {
    for (const Warning &w : warnings) {
        if (w.code == code) return true;
    }
    return false;
}

Eigen::Matrix3d symmetric(const double *p) {
    Eigen::Matrix3d m;
    m << p[0], p[3], p[4], p[3], p[1], p[5], p[4], p[5], p[2];
    return m;
}

EllipsoidFit fitEllipsoid(const Eigen::MatrixX3d &x) {
    const Eigen::Index n = x.rows();
    const Eigen::RowVector3d c0 = x.colwise().mean();
    const Eigen::MatrixX3d centered = x.rowwise() - c0;
    const double scale = std::sqrt(centered.rowwise().squaredNorm().mean());
    if (!(scale > 0.0)) throw CalibrationError(CalibrationError::Code::NoSpread, Text::tr("magnetometer data has no spread"));
    const Eigen::MatrixX3d y = centered / scale;
    Eigen::MatrixXd design(n, 10);
    for (Eigen::Index i = 0; i < n; ++i) {
        const double a = y(i, 0), b = y(i, 1), c = y(i, 2);
        design.row(i) << a * a, b * b, c * c, 2 * a * b, 2 * a * c, 2 * b * c, 2 * a, 2 * b, 2 * c, 1.0;
    }
    // The right singular vector of the smallest singular value.
    const Eigen::JacobiSVD<Eigen::MatrixXd> svd(design, Eigen::ComputeFullV);
    Eigen::VectorXd p = svd.matrixV().col(9);
    Eigen::Matrix3d q = symmetric(p.data());
    Eigen::Vector3d u(p[6], p[7], p[8]);
    double k = p[9];
    if (q.trace() < 0) {
        q = -q;
        u = -u;
        k = -k;
    }
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(q);
    if (eig.eigenvalues().minCoeff() <= 0.0) {
        throw CalibrationError(CalibrationError::Code::NotAnEllipsoid,
                               Text::tr("the samples do not describe an ellipsoid: rotate the board through many more "
                                        "orientations"));
    }
    const Eigen::Vector3d center = -q.lu().solve(u);
    const double gain = center.dot(q * center) - k;
    if (gain <= 0.0) throw CalibrationError(CalibrationError::Code::DegenerateFit, Text::tr("degenerate ellipsoid fit"));
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> scaled(q / gain);
    const Eigen::Matrix3d v = scaled.eigenvectors();
    const Eigen::Matrix3d a = v * scaled.eigenvalues().cwiseSqrt().asDiagonal() * v.transpose();
    EllipsoidFit fit;
    fit.center = c0.transpose() + scale * center;
    fit.a = a / scale;
    return fit;
}

int sphereCell(const Eigen::Vector3d &direction, int bands, int sectors) {
    const double norm = direction.norm();
    if (!(norm > 0.0)) return -1;
    const Eigen::Vector3d d = direction / norm;
    const int band = std::clamp(static_cast<int>((d[2] + 1.0) * 0.5 * bands), 0, bands - 1);
    const int sector =
        static_cast<int>((std::atan2(d[1], d[0]) + std::numbers::pi) / (2.0 * std::numbers::pi) * sectors) % sectors;
    return band * sectors + sector;
}

double sphereCoverage(const Eigen::MatrixX3d &directions, int bands, int sectors, std::vector<bool> *cells) {
    std::vector<bool> hit(static_cast<size_t>(bands * sectors), false);
    for (Eigen::Index i = 0; i < directions.rows(); ++i) {
        const int cell = sphereCell(directions.row(i).transpose(), bands, sectors);
        if (cell >= 0) hit[static_cast<size_t>(cell)] = true;
    }
    const auto count = std::count(hit.begin(), hit.end(), true);
    if (cells) *cells = hit;
    return static_cast<double>(count) / (bands * sectors);
}

Alignment alignToAccel(const Eigen::MatrixX3d &magUnit, const Eigen::MatrixX3d &accel, const LsqOptions &options,
                       Hemisphere hemisphere) {
    const Eigen::Index n = accel.rows();
    const Eigen::MatrixX3d ah = accel.array().colwise() / accel.rowwise().norm().array();
    Eigen::MatrixXd design(n, 10);
    for (Eigen::Index i = 0; i < n; ++i) {
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) design(i, 3 * r + c) = ah(i, r) * magUnit(i, c);
        }
        design(i, 9) = -1.0;
    }
    const Eigen::JacobiSVD<Eigen::MatrixXd> svd(design, Eigen::ComputeFullV);
    const Eigen::VectorXd p = svd.matrixV().col(9);
    Eigen::Matrix3d pm;
    pm << p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8];
    const Eigen::JacobiSVD<Eigen::Matrix3d> s3(pm, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const Eigen::Vector3d sv = s3.singularValues();
    Alignment out;
    out.conditioning = sv.minCoeff() / sv.maxCoeff();
    Eigen::Matrix3d r0 = s3.matrixU() * s3.matrixV().transpose();
    double c0 = p[9] / sv.mean();
    // (R, c) and (-R, -c) fit equally well. calib.py keeps the one with det R = +1;
    // here the field's direction decides (c = a . B is negative where it points
    // down), so a magnetometer with mirrored axes is aligned with a reflection
    // instead of being turned upside down. Near the magnetic equator c says nothing.
    const bool flip = std::abs(c0) < kMinSinDipForSign ? r0.determinant() < 0
                                                        : (hemisphere == Hemisphere::North) == (c0 > 0);
    if (flip) {
        r0 = -r0;
        c0 = -c0;
    }
    const auto residual = [&](const Eigen::VectorXd &params) {
        const Eigen::Matrix3d r = r0 * rotvecToMatrix(params.head<3>());
        const Eigen::MatrixX3d rotated = magUnit * r.transpose();
        return Eigen::VectorXd((ah.array() * rotated.array()).rowwise().sum() - params[3]);
    };
    LsqOptions o = options;
    o.loss = Loss::SoftL1;
    o.fScale = 0.03;
    const LsqResult fit = leastSquares(residual, Eigen::Vector4d(0.0, 0.0, 0.0, c0), o);
    out.r = r0 * rotvecToMatrix(fit.x.head<3>());
    const double constant = std::clamp(fit.x[3], -1.0, 1.0);
    const Eigen::VectorXd res = residual(fit.x);
    const double cosDip = std::max(std::sqrt(1.0 - constant * constant), 1e-3);
    out.residualDeg = degrees(robustRms(res) / cosDip);
    out.sinDip = -constant;
    out.mirrored = out.r.determinant() < 0;
    return out;
}

MagCalibration calibrateMag(const Eigen::MatrixX3d &mag, const Eigen::MatrixX3d *accel, const Eigen::MatrixX3d *gyro,
                            const std::atomic<bool> *cancel, Hemisphere hemisphere) {
    const Eigen::Index n = mag.rows();
    if (n < 200) {
        throw CalibrationError(CalibrationError::Code::TooFewSamples,
                               Text::tr("only %1 magnetometer samples; record at least 20 s").arg(n));
    }
    MagCalibration out;
    const EllipsoidFit algebraic = fitEllipsoid(mag);
    const double fieldAlgebraic = std::pow(algebraic.a.determinant(), -1.0 / 3.0);

    // Geometric refinement with a robust loss: the algebraic fit is biased by noise,
    // and a few samples taken next to something magnetic should not bend the result.
    const auto residual = [&](const Eigen::VectorXd &params) {
        const Eigen::Vector3d c = params.head<3>();
        const Eigen::Matrix3d m = symmetric(params.data() + 3);
        return Eigen::VectorXd(((mag.rowwise() - c.transpose()) * m.transpose()).rowwise().norm().array() - fieldAlgebraic);
    };
    const Eigen::Matrix3d a0 = algebraic.a * fieldAlgebraic;
    Eigen::VectorXd start(9);
    start << algebraic.center, a0(0, 0), a0(1, 1), a0(2, 2), a0(0, 1), a0(0, 2), a0(1, 2);
    LsqOptions options;
    options.loss = Loss::SoftL1;
    options.fScale = 0.02 * fieldAlgebraic;
    options.cancel = cancel;
    const LsqResult fit = leastSquares(residual, start, options);
    Eigen::Vector3d center = fit.x.head<3>();
    Eigen::Matrix3d a = symmetric(fit.x.data() + 3);
    if (Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(a).eigenvalues().minCoeff() <= 0.0) {
        throw CalibrationError(CalibrationError::Code::SoftIronNotPositive,
                               Text::tr("soft-iron matrix is not positive definite; record again"));
    }
    const double norm = std::pow(a.determinant(), 1.0 / 3.0); // keep the average gain at one
    a /= norm;
    const double field = fieldAlgebraic / norm;
    const Eigen::MatrixX3d calibrated = (mag.rowwise() - center.transpose()) * a.transpose();
    const Eigen::VectorXd radius = calibrated.rowwise().norm();
    const double normResidual = std::sqrt(median(Eigen::VectorXd((radius.array() - field).square())) * 2.198) / field * 100.0;
    const double coverage = sphereCoverage(calibrated);

    if (coverage < 0.6) {
        out.warnings.append({Warning::Code::CoverageLow,
                             Text::tr("only %1 % of all directions were sampled; the soft-iron estimate is weak. Rotate "
                                      "about all three axes, including upside down.")
                                 .arg(coverage * 100.0, 0, 'f', 0)});
    }
    if (!(field > 20.0 && field < 70.0)) {
        out.warnings.append({Warning::Code::FieldOutOfRange,
                             Text::tr("field magnitude %1 uT is outside the earth's range (22-67 uT): strong magnetic "
                                      "material nearby?")
                                 .arg(field, 0, 'f', 1)});
    }
    if (normResidual > 2.0) {
        out.warnings.append({Warning::Code::ResidualHigh,
                             Text::tr("residual %1 % is high: the field was not uniform (moving near steel, cables, "
                                      "magnets) or the sensor is saturating.")
                                 .arg(normResidual, 0, 'f', 1)});
    }

    Eigen::Matrix3d matrix = a;
    if (accel && accel->rows() == n) {
        const Eigen::VectorXd an = accel->rowwise().norm();
        const double anMedian = median(an);
        std::vector<Eigen::Index> keep;
        for (Eigen::Index i = 0; i < n; ++i) {
            bool k = std::abs(an[i] / anMedian - 1.0) < 0.03;
            if (gyro && gyro->rows() == n) k = k && gyro->row(i).norm() < 60.0;
            k = k && std::abs(radius[i] / field - 1.0) < 0.05;
            if (k) keep.push_back(i);
        }
        if (keep.size() < 150) {
            out.warnings.append({Warning::Code::TooFewSmoothSamples,
                                 Text::tr("too few samples in slow, smooth motion to align the magnetometer to the IMU: "
                                          "turn the board more slowly.")});
        } else {
            const Eigen::Index m = static_cast<Eigen::Index>(keep.size());
            Eigen::MatrixX3d unit(m, 3), acc(m, 3);
            for (Eigen::Index j = 0; j < m; ++j) {
                unit.row(j) = calibrated.row(keep[static_cast<size_t>(j)]) / field;
                acc.row(j) = accel->row(keep[static_cast<size_t>(j)]);
            }
            LsqOptions alignOptions;
            alignOptions.cancel = cancel;
            const Alignment al = alignToAccel(unit, acc, alignOptions, hemisphere);
            if (al.conditioning < 0.6) {
                out.warnings.append({Warning::Code::AlignmentPoor,
                                     Text::tr("the alignment to the IMU is poorly determined by this recording; it was not "
                                              "applied.")});
            } else {
                matrix = al.r * a;
                out.aligned = true;
                out.mirrored = al.mirrored;
                out.dipDeg = degrees(std::asin(std::clamp(al.sinDip, -1.0, 1.0)));
                out.dipResidualDeg = al.residualDeg;
                if (al.mirrored) {
                    out.warnings.append({Warning::Code::Mirrored,
                                         Text::tr("the magnetometer's axes are a mirror image of the accelerometer's, so "
                                                  "the alignment is a reflection rather than a rotation (as on the INS10-A3). "
                                                  "The calibration includes the reflection; the heading is right if the "
                                                  "hemisphere setting is.")});
                }
                if (al.residualDeg > 2.0) {
                    out.warnings.append({Warning::Code::DipVaries,
                                         Text::tr("dip angle varies by %1 deg over the recording: motion too fast or "
                                                  "field not uniform.")
                                             .arg(al.residualDeg, 0, 'f', 1)});
                }
            }
        }
    }
    out.bias = center;
    out.matrix = matrix;
    out.fieldUt = field;
    out.normResidualPct = normResidual;
    out.coverage = coverage;
    out.samples = static_cast<int>(n);
    return out;
}

} // namespace ins10::calib
