#include "core/calib/thermal.h"

#include "core/calib/stats.h"

#include <QCoreApplication>

#include <Eigen/QR>

#include <cmath>

namespace ins10::calib {

namespace {
struct Text {
    Q_DECLARE_TR_FUNCTIONS(ins10::Calibration)
};

// np.vander(d, order + 1, increasing=True)
Eigen::MatrixXd vander(const Eigen::VectorXd &d, int order) {
    Eigen::MatrixXd x(d.size(), order + 1);
    x.col(0).setOnes();
    if (order >= 1) x.col(1) = d;
    for (int k = 2; k <= order; ++k) x.col(k) = x.col(k - 1).cwiseProduct(d);
    return x;
}
} // namespace

int thermalOrder(double spanC) { return spanC < 8.0 ? 1 : spanC < 20.0 ? 2 : 3; }

ClippedFit polyfitClipped(const Eigen::VectorXd &d, const Eigen::MatrixX3d &y, int order) {
    const Eigen::MatrixXd x = vander(d, order);
    const Eigen::Index n = d.size();
    std::vector<bool> keep(static_cast<size_t>(n), true);
    Eigen::MatrixXd coef;
    Eigen::MatrixX3d res;
    for (int pass = 0; pass < 2; ++pass) {
        Eigen::Index m = 0;
        for (bool k : keep) m += k;
        Eigen::MatrixXd xk(m, order + 1);
        Eigen::MatrixX3d yk(m, 3);
        for (Eigen::Index i = 0, j = 0; i < n; ++i) {
            if (!keep[static_cast<size_t>(i)]) continue;
            xk.row(j) = x.row(i);
            yk.row(j) = y.row(i);
            ++j;
        }
        coef = xk.colPivHouseholderQr().solve(yk);
        res = y - x * coef;
        Eigen::RowVector3d sigma;
        for (int c = 0; c < 3; ++c) {
            std::vector<double> squares;
            squares.reserve(static_cast<size_t>(m));
            for (Eigen::Index i = 0; i < n; ++i) {
                if (keep[static_cast<size_t>(i)]) squares.push_back(res(i, c) * res(i, c));
            }
            sigma[c] = std::sqrt(median(squares) * 2.198) + 1e-12;
        }
        for (Eigen::Index i = 0; i < n; ++i)
            keep[static_cast<size_t>(i)] = (res.row(i).cwiseAbs().array() < 4.0 * sigma.array()).all();
    }
    ClippedFit out;
    out.coef = coef;
    Eigen::Index m = 0;
    for (bool k : keep) m += k;
    out.keptResiduals.resize(m, 3);
    for (Eigen::Index i = 0, j = 0; i < n; ++i) {
        if (keep[static_cast<size_t>(i)]) out.keptResiduals.row(j++) = res.row(i);
    }
    return out;
}

ThermalCalibration calibrateThermal(const Eigen::VectorXd &temp, const Eigen::MatrixX3d &gyro,
                                    const Eigen::MatrixX3d &accel, std::optional<double> accelTRef) {
    if (temp.size() < 30) throw CalibrationError(CalibrationError::Code::RecordingTooShort, Text::tr("recording too short"));
    ThermalCalibration out;
    const double span = ptp(temp);
    if (span < 3.0) {
        throw CalibrationError(CalibrationError::Code::TemperatureSpanTooSmall,
                               Text::tr("temperature changed by only %1 C; at least 3 C is needed, 15 C or more is better "
                                        "(start from a cold board)")
                                   .arg(span, 0, 'f', 1));
    }
    const int order = thermalOrder(span);
    if (order < 3) {
        out.warnings.append({Warning::Code::ThermalOrderLow,
                             Text::tr("temperature span %1 C: fitted a polynomial of order %2. Outside the recorded range "
                                      "the compensation is an extrapolation.")
                                 .arg(span, 0, 'f', 1)
                                 .arg(order)});
    }
    const Eigen::RowVector3d gyroMedian = columnMedian(gyro);
    const double moved = ((gyro.rowwise() - gyroMedian).cwiseAbs()).rowwise().norm().maxCoeff();
    if (moved > 2.0) {
        out.warnings.append({Warning::Code::ThermalMoved,
                             Text::tr("the board moved during the recording; those samples were rejected.")});
    }
    const double tRef = roundHalfEven(median(temp));
    const ClippedFit g = polyfitClipped(temp.array() - tRef, gyro, order);
    const double aRef = accelTRef.value_or(tRef);
    const ClippedFit a = polyfitClipped(temp.array() - aRef, accel, order);

    const auto terms = [order](const Eigen::MatrixXd &coef) {
        Eigen::Matrix3d m = Eigen::Matrix3d::Zero();
        for (int k = 0; k < order; ++k) m.col(k) = coef.row(k + 1).transpose();
        return m;
    };
    const auto drift = [&](const Eigen::MatrixXd &coef, double ref) {
        const Eigen::MatrixXd model = vander(temp.array() - ref, order) * coef;
        double worst = 0.0;
        for (int c = 0; c < 3; ++c) worst = std::max(worst, ptp(model.col(c)));
        return worst;
    };
    out.tRef = tRef;
    out.gyroBias = g.coef.row(0).transpose();
    out.gyroTc = terms(g.coef);
    out.accelTc = terms(a.coef);
    out.accelTRef = aRef;
    out.order = order;
    out.tMin = temp.minCoeff();
    out.tMax = temp.maxCoeff();
    out.gyroResidualDps = std::sqrt(g.keptResiduals.array().square().mean());
    out.gyroDriftDps = drift(g.coef, tRef);
    out.accelResidualMg = std::sqrt(a.keptResiduals.array().square().mean()) * 1000.0;
    out.accelDriftMg = drift(a.coef, aRef) * 1000.0;
    out.samples = static_cast<int>(temp.size());
    out.gyroCoef = g.coef;
    out.accelCoef = a.coef;
    return out;
}

} // namespace ins10::calib
