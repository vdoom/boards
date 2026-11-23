#include "app/calwizards.h"

#include "app/plotwidget.h"
#include "app/sphereview.h"
#include "app/uiutil.h"
#include "app/xyplot.h"

#include <QCheckBox>
#include <QGroupBox>
#include <QHeaderView>
#include <QLabel>
#include <QSplitter>
#include <QTreeWidget>
#include <QVBoxLayout>

#include <cmath>

namespace ins10 {

namespace {
QLabel *bigLabel(QWidget *parent) {
    auto *label = new QLabel(QStringLiteral("-"), parent);
    QFont f = label->font();
    f.setPointSizeF(f.pointSizeF() * 1.4);
    f.setBold(true);
    label->setFont(f);
    return label;
}

// A label whose text follows the interface language.
QLabel *caption(QWidget *parent, std::function<QString()> text) {
    auto *label = new QLabel(parent);
    ui::translated(label, [label, text] { label->setText(text()); });
    return label;
}

QGroupBox *groupBox(QWidget *parent, std::function<QString()> title) {
    auto *box = new QGroupBox(parent);
    ui::translated(box, [box, title] { box->setTitle(title()); });
    return box;
}

QString colored(CheckLevel level, const QString &text) {
    return QStringLiteral("<span style='color:%1'>%2</span>").arg(ui::levelColor(level).name(), text);
}

QString row(const QString &name, const QString &value) {
    return QStringLiteral("<tr><td>%1&nbsp;&nbsp;</td><td>%2</td></tr>").arg(name, value);
}

// At most `limit` rows of `points`, evenly spread.
Eigen::MatrixX3d thin(const Eigen::MatrixX3d &points, Eigen::Index limit) {
    if (points.rows() <= limit) return points;
    const double step = static_cast<double>(points.rows()) / limit;
    Eigen::MatrixX3d out(limit, 3);
    for (Eigen::Index i = 0; i < limit; ++i) out.row(i) = points.row(static_cast<Eigen::Index>(i * step));
    return out;
}

Eigen::MatrixX3d rawMag(const Recording &rec) {
    Eigen::MatrixX3d m(rec.magRows(), 3);
    for (Eigen::Index i = 0; i < m.rows(); ++i) m.row(i) = rec.mag()[static_cast<size_t>(i)].transpose();
    return m;
}
} // namespace

// ---- Magnetometer ------------------------------------------------------------------

MagWizard::MagWizard(BoardModel *board, QWidget *parent) : WizardWidget(board, calib::Procedure::Mag, parent) {
    build();
    m_southern = new QCheckBox(this);
    ui::translated(m_southern, [this] {
        m_southern->setText(tr("Southern hemisphere (the field points up)"));
        m_southern->setToolTip(tr("Which way the earth's field points decides whether the magnetometer is aligned to the "
                                  "IMU with a rotation or with a mirror image of its axes. Leave it unticked north of the "
                                  "magnetic equator, which includes all of Europe and North America."));
    });
    extraControls()->addWidget(m_southern);
    connect(m_southern, &QCheckBox::toggled, m_run, &CalibrationRun::setSouthern);
}

QString MagWizard::instructions() const {
    return tr("<b>Magnetometer: hard and soft iron, and alignment to the IMU axes.</b> Do this with the board mounted as it "
              "will be used (its carrier, wires and nearby parts are part of what is calibrated), away from steel "
              "furniture, speakers, monitors and power cables. Calibrate the accelerometer first. Turn the board slowly "
              "through every orientation: several full turns about each of its three axes, including upside down; about "
              "60-90 s, slower than one turn in 6 s, without shaking. Aim for 80 % or more of the cells green.");
}

QWidget *MagWizard::createLiveView() {
    m_view = new SphereView(this);
    ui::translated(m_view, [this] {
        m_view->setToolTip(tr("Raw magnetometer samples around the centre of their range, scaled to the unit sphere. "
                              "Cells turn green once a sample points through them."));
    });
    return m_view;
}

QWidget *MagWizard::createFigures() {
    auto *box = groupBox(this, [] { return tr("Progress"); });
    auto *layout = new QVBoxLayout(box);
    m_samples = new QLabel(box);
    m_coverage = bigLabel(box);
    m_speed = new QLabel(box);
    m_speed->setWordWrap(true);
    layout->addWidget(caption(box, [] { return tr("Directions covered (aim for 80 % or more)"); }));
    layout->addWidget(m_coverage);
    layout->addWidget(m_samples);
    layout->addWidget(m_speed);
    reset();
    return box;
}

void MagWizard::reset() {
    m_coverage->setText(QStringLiteral("-"));
    m_samples->setText(tr("no samples yet"));
    m_speed->clear();
    m_view->setCells(std::vector<bool>(72, false));
    m_view->clearCloud(0);
    m_view->clearCloud(1);
}

void MagWizard::updateLive() {
    const calib::MagProgress p = m_run->magProgress();
    m_samples->setText(tr("%1 magnetometer samples").arg(p.samples));
    if (p.samples < 20 || !(p.radius > 0)) return;
    const double pct = p.coverage * 100.0;
    m_coverage->setText(colored(pct >= 80 ? CheckLevel::Ok : pct >= 60 ? CheckLevel::Warning : CheckLevel::Fail,
                                QStringLiteral("%1 %").arg(ui::number(pct, 0))));
    m_view->setCells(p.cells);
    const Eigen::MatrixX3d raw = thin(rawMag(m_run->recording()), 6000);
    m_view->setCloud(0, (raw.rowwise() - p.center.transpose()) / p.radius, QColor(70, 120, 210), 3.0f);
    const double rate = m_run->recentRateDps();
    if (m_run->state() == CalibrationRun::State::Recording) {
        m_speed->setText(rate > 60.0 ? colored(CheckLevel::Fail, tr("<b>Slower!</b> Turning at %1 deg/s; keep below 60.").arg(ui::number(rate, 0)))
                                     : tr("Turning at %1 deg/s").arg(ui::number(rate, 0)));
    }
}

void MagWizard::showResult() {
    const calib::WorkflowResult &r = m_run->result();
    if (!r.ok || !r.mag) return;
    const calib::MagCalibration &c = *r.mag;
    const Eigen::MatrixX3d raw = thin(r.magInput, 6000);
    const Eigen::Vector3d center = 0.5 * (raw.colwise().maxCoeff() + raw.colwise().minCoeff()).transpose();
    const double radius = (raw.rowwise() - center.transpose()).rowwise().norm().mean();
    m_view->setCloud(0, (raw.rowwise() - center.transpose()) / radius * 1.25, QColor(150, 150, 150), 2.0f);
    const Eigen::MatrixX3d calibrated = ((raw.rowwise() - c.bias.transpose()) * c.matrix.transpose()) / c.fieldUt;
    m_view->setCloud(1, calibrated, QColor(40, 160, 80), 3.0f);
    // Cells from every sample, as the coverage figure.
    std::vector<bool> cells;
    calib::sphereCoverage((r.magInput.rowwise() - c.bias.transpose()) * c.matrix.transpose(), 6, 12, &cells);
    m_view->setCells(cells);
    m_coverage->setText(QStringLiteral("%1 %").arg(ui::number(c.coverage * 100.0, 0)));
}

QString MagWizard::resultHtml() const {
    const calib::WorkflowResult &r = m_run->result();
    if (!r.mag) return {};
    const calib::MagCalibration &c = *r.mag;
    QString html = QStringLiteral("<p>%1</p><table>").arg(tr("Grey: the samples before calibration; green: after, on the unit sphere."));
    html += row(tr("Samples"), QString::number(c.samples));
    html += row(tr("Directions covered"), QStringLiteral("%1 %").arg(ui::number(c.coverage * 100, 0)));
    html += row(tr("Hard iron, uT"), htmlNumbers(c.bias, 2));
    // Location-neutral hints: the board is used far from where the firmware was written.
    html += row(tr("Field magnitude"),
                tr("%1 uT (22-67 on the earth; the value for your place is in the NOAA WMM calculator)").arg(ui::number(c.fieldUt, 2)));
    html += row(tr("Spread after calibration"), tr("%1 % (below 1 % is good)").arg(ui::number(c.normResidualPct, 2)));
    if (c.aligned) {
        html += row(tr("Aligned to the IMU axes"),
                    c.mirrored ? tr("yes, with a mirror image (the magnetometer's axes are mirrored)") : tr("yes"));
        html += row(tr("Dip angle"), tr("%1 deg, consistent to %2 deg (grows with latitude: about +65 at 50 deg N, +75 at "
                                        "60 deg N; negative south of the magnetic equator)")
                                         .arg(ui::number(*c.dipDeg, 1), ui::number(c.dipResidualDeg.value_or(NAN), 2)));
    } else {
        html += row(tr("Aligned to the IMU axes"), colored(CheckLevel::Fail, tr("no")));
    }
    html += row(tr("Matrix (soft iron and alignment)"), htmlMatrix(c.matrix, 5));
    return html + QStringLiteral("</table>");
}

// ---- Accelerometer and gyro -----------------------------------------------------------

ImuWizard::ImuWizard(BoardModel *board, QWidget *parent) : WizardWidget(board, calib::Procedure::Imu, parent) { build(); }

QString ImuWizard::instructions() const {
    return tr("<b>Accelerometer and gyro: multi-position calibration.</b> Put the board down in 12 or more clearly "
              "different orientations: each face and edge down, and a few tilted ones (a small box to lean it against "
              "helps). Leave it completely still for 3-4 s in each, then turn it to the next in one smooth motion; the "
              "turns calibrate the gyro, so vary the direction of turning. Rest positions are listed as they are "
              "recognised.");
}

QWidget *ImuWizard::createLiveView() {
    auto *split = new QSplitter(Qt::Horizontal, this);
    m_view = new SphereView(split);
    m_positions = new QTreeWidget(split);
    ui::translated(m_positions, [this] {
        m_view->setToolTip(tr("Direction of gravity in the board's axes for every rest position recognised"));
        m_positions->setHeaderLabels({tr("#"), tr("Duration, s"), tr("Gravity in board axes, g"), tr("Temp, C")});
    });
    m_positions->setRootIsDecorated(false);
    m_positions->header()->setSectionResizeMode(QHeaderView::ResizeToContents);
    split->addWidget(m_view);
    split->addWidget(m_positions);
    split->setStretchFactor(0, 1);
    split->setStretchFactor(1, 1);
    return split;
}

QWidget *ImuWizard::createFigures() {
    auto *box = groupBox(this, [] { return tr("Progress"); });
    auto *layout = new QVBoxLayout(box);
    m_count = bigLabel(box);
    m_orientations = bigLabel(box);
    m_advice = new QLabel(box);
    m_advice->setWordWrap(true);
    layout->addWidget(caption(box, [] { return tr("Rest positions"); }));
    layout->addWidget(m_count);
    layout->addWidget(caption(box, [] { return tr("Distinct orientations"); }));
    layout->addWidget(m_orientations);
    layout->addWidget(m_advice);
    reset();
    return box;
}

void ImuWizard::reset() {
    m_count->setText(QStringLiteral("0"));
    m_orientations->setText(QStringLiteral("0"));
    m_advice->setText(tr("At least 6 orientations are needed, 9 for the cross-axis terms; 12 or more are better."));
    m_positions->clear();
    m_view->setMarkers({}, QColor());
}

void ImuWizard::updateLive() {
    const calib::ImuProgress p = m_run->imuProgress();
    m_count->setText(QString::number(p.positions));
    const CheckLevel level = p.orientations >= 12 ? CheckLevel::Ok : p.orientations >= 6 ? CheckLevel::Warning : CheckLevel::Fail;
    m_orientations->setText(colored(level, QString::number(p.orientations)));
    m_advice->setText(p.orientations < 6   ? tr("Need at least 6 (each face of the board down).")
                      : p.orientations < 9 ? tr("Enough for bias and scale; 9 or more also give the cross-axis terms.")
                      : p.orientations < 12 ? tr("Enough for the full matrix; a few more improve it (12+).")
                                            : tr("Plenty. Stop when you like."));
    std::vector<Eigen::Vector3d> directions;
    m_positions->clear();
    int index = 1;
    for (const calib::StaticInterval &iv : p.intervals) {
        directions.push_back(iv.accel.normalized());
        new QTreeWidgetItem(m_positions, {QString::number(index++), ui::number(iv.duration, 1),
                                          QStringLiteral("%1  %2  %3").arg(ui::number(iv.accel[0], 3), ui::number(iv.accel[1], 3), ui::number(iv.accel[2], 3)),
                                          ui::number(iv.temp, 1)});
    }
    m_view->setMarkers(directions, QColor(230, 150, 30));
}

void ImuWizard::showResult() {
    const calib::WorkflowResult &r = m_run->result();
    if (!r.ok || !r.imu) return;
    std::vector<Eigen::Vector3d> directions;
    for (const calib::StaticInterval &iv : r.imu->intervals)
        directions.push_back((r.imu->accelMatrix * (iv.accel - r.imu->accelBias)).normalized());
    m_view->setMarkers(directions, QColor(40, 160, 80));
    m_count->setText(QString::number(r.imu->positions));
    m_orientations->setText(QString::number(r.imu->orientations));
}

QString ImuWizard::resultHtml() const {
    const calib::WorkflowResult &r = m_run->result();
    if (!r.imu) return {};
    const calib::ImuCalibration &c = *r.imu;
    QString html = QStringLiteral("<table>");
    html += row(tr("Rest positions, orientations"), QStringLiteral("%1, %2").arg(c.positions).arg(c.orientations));
    html += row(tr("Temperature"), QStringLiteral("%1 C").arg(ui::number(c.tempC, 1)));
    html += row(tr("Accelerometer: |a| error over the positions"),
                tr("%1 mg before, %2 mg after").arg(ui::number(c.accelResidualBeforeMg, 2), ui::number(c.accelResidualMg, 3)));
    html += row(tr("Accelerometer bias, mg"), htmlNumbers(c.accelBias, 2, 1000.0));
    html += row(tr("Accelerometer matrix"), htmlMatrix(c.accelMatrix, 5));
    html += row(tr("Gyro bias, deg/s"), htmlNumbers(c.gyroBias, 4));
    if (c.gyroMatrixFitted != QLatin1String("none")) {
        html += row(tr("Gyro: gravity mismatch after the turns"),
                    tr("%1 deg before, %2 deg after (%3 matrix)")
                        .arg(ui::number(c.gyroResidualBeforeDeg.value_or(NAN), 3), ui::number(c.gyroResidualDeg.value_or(NAN), 3),
                             c.gyroMatrixFitted));
        html += row(tr("Gyro matrix"), htmlMatrix(c.gyroMatrix, 5));
    } else {
        html += row(tr("Gyro matrix"), colored(CheckLevel::Warning, tr("not fitted; the gyro flag is not set")));
    }
    return html + QStringLiteral("</table>");
}

// ---- Temperature -------------------------------------------------------------------

ThermalWizard::ThermalWizard(BoardModel *board, QWidget *parent)
    : WizardWidget(board, calib::Procedure::Thermal, parent),
      m_series(std::make_unique<TimeSeriesGroup>(QStringList{tr("temperature")}, 1 << 15)) {
    build();
}

ThermalWizard::~ThermalWizard() = default;

QString ThermalWizard::instructions() const {
    return tr("<b>Temperature: gyro and accelerometer bias drift.</b> 1. Cool the board: unpowered, in a closed bag, 20 "
              "minutes in a fridge. 2. Put it on a solid surface, connect it and do not touch the board or the table. "
              "3. Start recording and let it warm up for 15-30 minutes; warming it gently with a hair dryer from a "
              "distance widens the range. 15 C or more of range is worth having; less than 3 C is refused.");
}

QWidget *ThermalWizard::createLiveView() {
    auto *split = new QSplitter(Qt::Vertical, this);
    m_timeView = new TimeView(this);
    m_timeView->setWindow(3600.0);
    m_tempPlot = new PlotWidget(m_timeView, split);
    m_tempPlot->setAutoRange(1.0);
    m_scatter = new XyPlot(split);
    ui::translated(m_tempPlot, [this] {
        m_tempPlot->setTitle(tr("Temperature"));
        m_tempPlot->setUnit(tr("C"));
        m_tempPlot->clearTraces();
        m_tempPlot->addTrace(m_series.get(), 0, tr("IMU"), PlotWidget::seriesColor(3));
        m_scatter->setTitles(tr("Gyro reading against temperature"), tr("temperature, C"), tr("deg/s"));
    });
    split->addWidget(m_tempPlot);
    split->addWidget(m_scatter);
    return split;
}

QWidget *ThermalWizard::createFigures() {
    auto *box = groupBox(this, [] { return tr("Progress"); });
    auto *layout = new QVBoxLayout(box);
    m_now = bigLabel(box);
    m_range = new QLabel(box);
    m_range->setWordWrap(true);
    m_motion = new QLabel(box);
    m_motion->setWordWrap(true);
    layout->addWidget(caption(box, [] { return tr("Temperature now"); }));
    layout->addWidget(m_now);
    layout->addWidget(m_range);
    layout->addWidget(m_motion);
    reset();
    return box;
}

void ThermalWizard::reset() {
    m_series->clear();
    m_timeView->reset();
    m_rowsShown = 0;
    m_now->setText(QStringLiteral("-"));
    m_range->setText(tr("Range needed: at least 3 C, better 15 C or more."));
    m_motion->clear();
    m_scatter->setSeries({});
}

void ThermalWizard::updateLive() {
    const Recording &rec = m_run->recording();
    if (rec.imuRows() < m_rowsShown) reset();
    const double odr = rec.scales().odrHz;
    for (qsizetype i = m_rowsShown; i < rec.imuRows(); ++i) {
        const size_t k = static_cast<size_t>(i);
        m_series->append((static_cast<double>(rec.seq()[k]) + 0.5 * rec.n()[k]) / odr, {rec.temp()[k]});
    }
    m_rowsShown = rec.imuRows();
    if (!m_series->isEmpty()) m_timeView->setLatest(m_series->lastTime());
    const calib::ThermalProgress p = m_run->thermalProgress();
    if (p.samples == 0) return;
    m_now->setText(QStringLiteral("%1 C").arg(ui::number(p.now, 2)));
    const double span = p.span();
    const CheckLevel level = span >= 15.0 ? CheckLevel::Ok : span >= 3.0 ? CheckLevel::Warning : CheckLevel::Fail;
    m_range->setText(tr("%1 ... %2 C, span %3")
                         .arg(ui::number(p.min, 1), ui::number(p.max, 1), colored(level, QStringLiteral("%1 C").arg(ui::number(span, 1)))));
    m_motion->setText(p.gyroMotionDps > 2.0 ? colored(CheckLevel::Warning, tr("The board moved: those samples will be rejected. "
                                                                                 "Do not touch the board or the table."))
                                            : tr("Still."));
    updateScatter(false);
}

void ThermalWizard::updateScatter(bool withFit) {
    const Recording &rec = m_run->recording();
    QList<XyPlot::Series> series;
    const char *axes[3] = {"x", "y", "z"};
    const qsizetype n = rec.imuRows();
    const qsizetype step = std::max<qsizetype>(1, n / 1500);
    for (int a = 0; a < 3; ++a) {
        XyPlot::Series s;
        s.label = tr("gyro %1").arg(QLatin1String(axes[a]));
        s.color = PlotWidget::seriesColor(a);
        for (qsizetype i = 0; i < n; i += step) {
            const size_t k = static_cast<size_t>(i);
            s.points.append(QPointF(rec.temp()[k], rec.gyro()[k][a]));
        }
        series.append(s);
    }
    const calib::WorkflowResult &r = m_run->result();
    if (withFit && r.ok && r.thermal) {
        const calib::ThermalCalibration &c = *r.thermal;
        for (int a = 0; a < 3; ++a) {
            XyPlot::Series curve;
            curve.color = PlotWidget::seriesColor(a).darker(150);
            curve.line = true;
            for (int i = 0; i <= 60; ++i) {
                const double t = c.tMin + (c.tMax - c.tMin) * i / 60.0;
                const double u = t - c.tRef;
                double v = 0.0;
                for (Eigen::Index k = c.gyroCoef.rows() - 1; k >= 0; --k) v = v * u + c.gyroCoef(k, a);
                curve.points.append(QPointF(t, v));
            }
            series.append(curve);
        }
    }
    m_scatter->setSeries(series);
}

void ThermalWizard::showResult() {
    updateLive();
    updateScatter(true);
}

QString ThermalWizard::resultHtml() const {
    const calib::WorkflowResult &r = m_run->result();
    if (!r.thermal) return {};
    const calib::ThermalCalibration &c = *r.thermal;
    QString html = QStringLiteral("<table>");
    html += row(tr("Samples, range"), tr("%1, %2 ... %3 C").arg(c.samples).arg(ui::number(c.tMin, 1), ui::number(c.tMax, 1)));
    html += row(tr("Polynomial order, reference"), tr("%1, %2 C").arg(c.order).arg(ui::number(c.tRef, 0)));
    html += row(tr("Gyro bias drift over the range"),
                tr("%1 deg/s, residual after %2 deg/s").arg(ui::number(c.gyroDriftDps, 3), ui::number(c.gyroResidualDps, 4)));
    html += row(tr("Gyro bias at the reference, deg/s"), htmlNumbers(c.gyroBias, 4));
    html += row(tr("Gyro slope, deg/s per C"), htmlNumbers(c.gyroTc.col(0), 4));
    html += row(tr("Accelerometer drift over the range"),
                tr("%1 mg, residual after %2 mg").arg(ui::number(c.accelDriftMg, 2), ui::number(c.accelResidualMg, 3)));
    html += row(tr("Accelerometer slope, mg per C"), htmlNumbers(c.accelTc.col(0), 3, 1000.0));
    return html + QStringLiteral("</table>");
}

} // namespace ins10
