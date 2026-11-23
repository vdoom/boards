#include "app/statuspage.h"

#include "app/plotwidget.h"
#include "app/uiutil.h"

#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QSplitter>
#include <QTableWidget>
#include <QTreeWidget>
#include <QVBoxLayout>

#include <limits>

namespace ins10 {

namespace {
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
QString n(double v, int d) { return ui::number(v, d); }
QString v3(const std::array<double, 3> &a, int d) { return QStringLiteral("%1  %2  %3").arg(n(a[0], d), n(a[1], d), n(a[2], d)); }
} // namespace

StatusPage::StatusPage(BoardModel *board, QWidget *parent)
    : QWidget(parent), m_board(board), m_info(new QTableWidget(this)), m_status(new QTreeWidget(this)),
      m_thermal(new QTableWidget(this)), m_thermalSummary(new QLabel(this)),
      m_resetThermal(new QPushButton(this)), m_restartFilter(new QPushButton(this)) {
    auto *view = new TimeView(this);
    view->setWindow(300.0);
    m_counterPlot = new PlotWidget(view, this);
    m_noisePlot = new PlotWidget(view, this);
    // Channel names (and trace labels) are set with the other texts, below.
    m_counterSeries = std::make_unique<TimeSeriesGroup>(QStringList(4), 7200);
    m_noiseSeries = std::make_unique<TimeSeriesGroup>(QStringList(2), 7200);
    for (int c = 0; c < 4; ++c) m_counterPlot->addTrace(m_counterSeries.get(), c, QString(), PlotWidget::seriesColor(c));
    m_counterPlot->setAutoRange(2.0);
    m_counterPlot->setDecimals(0);
    m_noisePlot->addTrace(m_noiseSeries.get(), 0, QString(), PlotWidget::seriesColor(3));
    m_noisePlot->addTrace(m_noiseSeries.get(), 1, QString(), PlotWidget::seriesColor(4));
    m_noisePlot->setAutoRange(1.0);
    connect(board, &BoardModel::connectionChanged, this, [this, view](ConnectionState state) {
        if (state == ConnectionState::Connected) {
            m_counterSeries->clear();
            m_noiseSeries->clear();
            view->reset();
        }
    });
    connect(board, &BoardModel::statusUpdated, this, [view, board] { view->setLatest(board->clock()); });

    m_info->setColumnCount(2);
    m_info->horizontalHeader()->setStretchLastSection(true);
    m_info->verticalHeader()->hide();
    m_info->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_status->setColumnCount(2);
    m_status->setRootIsDecorated(true);
    m_status->setUniformRowHeights(true);
    m_status->header()->setSectionResizeMode(0, QHeaderView::ResizeToContents);

    auto *refresh = new QPushButton(this);
    connect(refresh, &QPushButton::clicked, this, [this] {
        m_board->refreshInfo();
        m_board->refreshStatus();
        m_board->refreshThermal();
    });

    auto *left = new QWidget(this);
    auto *leftLayout = new QVBoxLayout(left);
    leftLayout->setContentsMargins(0, 0, 0, 0);
    auto *infoBox = new QGroupBox(left);
    (new QVBoxLayout(infoBox))->addWidget(m_info);
    auto *statusBox = new QGroupBox(left);
    (new QVBoxLayout(statusBox))->addWidget(m_status);
    leftLayout->addWidget(infoBox, 2);
    leftLayout->addWidget(statusBox, 5);
    auto *buttons = new QHBoxLayout;
    buttons->addWidget(refresh);
    buttons->addWidget(m_restartFilter);
    buttons->addStretch(1);
    leftLayout->addLayout(buttons);

    auto *right = new QWidget(this);
    auto *rightLayout = new QVBoxLayout(right);
    rightLayout->setContentsMargins(0, 0, 0, 0);
    rightLayout->addWidget(m_counterPlot, 1);
    rightLayout->addWidget(m_noisePlot, 1);
    auto *thermalBox = new QGroupBox(right);
    auto *thermalLayout = new QVBoxLayout(thermalBox);
    m_thermal->setColumnCount(4);
    m_thermal->setRowCount(3);
    m_thermal->setVerticalHeaderLabels({QStringLiteral("x"), QStringLiteral("y"), QStringLiteral("z")});
    m_thermal->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_thermal->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    m_thermal->setMaximumHeight(m_thermal->verticalHeader()->defaultSectionSize() * 4 + 6);
    m_thermalSummary->setWordWrap(true);
    thermalLayout->addWidget(m_thermalSummary);
    thermalLayout->addWidget(m_thermal);
    auto *thermalButtons = new QHBoxLayout;
    auto *thermalRefresh = new QPushButton(thermalBox);
    thermalButtons->addWidget(thermalRefresh);
    thermalButtons->addWidget(m_resetThermal);
    thermalButtons->addStretch(1);
    thermalLayout->addLayout(thermalButtons);
    rightLayout->addWidget(thermalBox);
    connect(thermalRefresh, &QPushButton::clicked, this, [this] { m_board->refreshThermal(); });
    connect(m_resetThermal, &QPushButton::clicked, this, &StatusPage::resetThermal);
    connect(m_restartFilter, &QPushButton::clicked, this, &StatusPage::restartFilter);

    auto *splitter = new QSplitter(this);
    splitter->addWidget(left);
    splitter->addWidget(right);
    splitter->setStretchFactor(0, 1);
    splitter->setStretchFactor(1, 1);
    splitter->setSizes({620, 740});
    const int rowHeight = fontMetrics().height() + 6;
    m_info->verticalHeader()->setDefaultSectionSize(rowHeight);
    m_thermal->verticalHeader()->setDefaultSectionSize(rowHeight);
    m_thermal->setMaximumHeight(rowHeight * 4 + 8);
    auto *layout = new QVBoxLayout(this);
    layout->addWidget(splitter);

    connect(board, &BoardModel::infoChanged, this, &StatusPage::updateInfo);
    connect(board, &BoardModel::statusUpdated, this, &StatusPage::updateStatus);
    connect(board, &BoardModel::thermalUpdated, this, &StatusPage::updateThermal);
    connect(board, &BoardModel::connectionChanged, this, [this](ConnectionState state) {
        const bool on = state == ConnectionState::Connected;
        m_resetThermal->setEnabled(on);
        m_restartFilter->setEnabled(on);
    });
    m_resetThermal->setEnabled(false);
    m_restartFilter->setEnabled(false);
    ui::translated(this, [this, refresh, infoBox, statusBox, thermalBox, thermalRefresh] {
        m_resetThermal->setText(tr("Forget the thermal model..."));
        m_restartFilter->setText(tr("Restart the filter..."));
        m_counterSeries->setNames({tr("IMU faults"), tr("mag errors"), tr("filter recoveries"), tr("rejected samples")});
        m_noiseSeries->setNames({tr("accel, mg"), tr("mag, uT")});
        m_counterPlot->setTitle(tr("Health counters since connecting"));
        m_counterPlot->relabelTraces();
        m_noisePlot->setTitle(tr("Interference: accelerometer (mg) and magnetometer (uT)"));
        m_noisePlot->relabelTraces();
        m_info->setHorizontalHeaderLabels({tr("Key"), tr("Value")});
        m_status->setHeaderLabels({tr("Quantity"), tr("Value")});
        m_thermal->setHorizontalHeaderLabels({tr("c0, dps"), tr("c1, dps"), tr("c2, dps"), tr("bias now, dps")});
        refresh->setText(tr("Refresh"));
        infoBox->setTitle(tr("info"));
        statusBox->setTitle(tr("status (polled once a second)"));
        thermalBox->setTitle(tr("Learned thermal model of the gyro bias (thermal show)"));
        thermalRefresh->setText(tr("Read again"));
        // The rows are found by their names: build the tree again in the new language.
        m_status->clear();
        m_rows.clear();
        showStatus();
        updateThermal();
    });
}

StatusPage::~StatusPage() = default;

void StatusPage::showEvent(QShowEvent *event) {
    QWidget::showEvent(event);
    m_board->wantStatus(this, true);
}

void StatusPage::hideEvent(QHideEvent *event) {
    QWidget::hideEvent(event);
    // The history keeps growing only while the page is visible or another page polls.
    m_board->wantStatus(this, false);
}

void StatusPage::updateInfo() {
    const InfoReport &info = m_board->info();
    QList<std::pair<QString, QString>> rows;
    for (const QString &line : info.lines) {
        for (const auto &pair : parseInfoPairs(line)) rows.append(pair);
    }
    m_info->setRowCount(static_cast<int>(rows.size()));
    for (int i = 0; i < rows.size(); ++i) {
        m_info->setItem(i, 0, new QTableWidgetItem(rows[i].first));
        m_info->setItem(i, 1, new QTableWidgetItem(rows[i].second));
    }
    m_info->resizeColumnToContents(0);
}

QTreeWidgetItem *StatusPage::row(const QString &group, const QString &name) {
    const QString key = group + u'/' + name;
    if (auto it = m_rows.find(key); it != m_rows.end()) return *it;
    QTreeWidgetItem *parent = m_rows.value(group);
    if (!parent) {
        parent = new QTreeWidgetItem(m_status, {group});
        parent->setExpanded(true);
        QFont bold = parent->font(0);
        bold.setBold(true);
        parent->setFont(0, bold);
        m_rows.insert(group, parent);
    }
    auto *item = new QTreeWidgetItem(parent, {name});
    item->setFont(1, ui::monospaceFont());
    m_rows.insert(key, item);
    return item;
}

void StatusPage::updateStatus() {
    showStatus();
    const StatusReport &s = m_board->status();
    const double t = m_board->clock();
    const double imuFaults = s.imu.valid ? static_cast<double>(s.imu.gaps + s.imu.overflows + s.imu.bad + s.imu.busErrors) : kNaN;
    m_counterSeries->append(t, {imuFaults, s.magCounters.valid ? static_cast<double>(s.magCounters.errors) : kNaN,
                                s.filter.valid ? static_cast<double>(s.filter.recoveries) : kNaN,
                                s.filter.valid ? static_cast<double>(s.filter.rejected) : kNaN});
    m_noiseSeries->append(t, {s.accel.valid ? s.accel.disturbanceG * 1000.0 : kNaN, s.mag.valid ? s.mag.deviationUt : kNaN});
}

void StatusPage::showStatus() {
    const StatusReport &s = m_board->status();
    auto set = [this](const QString &group, const QString &name, const QString &value) { row(group, name)->setText(1, value); };
    const QString filter = tr("Filter"), sensors = tr("Sensors"), health = tr("Health");
    if (s.flags.valid) set(filter, tr("Flags"), QStringLiteral("0x%1  %2").arg(s.flags.value, 3, 16, QLatin1Char('0')).arg(s.flags.names.join(u' ')));
    if (s.angles.valid) {
        set(filter, tr("Roll, pitch, yaw, deg"), QStringLiteral("%1  %2  %3").arg(n(s.angles.rollDeg, 2), n(s.angles.pitchDeg, 2), n(s.angles.yawDeg, 2)));
        set(filter, tr("Heading, deg"), n(s.angles.headingDeg, 2));
        set(filter, tr("Sigma tilt / heading, deg"), QStringLiteral("%1 / %2").arg(n(s.angles.tiltSigmaDeg, 2), n(s.angles.headingSigmaDeg, 2)));
    }
    if (s.gyroBias.valid) set(filter, tr("Gyro bias, dps (sigma)"), QStringLiteral("%1  (%2)").arg(v3(s.gyroBias.dps, 4), n(s.gyroBias.sigmaDps, 4)));
    if (s.accel.valid) {
        set(sensors, tr("Acceleration, g"), v3(s.accel.g, 4));
        set(sensors, tr("Acceleration norm, g"), n(s.accel.norm, 4));
        set(sensors, tr("Accelerometer disturbance, g"), n(s.accel.disturbanceG, 3));
    }
    if (s.mag.valid) {
        set(sensors, tr("Field, uT"), v3(s.mag.ut, 2));
        set(sensors, tr("Field norm / reference, uT"), QStringLiteral("%1 / %2").arg(n(s.mag.norm, 2), n(s.mag.referenceUt, 2)));
        set(sensors, tr("Dip, deg"), n(s.mag.dipDeg, 1));
        set(sensors, tr("Magnetic deviation, uT"), n(s.mag.deviationUt, 2));
    }
    if (s.temp.valid) set(sensors, tr("Temperature, C (rate C/s)"), QStringLiteral("%1  (%2)").arg(n(s.temp.c, 2), n(s.temp.rateCps, 4)));
    if (s.baro.valid) set(sensors, tr("Barometer: hPa, C, altitude m"), QStringLiteral("%1  %2  %3").arg(n(s.baro.hpa, 3), n(s.baro.tempC, 2), n(s.baro.altitudeM, 1)));
    if (s.imu.valid) {
        set(health, tr("IMU samples"), QString::number(s.imu.samples));
        set(health, tr("IMU gaps / overflows / bad / bus errors"),
            QStringLiteral("%1 / %2 / %3 / %4").arg(s.imu.gaps).arg(s.imu.overflows).arg(s.imu.bad).arg(s.imu.busErrors));
        set(health, tr("IMU restarts"), QString::number(s.imu.restarts));
    }
    if (s.load.valid) set(health, tr("CPU load, largest batch"), tr("%1 %, %2 samples").arg(n(s.load.cpuPercent, 1)).arg(s.load.largestBatch));
    if (s.magCounters.valid) {
        set(health, tr("Mag samples / pairs / errors / restarts"),
            QStringLiteral("%1 / %2 / %3 / %4").arg(s.magCounters.samples).arg(s.magCounters.pairs).arg(s.magCounters.errors).arg(s.magCounters.restarts));
        set(health, tr("Baro samples / errors"), QStringLiteral("%1 / %2").arg(s.magCounters.baroSamples).arg(s.magCounters.baroErrors));
    }
    if (s.filter.valid) {
        set(health, tr("Filter recoveries / rejected / restarts"),
            QStringLiteral("%1 / %2 / %3").arg(s.filter.recoveries).arg(s.filter.rejected).arg(s.filter.restarts));
    }
}

void StatusPage::updateThermal() {
    const ThermalReport &t = m_board->thermal();
    if (!t.valid) {
        m_thermalSummary->setText(tr("No model read yet."));
        return;
    }
    m_thermalSummary->setText(t.rangeValid && t.observations > 0
                                  ? tr("%1 observations at rest, %2 to %3 C. Bias = c0 + c1 u + c2 u^2, u = (T - 25 C) / 10.")
                                        .arg(t.observations).arg(n(t.tMinC, 1), n(t.tMaxC, 1))
                                  : tr("No observations yet: the board learns the model while it lies still."));
    const double temp = m_board->status().temp.valid ? m_board->status().temp.c : 25.0;
    const auto now = t.biasAt(temp);
    for (int a = 0; a < 3; ++a) {
        for (int k = 0; k < 3; ++k) {
            m_thermal->setItem(a, k, new QTableWidgetItem(t.axisValid[static_cast<size_t>(a)] ? n(t.coef[static_cast<size_t>(a)][static_cast<size_t>(k)], 5) : QStringLiteral("-")));
        }
        m_thermal->setItem(a, 3, new QTableWidgetItem(n(now[static_cast<size_t>(a)], 4)));
    }
}

void StatusPage::resetThermal() {
    const auto answer = QMessageBox::question(
        this, tr("Forget the thermal model"),
        tr("The board forgets the learned gyro bias-versus-temperature model, and the filter starts again from "
           "initial alignment (leave the board still for 2 s). The stored copy in flash changes only with the next "
           "save.\n\nForget it?"));
    if (answer != QMessageBox::Yes) return;
    m_board->device()->command(QStringLiteral("thermal reset"), [this](const CommandResult &r) {
        if (!r.ok) QMessageBox::warning(this, tr("Thermal model"), tr("The board refused: %1").arg(r.error));
        m_board->refreshThermal();
    });
}

void StatusPage::restartFilter() {
    const auto answer = QMessageBox::question(this, tr("Restart the filter"),
                                              tr("The filter starts again from initial alignment; calibration and the "
                                                 "thermal model are kept. Leave the board still for 2 s.\n\nRestart?"));
    if (answer != QMessageBox::Yes) return;
    m_board->device()->command(QStringLiteral("reset"));
}

} // namespace ins10
