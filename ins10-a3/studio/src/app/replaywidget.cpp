#include "app/replaywidget.h"

#include "app/plotwidget.h"
#include "app/uiutil.h"
#include "core/logfile.h"

#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QVBoxLayout>
#include <QtConcurrent/QtConcurrentRun>

namespace ins10 {

ReplayWidget::ReplayWidget(QWidget *parent)
    : QWidget(parent), m_log(new QLineEdit(this)), m_calibration(new QLineEdit(this)), m_rate(new QDoubleSpinBox(this)),
      m_run(new QPushButton(this)), m_cancel(new QPushButton(this)), m_export(new QPushButton(this)),
      m_status(new QLabel(this)), m_view(new TimeView(this)),
      m_series(std::make_unique<TimeSeriesGroup>(QStringList(4), 1 << 20)), m_watcher(new QFutureWatcher<ReplayResult>(this)) {
    auto *layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    auto *intro = new QLabel(this);
    intro->setWordWrap(true);
    layout->addWidget(intro);
    auto *form = new QFormLayout;
    auto *logRow = new QHBoxLayout;
    auto *browseLog = new QPushButton(this);
    logRow->addWidget(m_log, 1);
    logRow->addWidget(browseLog);
    auto *logLabel = new QLabel(this);
    form->addRow(logLabel, logRow);
    auto *calRow = new QHBoxLayout;
    auto *browseCal = new QPushButton(this);
    calRow->addWidget(m_calibration, 1);
    calRow->addWidget(browseCal);
    auto *calLabel = new QLabel(this);
    form->addRow(calLabel, calRow);
    m_rate->setRange(1.0, 5000.0);
    m_rate->setDecimals(3);
    m_rate->setValue(512.0);
    auto *rateLabel = new QLabel(this);
    form->addRow(rateLabel, m_rate);
    layout->addLayout(form);
    auto *buttons = new QHBoxLayout;
    buttons->addWidget(m_run);
    buttons->addWidget(m_cancel);
    buttons->addWidget(m_export);
    buttons->addWidget(m_status, 1);
    layout->addLayout(buttons);
    m_plot = new PlotWidget(m_view, this);
    for (int c = 0; c < 4; ++c) m_plot->addTrace(m_series.get(), c, QString(), PlotWidget::seriesColor(c));
    m_plot->setMinimumHeight(180);
    ui::translated(this, [this, intro, browseLog, logLabel, browseCal, calLabel, rateLabel] {
        m_run->setText(tr("Replay"));
        m_cancel->setText(tr("Cancel"));
        m_export->setText(tr("Export CSV..."));
        intro->setText(tr("Runs a recorded log through the firmware's own fusion library (lib/ins10, built from the "
                          "firmware sources), exactly as ins10_replay does: what the board's filter would have output on "
                          "that data. Useful with logs recorded at 'raw 1' and with the magnetometer."));
        browseLog->setText(tr("Browse..."));
        logLabel->setText(tr("Log"));
        browseCal->setText(tr("Browse..."));
        m_calibration->setPlaceholderText(tr("optional: a saved 'cal show' listing; 'cal set' lines in the log apply too"));
        calLabel->setText(tr("Calibration"));
        m_rate->setSuffix(tr(" Hz"));
        rateLabel->setText(tr("IMU rate"));
        m_series->setNames({tr("roll"), tr("pitch"), tr("yaw"), tr("heading")});
        m_plot->setTitle(tr("Filter output"));
        m_plot->setUnit(tr("deg"));
        m_plot->relabelTraces();
    });
    layout->addWidget(m_plot, 1);
    m_cancel->setEnabled(false);
    m_export->setEnabled(false);

    connect(browseLog, &QPushButton::clicked, this, [this] {
        const QString path = QFileDialog::getOpenFileName(this, tr("Log to replay"), m_log->text(), tr("Logs (*.log *.txt);;All files (*)"));
        if (!path.isEmpty()) m_log->setText(path);
    });
    connect(browseCal, &QPushButton::clicked, this, [this] {
        const QString path = QFileDialog::getOpenFileName(this, tr("Calibration"), m_calibration->text(), tr("Text (*.txt *.cal);;All files (*)"));
        if (!path.isEmpty()) m_calibration->setText(path);
    });
    connect(m_run, &QPushButton::clicked, this, [this] { run(m_log->text().trimmed(), m_calibration->text().trimmed()); });
    connect(m_cancel, &QPushButton::clicked, this, [this] {
        if (m_cancelFlag) m_cancelFlag->store(true);
    });
    connect(m_export, &QPushButton::clicked, this, &ReplayWidget::exportCsv);
    connect(m_watcher, &QFutureWatcher<ReplayResult>::finished, this, &ReplayWidget::onFinished);
}

ReplayWidget::~ReplayWidget() {
    if (m_cancelFlag) m_cancelFlag->store(true);
    m_watcher->waitForFinished();
}

void ReplayWidget::run(const QString &logPath, const QString &calibrationPath) {
    if (m_watcher->isRunning()) return;
    m_log->setText(logPath);
    m_calibration->setText(calibrationPath);
    QString error;
    const auto lines = readLogFile(logPath, &error);
    if (!lines) {
        m_status->setText(tr("Cannot read %1: %2").arg(logPath, error));
        emit finished(false);
        return;
    }
    ReplayOptions options;
    options.rateHz = m_rate->value();
    if (!calibrationPath.isEmpty()) {
        const auto cal = readLogFile(calibrationPath, &error);
        if (!cal) {
            m_status->setText(tr("Cannot read %1: %2").arg(calibrationPath, error));
            emit finished(false);
            return;
        }
        options.calibration = *cal;
    }
    m_cancelFlag = std::make_shared<std::atomic<bool>>(false);
    auto cancel = m_cancelFlag;
    m_run->setEnabled(false);
    m_cancel->setEnabled(true);
    m_export->setEnabled(false);
    m_status->setText(tr("Replaying %1 lines...").arg(lines->size()));
    m_watcher->setFuture(QtConcurrent::run([lines = *lines, options, cancel]() mutable {
        options.cancel = cancel.get();
        return replayLog(lines, options);
    }));
}

void ReplayWidget::onFinished() {
    m_result = m_watcher->result();
    m_run->setEnabled(true);
    m_cancel->setEnabled(false);
    m_series->clear();
    for (const ReplayRow &r : m_result.rows) m_series->append(r.t, {r.rollDeg, r.pitchDeg, r.yawDeg, r.headingDeg});
    m_view->setPaused(false);
    m_view->setWindow(std::max(1.0, m_result.seconds));
    m_view->setLatest(m_result.seconds);
    m_export->setEnabled(!m_result.rows.empty());
    QString text = m_result.cancelled ? tr("Cancelled. ") : QString();
    text += tr("%1 IMU records, %2 s, %3 samples lost in the log; %4 attitude rows.")
                .arg(m_result.imuRecords)
                .arg(ui::number(m_result.seconds, 1))
                .arg(m_result.lostSamples)
                .arg(m_result.rows.size());
    if (!m_result.ignoredCalibration.isEmpty()) text += u' ' + tr("Ignored calibration lines: %1.").arg(m_result.ignoredCalibration.size());
    m_status->setText(text);
    emit finished(!m_result.cancelled);
}

void ReplayWidget::exportCsv() {
    const QString path = QFileDialog::getSaveFileName(this, tr("Export the replay"), QStringLiteral("attitude.csv"), tr("CSV files (*.csv)"));
    if (path.isEmpty()) return;
    QString error;
    if (!writeLogFile(path, replayCsv(m_result), &error)) QMessageBox::warning(this, tr("Export"), tr("Could not write: %1").arg(error));
}

} // namespace ins10
