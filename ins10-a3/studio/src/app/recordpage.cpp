#include "app/recordpage.h"

#include "app/uiutil.h"
#ifdef INS10_HAVE_REPLAY
#include "app/replaywidget.h"
#endif

#include <QCheckBox>
#include <QComboBox>
#include <QDateTime>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QSettings>
#include <QStandardPaths>
#include <QTimer>
#include <QVBoxLayout>

namespace ins10 {

namespace {
const QString kOwner = QStringLiteral("recorder");
}

RecordPage::RecordPage(BoardModel *board, QWidget *parent)
    : QWidget(parent), m_board(board), m_path(new QLineEdit(this)), m_raw(new QComboBox(this)),
      m_mag(new QCheckBox(this)), m_baro(new QCheckBox(this)), m_calibration(new QCheckBox(this)),
      m_start(new QPushButton(this)), m_stop(new QPushButton(this)), m_status(new QLabel(this)), m_timer(new QTimer(this)) {
    auto *layout = new QVBoxLayout(this);
    auto *intro = new QLabel(this);
    intro->setWordWrap(true);
    layout->addWidget(intro);

    auto *box = new QGroupBox(this);
    auto *form = new QFormLayout(box);
    auto *pathRow = new QHBoxLayout;
    const QString folder = QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation);
    m_path->setText(QSettings().value(QStringLiteral("record/path"), QDir(folder).filePath(QStringLiteral("ins10.log"))).toString());
    auto *browse = new QPushButton(box);
    pathRow->addWidget(m_path, 1);
    pathRow->addWidget(browse);
    auto *fileLabel = new QLabel(box);
    form->addRow(fileLabel, pathRow);
    m_raw->addItem(QString(), 0);
    for (int n : {1, 2, 4, 8, 16, 64, 256}) m_raw->addItem(QString(), n);
    m_raw->setCurrentIndex(std::clamp(QSettings().value(QStringLiteral("record/raw"), 1).toInt(), 0, m_raw->count() - 1));
    auto *rawLabel = new QLabel(box);
    form->addRow(rawLabel, m_raw);
    m_mag->setChecked(QSettings().value(QStringLiteral("record/mag"), true).toBool());
    m_baro->setChecked(QSettings().value(QStringLiteral("record/baro"), false).toBool());
    m_calibration->setChecked(QSettings().value(QStringLiteral("record/calibration"), true).toBool());
    form->addRow(QString(), m_mag);
    form->addRow(QString(), m_baro);
    form->addRow(QString(), m_calibration);
    layout->addWidget(box);

    auto *buttons = new QHBoxLayout;
    buttons->addWidget(m_start);
    buttons->addWidget(m_stop);
    buttons->addStretch(1);
    layout->addLayout(buttons);
    m_status->setObjectName(QStringLiteral("recordStatus"));
    m_status->setWordWrap(true);
    m_status->setFont(ui::monospaceFont());
    layout->addWidget(m_status);
#ifdef INS10_HAVE_REPLAY
    auto *replayBox = new QGroupBox(this);
    ui::translated(replayBox, [replayBox] { replayBox->setTitle(tr("Replay a log through the filter")); });
    (new QVBoxLayout(replayBox))->addWidget(new ReplayWidget(replayBox));
    layout->addWidget(replayBox, 1);
#else
    layout->addStretch(1);
#endif
    ui::translated(this, [this, intro, box, browse, fileLabel, rawLabel] {
        intro->setText(tr("Everything the board sends is written to a text file: first the 'info' lines, then the "
                          "streams, as 'ins10.py log' writes it. Such a file can be replayed through the filter with the "
                          "firmware's ins10_replay, or used for calibration with 'ins10.py ... --from-log' and with the "
                          "wizards here (Compute from a file)."));
        box->setTitle(tr("Recording"));
        browse->setText(tr("Browse..."));
        fileLabel->setText(tr("File"));
        m_raw->setItemText(0, tr("off"));
        for (int i = 1; i < m_raw->count(); ++i) {
            const int n = m_raw->itemData(i).toInt();
            m_raw->setItemText(i, tr("%1 Hz (raw %2)").arg(512.0 / n, 0, 'g', 4).arg(n));
        }
        m_raw->setToolTip(tr("'stream raw n' while recording. 1 is every IMU sample (about 30 kB/s), which ins10_replay "
                             "needs to run the filter as the board does."));
        rawLabel->setText(tr("Raw IMU ($RAW)"));
        m_mag->setText(tr("Magnetometer ($MAG)"));
        m_baro->setText(tr("Barometer ($BARO)"));
        m_calibration->setText(tr("Write the calibration in force ('cal set' lines) after the header"));
        m_calibration->setToolTip(tr("ins10_replay applies these, so the replay uses the board's calibration; the Python "
                                     "tool ignores them."));
        m_start->setText(tr("Start recording"));
        m_stop->setText(tr("Stop"));
        updateStatus();
    });

    connect(browse, &QPushButton::clicked, this, &RecordPage::browse);
    connect(m_start, &QPushButton::clicked, this, [this] {
        const QString path = m_path->text().trimmed();
        if (QFileInfo::exists(path) &&
            QMessageBox::question(this, tr("Recording"), tr("%1 exists. Replace it?").arg(path)) != QMessageBox::Yes)
            return;
        QString error;
        if (!startRecording(path, &error)) QMessageBox::warning(this, tr("Recording"), tr("Cannot write %1: %2").arg(path, error));
    });
    connect(m_stop, &QPushButton::clicked, this, &RecordPage::stopRecording);
    connect(board->device(), &DeviceController::traffic, this, &RecordPage::onTraffic);
    connect(board, &BoardModel::connectionChanged, this, [this](ConnectionState state) {
        if (state != ConnectionState::Connected && state != ConnectionState::Closing && isRecording()) {
            stopRecording();
            m_status->setText(m_status->text() + u'\n' + tr("Stopped: the connection ended."));
        }
        updateStatus();
    });
    m_timer->setInterval(500);
    connect(m_timer, &QTimer::timeout, this, &RecordPage::updateStatus);
}

RecordPage::~RecordPage() { m_writer.close(); }

void RecordPage::browse() {
    const QString path = QFileDialog::getSaveFileName(this, tr("Record to"), m_path->text(), tr("Logs (*.log)"));
    if (!path.isEmpty()) m_path->setText(path);
}

StreamRequest RecordPage::request() const {
    StreamRequest r;
    r.rawDiv = m_raw->currentData().toInt();
    r.mag = m_mag->isChecked();
    r.baro = m_baro->isChecked();
    r.priority = 5; // below the calibration wizards
    return r;
}

bool RecordPage::startRecording(const QString &path, QString *error) {
    if (!m_board->isConnected()) {
        if (error) *error = tr("not connected");
        return false;
    }
    if (!m_writer.open(path, error)) return false;
    QSettings settings;
    settings.setValue(QStringLiteral("record/path"), path);
    settings.setValue(QStringLiteral("record/raw"), m_raw->currentIndex());
    settings.setValue(QStringLiteral("record/mag"), m_mag->isChecked());
    settings.setValue(QStringLiteral("record/baro"), m_baro->isChecked());
    settings.setValue(QStringLiteral("record/calibration"), m_calibration->isChecked());
    m_lostAtStart = m_board->device()->counters().lostSamples;
    m_damagedAtStart = m_board->device()->counters().damagedLines;
    m_pending.clear();
    // The header comes from a fresh 'info' (and 'cal show'); stream lines arriving
    // meanwhile wait for it.
    m_headerPending = true;
    m_board->device()->setStreamRequest(kOwner, request());
    const bool withCalibration = m_calibration->isChecked();
    m_board->refreshInfo([this, withCalibration](bool) {
        if (!m_writer.isOpen()) return;
        m_writer.writeLines(m_board->info().lines);
        auto finishHeader = [this] {
            m_headerPending = false;
            m_writer.writeLines(m_pending);
            m_pending.clear();
        };
        if (!withCalibration) {
            finishHeader();
            return;
        }
        m_board->readCalibration([this, finishHeader](bool ok, const QString &) {
            if (!m_writer.isOpen()) return;
            if (ok) m_writer.writeLines(calShowLines(m_board->calibration()));
            finishHeader();
        });
    });
    m_elapsed.start();
    m_timer->start();
    updateStatus();
    emit recordingChanged(true);
    return true;
}

void RecordPage::stopRecording() {
    if (!m_writer.isOpen()) return;
    m_board->device()->releaseStreams(kOwner);
    m_writer.writeLines(m_pending);
    m_pending.clear();
    const QString path = m_writer.path();
    const qint64 lines = m_writer.linesWritten();
    QString error;
    const bool ok = m_writer.close(&error);
    m_timer->stop();
    updateStatus();
    m_status->setText(ok ? tr("Saved %1 lines to %2").arg(lines).arg(path) : tr("Writing %1 failed: %2").arg(path, error));
    emit recordingChanged(false);
}

void RecordPage::onTraffic(const TrafficBatch &batch) {
    if (!m_writer.isOpen()) return;
    for (const TrafficLine &line : batch) {
        if (line.outgoing) continue;
        // Replies that belong to the header are written by the header code itself.
        if (m_headerPending) {
            if (line.kind == LineKind::Att || line.kind == LineKind::Raw || line.kind == LineKind::Mag ||
                line.kind == LineKind::Baro)
                m_pending.append(line.text);
            continue;
        }
        m_writer.writeLine(line.text);
    }
}

void RecordPage::updateStatus() {
    const bool connected = m_board->isConnected();
    const bool recording = isRecording();
    m_start->setEnabled(connected && !recording);
    m_stop->setEnabled(recording);
    m_path->setEnabled(!recording);
    m_raw->setEnabled(!recording);
    m_mag->setEnabled(!recording);
    m_baro->setEnabled(!recording);
    m_calibration->setEnabled(!recording);
    if (!recording) {
        // The hint is for a missing board only: it goes away once one is connected, and
        // never replaces the result of the last recording.
        const bool hintShown = !m_status->text().isEmpty() && m_status->text() == m_hint;
        m_hint = tr("Connect a board to record.");
        if (!connected && (m_status->text().isEmpty() || hintShown)) m_status->setText(m_hint);
        else if (connected && hintShown) m_status->clear();
        return;
    }
    const SessionCounters &c = m_board->device()->counters();
    m_status->setText(tr("Recording to %1\n%2 s, %3 lines, %4 kB; lost samples %5, damaged lines %6")
                          .arg(m_writer.path())
                          .arg(m_elapsed.elapsed() / 1000)
                          .arg(m_writer.linesWritten())
                          .arg(m_writer.bytesWritten() / 1024)
                          .arg(c.lostSamples - m_lostAtStart)
                          .arg(c.damagedLines - m_damagedAtStart));
}

} // namespace ins10
