#include "device/calibrationrun.h"

#include "core/logfile.h"

#include <QCoreApplication>
#include <QTimer>
#include <QtConcurrent/QtConcurrentRun>

#include <cmath>

namespace ins10 {

namespace {
struct Text {
    Q_DECLARE_TR_FUNCTIONS(ins10::CalibrationRun)
};
constexpr int kWizardPriority = 10;
} // namespace

StreamRequest CalibrationRun::streamsFor(calib::Procedure procedure) {
    StreamRequest r;
    r.priority = kWizardPriority;
    switch (procedure) {
    case calib::Procedure::Mag: r.rawDiv = 8; r.mag = true; break;          // as cmd_magcal
    case calib::Procedure::Imu: r.rawDiv = 2; break;                        // as cmd_imucal
    case calib::Procedure::Thermal: r.rawDiv = 256; break;                  // as cmd_thermalcal
    }
    return r;
}

CalibrationRun::CalibrationRun(BoardModel *board, calib::Procedure procedure, QObject *parent)
    : QObject(parent), m_board(board), m_procedure(procedure), m_watcher(new QFutureWatcher<calib::WorkflowResult>(this)),
      m_progressTimer(new QTimer(this)) {
    m_progressTimer->setInterval(500);
    connect(m_progressTimer, &QTimer::timeout, this, [this] {
        if (m_progressDirty) updateProgress();
    });
    connect(m_watcher, &QFutureWatcher<calib::WorkflowResult>::finished, this, [this] {
        m_result = m_watcher->result();
        setState(State::Computed);
        emit computed(m_result.ok);
    });
    connect(board->device(), &DeviceController::streamBatch, this, &CalibrationRun::onBatch);
    connect(board, &BoardModel::connectionChanged, this, &CalibrationRun::onConnection);
}

CalibrationRun::~CalibrationRun() {
    releaseStreams();
    if (m_cancel) m_cancel->store(true);
    m_watcher->waitForFinished();
    // A trial that was neither saved nor cancelled must not stay behind on the board.
    if (m_tried && !m_saved && m_haveSnapshot && m_board->isConnected()) m_board->restoreCalibration(m_snapshot, {});
}

QString CalibrationRun::owner() const {
    switch (m_procedure) {
    case calib::Procedure::Mag: return QStringLiteral("wizard-mag");
    case calib::Procedure::Imu: return QStringLiteral("wizard-imu");
    case calib::Procedure::Thermal: return QStringLiteral("wizard-thermal");
    }
    return QStringLiteral("wizard");
}

void CalibrationRun::setState(State state) {
    if (state == m_state) return;
    m_state = state;
    emit stateChanged(state);
}

void CalibrationRun::releaseStreams() {
    if (!m_streamsRequested) return;
    m_streamsRequested = false;
    m_board->device()->releaseStreams(owner());
}

double CalibrationRun::recordedSeconds() const {
    if (m_recording.imuRows() == 0) return 0.0;
    const auto &seq = m_recording.seq();
    const auto &n = m_recording.n();
    return static_cast<double>(seq.back() + n.back() - seq.front()) / m_recording.scales().odrHz;
}

void CalibrationRun::start() {
    if (!m_board->isConnected()) {
        emit problem(Text::tr("Connect a board first, or load a recording from a file."));
        return;
    }
    if (m_state == State::Recording || m_state == State::Starting || m_state == State::Computing) return;
    if (m_tried && !m_saved) revert();
    m_recording.clear();
    m_lines.clear();
    m_result = calib::WorkflowResult();
    m_offline = false;
    m_tried = m_saved = false;
    m_recentRate = 0.0;
    m_imuProgress = {};
    m_magProgress = {};
    m_thermalProgress = {};
    setState(State::Starting);
    // The stream commands are queued before 'cal show': when its reply arrives the
    // board is streaming with our settings, and nothing older gets recorded.
    m_streamsRequested = true;
    m_board->device()->setStreamRequest(owner(), streamsFor(m_procedure));
    m_board->readCalibration([this](bool ok, const QString &error) {
        if (m_state != State::Starting) return;
        if (!ok) {
            releaseStreams();
            setState(State::Idle);
            emit problem(Text::tr("Could not read the calibration: %1").arg(error));
            return;
        }
        m_snapshot = m_board->calibration();
        m_haveSnapshot = true;
        // The log starts like `ins10.py log`: the 'info' lines, then (for replay and
        // offline recomputation) the calibration in force, then the stream.
        for (const QString &line : m_board->info().lines) {
            m_lines.append(line);
            m_recording.feed(line);
        }
        for (const QString &line : calShowLines(m_snapshot)) {
            m_lines.append(line);
            m_recording.feed(line);
        }
        setState(State::Recording);
        m_progressTimer->start();
        emit progress();
    });
}

void CalibrationRun::stop() {
    if (m_state == State::Starting) {
        releaseStreams();
        setState(State::Idle);
        return;
    }
    if (m_state != State::Recording) return;
    releaseStreams();
    m_progressTimer->stop();
    updateProgress();
    setState(State::Recorded);
}

void CalibrationRun::onBatch(const StreamBatch &batch) {
    if (m_state != State::Recording) return;
    const double window = 0.25 * m_recording.scales().odrHz; // samples in a quarter second
    for (const ParsedLine &line : batch) {
        if (line.kind != LineKind::Raw && line.kind != LineKind::Mag) continue;
        m_lines.append(line.text);
        m_recording.feed(line);
        if (line.kind == LineKind::Raw) {
            const double rate = m_recording.gyro().back().norm();
            const qint64 seq = m_recording.seq().back();
            if (seq - m_rateWindowStart > window || seq < m_rateWindowStart) {
                m_rateWindowStart = seq;
                m_recentRate = rate;
            } else {
                m_recentRate = std::max(m_recentRate, rate);
            }
        }
    }
    m_progressDirty = true;
}

void CalibrationRun::onConnection(ConnectionState state) {
    if (state == ConnectionState::Connected || state == ConnectionState::Closing) return;
    if (m_state == State::Recording || m_state == State::Starting) {
        m_streamsRequested = false; // the session has forgotten them with the connection
        m_progressTimer->stop();
        updateProgress();
        setState(m_recording.imuRows() > 0 ? State::Recorded : State::Idle);
        emit problem(Text::tr("The connection ended during the recording; what was recorded so far is kept."));
    }
}

void CalibrationRun::updateProgress() {
    m_progressDirty = false;
    switch (m_procedure) {
    case calib::Procedure::Imu: m_imuProgress = calib::imuProgress(m_recording); break;
    case calib::Procedure::Mag: m_magProgress = calib::magProgress(m_recording); break;
    case calib::Procedure::Thermal: m_thermalProgress = calib::thermalProgress(m_recording); break;
    }
    emit progress();
}

bool CalibrationRun::loadRecording(const QString &path, QString *error) {
    if (m_state == State::Recording || m_state == State::Starting || m_state == State::Computing) {
        if (error) *error = Text::tr("a recording is in progress");
        return false;
    }
    const auto lines = readLogFile(path, error);
    if (!lines) return false;
    Recording rec;
    for (const QString &line : *lines) rec.feed(line);
    if (rec.imuRows() == 0 && rec.magRows() == 0) {
        if (error) *error = Text::tr("no $RAW or $MAG records in the file");
        return false;
    }
    m_recording = rec;
    m_lines = *lines;
    m_offline = true;
    m_tried = m_saved = false;
    // The calibration in force when the log was made, if the log carries it.
    m_snapshot = rec.calibration();
    m_haveSnapshot = false;
    m_result = calib::WorkflowResult();
    updateProgress();
    setState(State::Recorded);
    return true;
}

bool CalibrationRun::saveRecording(const QString &path, QString *error) const {
    if (m_lines.isEmpty()) {
        if (error) *error = Text::tr("nothing recorded");
        return false;
    }
    return writeLogFile(path, m_lines, error);
}

void CalibrationRun::compute() {
    if (m_state != State::Recorded && m_state != State::Computed) return;
    m_cancel = std::make_shared<std::atomic<bool>>(false);
    const Recording rec = m_recording;
    // Offline, the calibration recorded in the log (if any) stands for the board's.
    const CalibrationSet current = m_snapshot;
    const calib::Procedure procedure = m_procedure;
    calib::MagOptions magOptions;
    magOptions.deviceConnected = !m_offline || !current.isEmpty();
    magOptions.southern = m_southern;
    auto cancel = m_cancel;
    setState(State::Computing);
    m_watcher->setFuture(QtConcurrent::run([rec, current, procedure, magOptions, cancel] {
        switch (procedure) {
        case calib::Procedure::Imu: return calib::runImuCal(rec, current, cancel.get());
        case calib::Procedure::Mag: return calib::runMagCal(rec, current, magOptions, cancel.get());
        case calib::Procedure::Thermal: break;
        }
        return calib::runThermalCal(rec, current, cancel.get());
    }));
}

void CalibrationRun::cancelCompute() {
    if (m_cancel) m_cancel->store(true);
}

void CalibrationRun::tryOnBoard() {
    if (!hasResult() || !m_result.writable) return;
    if (!m_board->isConnected()) {
        emit written(false, Text::tr("not connected"));
        return;
    }
    const auto write = [this] {
        setState(State::Writing);
        m_board->writeCalibration(m_result.fields, m_result.setFlags, false, [this](bool ok, const QString &error) {
            m_tried = ok || m_tried;
            setState(State::Computed);
            emit written(ok, ok ? Text::tr("Written to the board's RAM; the filter uses it now.") : error);
        });
    };
    if (m_haveSnapshot) {
        write();
        return;
    }
    // From a file: what is on the board now is what "cancel" returns to.
    m_board->readCalibration([this, write](bool ok, const QString &error) {
        if (!ok) {
            emit written(false, error);
            return;
        }
        m_snapshot = m_board->calibration();
        m_haveSnapshot = true;
        write();
    });
}

void CalibrationRun::saveToFlash() {
    if (!hasResult() || !m_result.writable) return;
    if (!m_board->isConnected()) {
        emit written(false, Text::tr("not connected"));
        return;
    }
    setState(State::Writing);
    const auto done = [this](bool ok, const QString &error) {
        if (ok) m_saved = m_tried = true;
        setState(State::Computed);
        emit written(ok, ok ? Text::tr("Written and saved to flash.") : error);
    };
    if (m_tried) m_board->saveCalibration(done);
    else m_board->writeCalibration(m_result.fields, m_result.setFlags, true, done);
}

void CalibrationRun::revert() {
    if (!m_haveSnapshot || !m_board->isConnected()) return;
    const State before = m_state;
    setState(State::Writing);
    m_board->restoreCalibration(m_snapshot, [this, before](bool ok, const QString &error) {
        if (ok) m_tried = false;
        setState(before == State::Writing ? State::Computed : before);
        emit written(ok, ok ? Text::tr("The calibration from before the run is back in the board's RAM.") : error);
    });
}

} // namespace ins10
