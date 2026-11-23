// One run of a calibration procedure, without any widgets: the logic behind each
// calibration wizard.
//
//   start()      reads the calibration ('cal show') as the snapshot to return to, asks
//                for the streams the procedure needs and records them
//   stop()       gives the streams back
//   compute()    runs the procedure (core/calib/workflow) in a worker thread
//   tryOnBoard() writes the result to the board's RAM: the filter uses it at once
//   saveToFlash() writes it (if not tried yet) and saves to flash
//   revert()     sends the snapshot back (RAM)
//
// A run can also compute from a recording loaded from a file, without a board.
// Destroying a run gives the streams back and, if a trial was not saved, restores the
// snapshot.
#pragma once

#include "core/calib/workflow.h"
#include "core/recording.h"
#include "device/boardmodel.h"

#include <QFutureWatcher>
#include <QObject>
#include <QStringList>

#include <atomic>
#include <memory>

class QTimer;

namespace ins10 {

class CalibrationRun : public QObject {
    Q_OBJECT
public:
    enum class State { Idle, Starting, Recording, Recorded, Computing, Computed, Writing };
    Q_ENUM(State)

    CalibrationRun(BoardModel *board, calib::Procedure procedure, QObject *parent = nullptr);
    ~CalibrationRun() override;

    calib::Procedure procedure() const { return m_procedure; }
    State state() const { return m_state; }
    // 'stream raw' divisor and magnetometer stream the procedure records with.
    static StreamRequest streamsFor(calib::Procedure procedure);

    void start();
    void stop();
    bool loadRecording(const QString &path, QString *error = nullptr);
    bool saveRecording(const QString &path, QString *error = nullptr) const;
    void compute();
    void cancelCompute();
    void tryOnBoard();
    void saveToFlash();
    void revert();
    void setSouthern(bool southern) { m_southern = southern; }

    const Recording &recording() const { return m_recording; }
    const QStringList &lines() const { return m_lines; }
    const CalibrationSet &snapshot() const { return m_snapshot; }
    bool hasSnapshot() const { return m_haveSnapshot; }
    bool isOffline() const { return m_offline; }
    const calib::WorkflowResult &result() const { return m_result; }
    bool hasResult() const { return m_state == State::Computed || m_state == State::Writing; }
    bool tried() const { return m_tried; }
    bool saved() const { return m_saved; }
    double recordedSeconds() const;
    // Largest body rate over the last quarter second, from the raw gyro (dps).
    double recentRateDps() const { return m_recentRate; }

    calib::ImuProgress imuProgress() const { return m_imuProgress; }
    calib::MagProgress magProgress() const { return m_magProgress; }
    calib::ThermalProgress thermalProgress() const { return m_thermalProgress; }

signals:
    void stateChanged(ins10::CalibrationRun::State state);
    void progress();                       // new data and progress figures
    void computed(bool ok);
    void written(bool ok, const QString &message);
    void problem(const QString &message);

private:
    void setState(State state);
    void onBatch(const StreamBatch &batch);
    void onConnection(ConnectionState state);
    void updateProgress();
    void releaseStreams();
    QString owner() const;

    BoardModel *m_board;
    calib::Procedure m_procedure;
    State m_state = State::Idle;
    Recording m_recording;
    QStringList m_lines;
    CalibrationSet m_snapshot;
    bool m_haveSnapshot = false;
    bool m_offline = false;
    bool m_tried = false, m_saved = false;
    bool m_southern = false;
    bool m_streamsRequested = false;
    calib::WorkflowResult m_result;
    std::shared_ptr<std::atomic<bool>> m_cancel;
    QFutureWatcher<calib::WorkflowResult> *m_watcher;
    QTimer *m_progressTimer;
    bool m_progressDirty = false;
    double m_recentRate = 0.0;
    qint64 m_rateWindowStart = 0;
    calib::ImuProgress m_imuProgress;
    calib::MagProgress m_magProgress;
    calib::ThermalProgress m_thermalProgress;
};

} // namespace ins10
