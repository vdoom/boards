// Recording of everything the board sends to a file, in the layout of `ins10.py log`
// ('info' lines first, then the streams). `ins10.py ... --from-log` and the firmware's
// `ins10_replay` read such files.
#pragma once

#include "core/logfile.h"
#include "device/boardmodel.h"

#include <QElapsedTimer>
#include <QWidget>

class QCheckBox;
class QComboBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QTimer;

namespace ins10 {

class RecordPage : public QWidget {
    Q_OBJECT
public:
    explicit RecordPage(BoardModel *board, QWidget *parent = nullptr);
    ~RecordPage() override;

    bool isRecording() const { return m_writer.isOpen(); }
    // Starts writing to `path` (no dialogs; for tests and the button).
    bool startRecording(const QString &path, QString *error = nullptr);
    void stopRecording();
    qint64 linesWritten() const { return m_writer.linesWritten(); }

signals:
    void recordingChanged(bool recording);

private:
    void onTraffic(const TrafficBatch &batch);
    void updateStatus();
    StreamRequest request() const;
    void browse();

    BoardModel *m_board;
    LogWriter m_writer;
    QLineEdit *m_path;
    QComboBox *m_raw;
    QCheckBox *m_mag, *m_baro, *m_calibration;
    QPushButton *m_start, *m_stop;
    QLabel *m_status;
    QString m_hint; // the "connect a board" text, as last shown
    QTimer *m_timer;
    QElapsedTimer m_elapsed;
    quint64 m_lostAtStart = 0, m_damagedAtStart = 0;
    bool m_headerPending = false;
    QStringList m_pending;
};

} // namespace ins10
