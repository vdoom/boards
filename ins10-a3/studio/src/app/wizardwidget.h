// Common frame of the three calibration wizards: instructions, recording controls,
// the live view, the computation with a busy indicator and cancel, the report, and
// the "try / save to flash / cancel" buttons. The work itself is a CalibrationRun.
#pragma once

#include "device/boardmodel.h"
#include "device/calibrationrun.h"

#include <QWidget>

class QLabel;
class QProgressBar;
class QPushButton;
class QTextBrowser;
class QVBoxLayout;

namespace ins10 {

class WizardWidget : public QWidget {
    Q_OBJECT
public:
    WizardWidget(BoardModel *board, calib::Procedure procedure, QWidget *parent = nullptr);
    CalibrationRun *run() const { return m_run; }
    bool isBusy() const;
    // Another wizard records: this one may not start.
    void setBlocked(bool blocked);

signals:
    void busyChanged(bool busy);

protected:
    // Called once from build(), after the subclass is constructed.
    virtual QString instructions() const = 0;
    virtual QWidget *createLiveView() = 0;
    virtual QWidget *createFigures() = 0;
    virtual void updateLive() = 0;
    virtual void showResult() = 0;
    virtual QString resultHtml() const = 0;
    virtual void reset() {}
    void build();
    QVBoxLayout *extraControls() const { return m_extra; }

    BoardModel *m_board;
    CalibrationRun *m_run;

private:
    void updateControls();
    void onComputed();
    void onWritten(bool ok, const QString &message);
    void confirmSave();
    void loadFile();
    void saveFile();
    QString reportHtml() const;

    QPushButton *m_start = nullptr, *m_stop = nullptr, *m_load = nullptr, *m_save = nullptr;
    QPushButton *m_compute = nullptr, *m_cancelCompute = nullptr;
    QPushButton *m_try = nullptr, *m_flash = nullptr, *m_revert = nullptr;
    QLabel *m_status = nullptr;
    QProgressBar *m_busy = nullptr;
    QTextBrowser *m_report = nullptr;
    QVBoxLayout *m_extra = nullptr;
    bool m_blocked = false;
    QString m_message;
};

// Report helpers.
QString htmlNumbers(const Eigen::Vector3d &v, int decimals, double factor = 1.0);
QString htmlMatrix(const Eigen::Matrix3d &m, int decimals);

} // namespace ins10
