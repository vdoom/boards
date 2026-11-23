// Replays a recorded log through the firmware's fusion library (as ins10_replay does)
// and shows what the board's filter would have output; the result can be exported in
// ins10_replay's CSV format.
#pragma once

#include "core/series.h"
#include "replay/replay.h"

#include <QFutureWatcher>
#include <QWidget>

#include <atomic>
#include <memory>

class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QPushButton;

namespace ins10 {

class PlotWidget;
class TimeView;

class ReplayWidget : public QWidget {
    Q_OBJECT
public:
    explicit ReplayWidget(QWidget *parent = nullptr);
    ~ReplayWidget() override;

    // Runs on a file without dialogs; finished() follows.
    void run(const QString &logPath, const QString &calibrationPath = QString());
    const ReplayResult &result() const { return m_result; }

signals:
    void finished(bool ok);

private:
    void onFinished();
    void exportCsv();

    QLineEdit *m_log, *m_calibration;
    QDoubleSpinBox *m_rate;
    QPushButton *m_run, *m_cancel, *m_export;
    QLabel *m_status;
    TimeView *m_view;
    PlotWidget *m_plot;
    std::unique_ptr<TimeSeriesGroup> m_series;
    QFutureWatcher<ReplayResult> *m_watcher;
    std::shared_ptr<std::atomic<bool>> m_cancelFlag;
    ReplayResult m_result;
};

} // namespace ins10
