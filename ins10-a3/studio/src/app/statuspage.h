// 'info' and 'status' as tables, health counters over time, interference levels of
// the accelerometer and magnetometer, and the learned thermal model.
#pragma once

#include "device/boardmodel.h"

#include <QWidget>

class QLabel;
class QPushButton;
class QTableWidget;
class QTreeWidget;
class QTreeWidgetItem;

namespace ins10 {

class PlotWidget;
class TimeSeriesGroup;

class StatusPage : public QWidget {
    Q_OBJECT
public:
    explicit StatusPage(BoardModel *board, QWidget *parent = nullptr);
    ~StatusPage() override;

protected:
    void showEvent(QShowEvent *event) override;
    void hideEvent(QHideEvent *event) override;

private:
    void updateInfo();
    void updateStatus();
    // The status tree from the last 'status' reply.
    void showStatus();
    void updateThermal();
    void resetThermal();
    void restartFilter();
    QTreeWidgetItem *row(const QString &group, const QString &name);

    BoardModel *m_board;
    QTableWidget *m_info;
    QTreeWidget *m_status;
    QTableWidget *m_thermal;
    QLabel *m_thermalSummary;
    QPushButton *m_resetThermal;
    QPushButton *m_restartFilter;
    PlotWidget *m_counterPlot;
    PlotWidget *m_noisePlot;
    std::unique_ptr<TimeSeriesGroup> m_counterSeries;
    std::unique_ptr<TimeSeriesGroup> m_noiseSeries;
    QMap<QString, QTreeWidgetItem *> m_rows;
};

} // namespace ins10
