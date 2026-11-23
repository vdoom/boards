// Scrolling plots of the streams: angles, gyro bias, sigmas, temperature, raw gyro and
// accelerometer, field magnitude, barometric altitude. Pause, window length, cursor,
// CSV export of what is visible.
#pragma once

#include "core/streamseries.h"
#include "device/boardmodel.h"

#include <QWidget>

class QCheckBox;
class QComboBox;
class QPushButton;
class QTimer;

namespace ins10 {

class PlotWidget;
class TimeView;

class PlotsPage : public QWidget {
    Q_OBJECT
public:
    explicit PlotsPage(BoardModel *board, QWidget *parent = nullptr);
    ~PlotsPage() override;

    StreamSeries &series() { return m_series; }
    TimeView *timeView() const { return m_view; }
    // Writes the visible part of every shown plot's data; one file per stream.
    bool exportVisible(const QString &basePath, QString *error = nullptr) const;

protected:
    void showEvent(QShowEvent *event) override;
    void hideEvent(QHideEvent *event) override;

private:
    struct Plot {
        QString key;
        PlotWidget *widget;
        QCheckBox *toggle;
        enum class Needs { Att, Raw, Mag, Baro } needs;
        const char *title; // untranslated (QT_TR_NOOP)
        const char *unit;
    };
    void addPlot(const QString &key, const char *title, const char *unit, Plot::Needs needs,
                 const TimeSeriesGroup *group, const QList<int> &channels, bool on);
    void updateStreams();
    void exportDialog();

    BoardModel *m_board;
    StreamSeries m_series;
    TimeView *m_view;
    QList<Plot> m_plots;
    QComboBox *m_window;
    QComboBox *m_rawDiv;
    QPushButton *m_pause;
    QTimer *m_repaint;
    QWidget *m_plotArea;
};

} // namespace ins10
