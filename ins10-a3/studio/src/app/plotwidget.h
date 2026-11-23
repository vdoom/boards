// Scrolling time plots drawn with QPainter.
//
// Each screen column shows the minimum and maximum of the samples that fall into it,
// so a ten-minute window at 200 Hz costs no more to draw than a few seconds. Several
// plots share one TimeView: window length, pause, panning and the cursor move them
// together.
#pragma once

#include "core/series.h"

#include <QColor>
#include <QObject>
#include <QWidget>

#include <optional>
#include <vector>

namespace ins10 {

class TimeView : public QObject {
    Q_OBJECT
public:
    using QObject::QObject;
    double window() const { return m_window; }
    void setWindow(double seconds);
    bool paused() const { return m_paused; }
    void setPaused(bool paused);
    // Newest time of any data; the view follows it while not paused.
    void setLatest(double t);
    double latest() const { return m_latest; }
    double end() const { return m_paused ? m_end : m_latest; }
    double start() const { return end() - m_window; }
    void pan(double seconds);   // pauses
    std::optional<double> cursor() const { return m_cursor; }
    void setCursor(std::optional<double> t);
    void reset();

signals:
    void changed();

private:
    double m_window = 30.0;
    bool m_paused = false;
    double m_latest = 0.0;
    double m_end = 0.0;
    std::optional<double> m_cursor;
};

struct PlotTrace {
    const TimeSeriesGroup *group = nullptr;
    int channel = 0;
    QString label;
    QColor color;
};

class PlotWidget : public QWidget {
    Q_OBJECT
public:
    explicit PlotWidget(TimeView *view, QWidget *parent = nullptr);

    void setTitle(const QString &title) { m_title = title; update(); }
    QString title() const { return m_title; }
    void setUnit(const QString &unit) { m_unit = unit; update(); }
    QString unit() const { return m_unit; }
    void addTrace(const TimeSeriesGroup *group, int channel, const QString &label, const QColor &color);
    void clearTraces() { m_traces.clear(); update(); }
    // Labels the traces with their channels' names again (another interface language).
    void relabelTraces();
    const QList<PlotTrace> &traces() const { return m_traces; }
    // Fixed vertical range, or automatic with at least `minSpan` between the limits.
    void setFixedRange(double lo, double hi);
    void setAutoRange(double minSpan = 0.0);
    void setDecimals(int decimals) { m_decimals = decimals; }

    // The colours used for X, Y, Z and further traces.
    static QColor seriesColor(int index);

    QSize sizeHint() const override { return {600, 180}; }
    QSize minimumSizeHint() const override { return {200, 90}; }

protected:
    void paintEvent(QPaintEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void leaveEvent(QEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;

private:
    QRectF plotArea() const;
    double timeAt(double x) const;

    TimeView *m_view;
    QString m_title, m_unit;
    QList<PlotTrace> m_traces;
    bool m_fixed = false;
    double m_lo = -1.0, m_hi = 1.0, m_minSpan = 0.0;
    int m_decimals = -1;
    std::optional<double> m_dragX;
    double m_dragEnd = 0.0;
    mutable std::vector<TimeSeriesGroup::Bin> m_bins;
};

} // namespace ins10
