#include "app/xyplot.h"

#include <QFontMetrics>
#include <QLocale>
#include <QPainter>

#include <cmath>
#include <limits>

namespace ins10 {

namespace {
double niceStep(double raw) {
    if (!(raw > 0.0)) return 1.0;
    const double p = std::pow(10.0, std::floor(std::log10(raw)));
    for (double m : {1.0, 2.0, 5.0, 10.0}) {
        if (m * p >= raw) return m * p;
    }
    return 10.0 * p;
}
int decimalsFor(double step) { return std::max(0, static_cast<int>(-std::floor(std::log10(step) + 1e-9))); }
} // namespace

XyPlot::XyPlot(QWidget *parent) : QWidget(parent) { setMinimumSize(240, 160); }

void XyPlot::setSeries(const QList<Series> &series) {
    m_series = series;
    update();
}

void XyPlot::setTitles(const QString &title, const QString &xLabel, const QString &yLabel) {
    m_title = title;
    m_xLabel = xLabel;
    m_yLabel = yLabel;
    update();
}

void XyPlot::paintEvent(QPaintEvent *) {
    QPainter p(this);
    const QPalette pal = palette();
    p.fillRect(rect(), pal.color(QPalette::Base));
    const QFontMetrics fm(font());
    const QRectF area(fm.horizontalAdvance(QStringLiteral("-0.0000")) + 10, fm.height() * 1.6, width() - fm.horizontalAdvance(QStringLiteral("-0.0000")) - 20,
                      height() - fm.height() * 3.6);
    constexpr double inf = std::numeric_limits<double>::infinity();
    double x0 = inf, x1 = -inf, y0 = inf, y1 = -inf;
    for (const Series &s : m_series) {
        for (const QPointF &pt : s.points) {
            if (!std::isfinite(pt.x()) || !std::isfinite(pt.y())) continue;
            x0 = std::min(x0, pt.x());
            x1 = std::max(x1, pt.x());
            y0 = std::min(y0, pt.y());
            y1 = std::max(y1, pt.y());
        }
    }
    QFont bold = font();
    bold.setBold(true);
    p.setFont(bold);
    p.setPen(pal.color(QPalette::Text));
    p.drawText(QPointF(area.left(), fm.ascent() + 2), m_title);
    p.setFont(font());
    if (!(x1 >= x0)) {
        p.setPen(pal.color(QPalette::PlaceholderText));
        p.drawText(area, Qt::AlignCenter, tr("no data yet"));
        return;
    }
    if (x1 - x0 < 1e-9) { x0 -= 0.5; x1 += 0.5; }
    if (y1 - y0 < 1e-9) { y0 -= 0.5; y1 += 0.5; }
    const double padY = 0.06 * (y1 - y0);
    y0 -= padY;
    y1 += padY;
    auto map = [&](const QPointF &pt) {
        return QPointF(area.left() + (pt.x() - x0) / (x1 - x0) * area.width(), area.bottom() - (pt.y() - y0) / (y1 - y0) * area.height());
    };
    QColor gridColor = pal.color(QPalette::Mid);
    gridColor.setAlpha(90);
    const double xs = niceStep((x1 - x0) / 6.0), ys = niceStep((y1 - y0) / 5.0);
    for (double v = std::ceil(x0 / xs) * xs; v <= x1; v += xs) {
        const double x = map({v, y0}).x();
        p.setPen(QPen(gridColor, 0));
        p.drawLine(QPointF(x, area.top()), QPointF(x, area.bottom()));
        p.setPen(pal.color(QPalette::Text));
        p.drawText(QRectF(x - 40, area.bottom() + 2, 80, fm.height()), Qt::AlignHCenter | Qt::AlignTop,
                   QLocale().toString(v, 'f', decimalsFor(xs)));
    }
    for (double v = std::ceil(y0 / ys) * ys; v <= y1; v += ys) {
        const double y = map({x0, v}).y();
        p.setPen(QPen(gridColor, 0));
        p.drawLine(QPointF(area.left(), y), QPointF(area.right(), y));
        p.setPen(pal.color(QPalette::Text));
        p.drawText(QRectF(0, y - fm.height() / 2.0, area.left() - 4, fm.height()), Qt::AlignRight | Qt::AlignVCenter,
                   QLocale().toString(std::abs(v) < ys * 1e-6 ? 0.0 : v, 'f', decimalsFor(ys)));
    }
    p.setPen(pal.color(QPalette::Mid));
    p.drawRect(area);
    p.setPen(pal.color(QPalette::Text));
    p.drawText(QRectF(area.left(), area.bottom() + fm.height() + 2, area.width(), fm.height()), Qt::AlignHCenter, m_xLabel);

    p.setClipRect(area);
    p.setRenderHint(QPainter::Antialiasing);
    for (const Series &s : m_series) {
        if (s.line) {
            QPolygonF poly;
            for (const QPointF &pt : s.points) poly << map(pt);
            p.setPen(QPen(s.color, 2.0));
            p.drawPolyline(poly);
        } else {
            p.setPen(Qt::NoPen);
            QColor c = s.color;
            c.setAlpha(150);
            p.setBrush(c);
            for (const QPointF &pt : s.points) p.drawEllipse(map(pt), 2.0, 2.0);
        }
    }
    p.setClipping(false);
    // Legend and unit on the title line, above the plot: inside it they covered data.
    const double y = fm.ascent() + 2;
    double x = area.left() + QFontMetrics(bold).horizontalAdvance(m_title) + 16;
    const double unitWidth = m_yLabel.isEmpty() ? 0.0 : fm.horizontalAdvance(m_yLabel) + 12;
    for (const Series &s : m_series) {
        if (s.label.isEmpty()) continue;
        if (x + 16 + fm.horizontalAdvance(s.label) > area.right() - unitWidth) break;
        p.fillRect(QRectF(x, fm.ascent() / 2.0, 12, 4), s.color);
        x += 16;
        p.setPen(pal.color(QPalette::Text));
        p.drawText(QPointF(x, y), s.label);
        x += fm.horizontalAdvance(s.label) + 12;
    }
    if (!m_yLabel.isEmpty()) {
        p.setPen(pal.color(QPalette::Text));
        p.drawText(QPointF(area.right() - fm.horizontalAdvance(m_yLabel), y), m_yLabel);
    }
}

} // namespace ins10
