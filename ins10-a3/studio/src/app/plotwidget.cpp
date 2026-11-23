#include "app/plotwidget.h"

#include "app/uiutil.h"

#include <QFontMetrics>
#include <QLocale>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QWheelEvent>

#include <cmath>

namespace ins10 {

// ---- TimeView ----------------------------------------------------------------------

void TimeView::setWindow(double seconds) {
    m_window = std::clamp(seconds, 0.5, 3600.0);
    emit changed();
}

void TimeView::setPaused(bool paused) {
    if (paused == m_paused) return;
    m_paused = paused;
    m_end = m_latest;
    emit changed();
}

void TimeView::setLatest(double t) {
    if (!std::isfinite(t)) return;
    m_latest = t;
    if (!m_paused) emit changed();
}

void TimeView::pan(double seconds) {
    if (!m_paused) {
        m_paused = true;
        m_end = m_latest;
    }
    m_end = std::min(m_end + seconds, m_latest);
    emit changed();
}

void TimeView::setCursor(std::optional<double> t) {
    m_cursor = t;
    emit changed();
}

void TimeView::reset() {
    m_latest = m_end = 0.0;
    m_cursor.reset();
    emit changed();
}

// ---- PlotWidget --------------------------------------------------------------------

namespace {
// 1, 2 or 5 times a power of ten, the smallest not below `raw`.
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

PlotWidget::PlotWidget(TimeView *view, QWidget *parent) : QWidget(parent), m_view(view) {
    setMouseTracking(true);
    setAttribute(Qt::WA_OpaquePaintEvent);
    connect(view, &TimeView::changed, this, qOverload<>(&QWidget::update));
}

QColor PlotWidget::seriesColor(int index) {
    static const QColor colors[] = {QColor(0xd6, 0x27, 0x28), QColor(0x2c, 0xa0, 0x2c), QColor(0x1f, 0x77, 0xb4),
                                    QColor(0xff, 0x7f, 0x0e), QColor(0x94, 0x67, 0xbd), QColor(0x8c, 0x56, 0x4b),
                                    QColor(0xe3, 0x77, 0xc2), QColor(0x17, 0xbe, 0xcf)};
    return colors[static_cast<size_t>(index) % std::size(colors)];
}

void PlotWidget::addTrace(const TimeSeriesGroup *group, int channel, const QString &label, const QColor &color) {
    m_traces.append({group, channel, label, color});
    update();
}

void PlotWidget::relabelTraces() {
    for (PlotTrace &trace : m_traces) {
        if (trace.group && trace.channel < trace.group->channels()) trace.label = trace.group->names()[trace.channel];
    }
    update();
}

void PlotWidget::setFixedRange(double lo, double hi) {
    m_fixed = true;
    m_lo = lo;
    m_hi = hi;
    update();
}

void PlotWidget::setAutoRange(double minSpan) {
    m_fixed = false;
    m_minSpan = minSpan;
    update();
}

QRectF PlotWidget::plotArea() const {
    const QFontMetrics fm(font());
    const double left = fm.horizontalAdvance(QStringLiteral("-00000.00")) + 8;
    return QRectF(left, fm.height() + 6, std::max(10.0, width() - left - 8.0), std::max(10.0, height() - 2.0 * fm.height() - 12.0));
}

double PlotWidget::timeAt(double x) const {
    const QRectF area = plotArea();
    return m_view->start() + (x - area.left()) / area.width() * m_view->window();
}

void PlotWidget::paintEvent(QPaintEvent *) {
    QPainter p(this);
    const QPalette pal = palette();
    p.fillRect(rect(), pal.color(QPalette::Base));
    const QRectF area = plotArea();
    const QFontMetrics fm(font());
    const double t0 = m_view->start(), t1 = m_view->end();
    const int columns = std::max(1, static_cast<int>(area.width()));

    // Vertical range.
    double lo = m_lo, hi = m_hi;
    if (!m_fixed) {
        bool any = false;
        for (const PlotTrace &trace : m_traces) {
            if (!trace.group) continue;
            if (auto r = trace.group->range(trace.channel, t0, t1)) {
                lo = any ? std::min(lo, static_cast<double>(r->first)) : r->first;
                hi = any ? std::max(hi, static_cast<double>(r->second)) : r->second;
                any = true;
            }
        }
        if (!any) {
            lo = -1.0;
            hi = 1.0;
        }
        const double span = std::max(hi - lo, std::max(m_minSpan, 1e-9));
        const double mid = 0.5 * (hi + lo);
        lo = mid - 0.55 * span;
        hi = mid + 0.55 * span;
    }
    double ystep = niceStep((hi - lo) / std::max(2.0, area.height() / 40.0));
    if (m_decimals >= 0) ystep = std::max(ystep, std::pow(10.0, -m_decimals)); // no repeated labels
    const int ydec = m_decimals >= 0 ? m_decimals : decimalsFor(ystep);
    auto yOf = [&](double v) { return area.bottom() - (v - lo) / (hi - lo) * area.height(); };

    // Grid and labels.
    // Solid hairlines: dashed and wide pens go through QPainter's slow stroker.
    QColor gridColor = pal.color(QPalette::Mid);
    gridColor.setAlpha(90);
    const QPen gridPen(gridColor, 0);
    p.setFont(font());
    for (double v = std::ceil(lo / ystep) * ystep; v <= hi; v += ystep) {
        const double y = yOf(v);
        p.setPen(gridPen);
        p.drawLine(QPointF(area.left(), y), QPointF(area.right(), y));
        p.setPen(pal.color(QPalette::Text));
        const QString label = QLocale().toString(std::abs(v) < ystep * 1e-6 ? 0.0 : v, 'f', ydec);
        p.drawText(QRectF(0, y - fm.height() / 2.0, area.left() - 4, fm.height()), Qt::AlignRight | Qt::AlignVCenter, label);
    }
    const double tstep = niceStep(m_view->window() / std::max(2.0, area.width() / 90.0));
    const double latest = m_view->latest();
    for (double rel = std::ceil((t0 - latest) / tstep) * tstep; rel <= t1 - latest + 1e-9; rel += tstep) {
        const double x = area.left() + (latest + rel - t0) / m_view->window() * area.width();
        p.setPen(gridPen);
        p.drawLine(QPointF(x, area.top()), QPointF(x, area.bottom()));
        p.setPen(pal.color(QPalette::Text));
        const QString label = QStringLiteral("%1 s").arg(QLocale().toString(std::abs(rel) < 1e-9 ? 0.0 : rel, 'f', decimalsFor(tstep)));
        p.drawText(QRectF(x - 40, area.bottom() + 2, 80, fm.height()), Qt::AlignHCenter | Qt::AlignTop, label);
    }
    p.setPen(pal.color(QPalette::Mid));
    p.drawRect(area);

    // Traces.
    p.save();
    p.setClipRect(area.adjusted(0, -1, 0, 1));
    p.setRenderHint(QPainter::Antialiasing, false);
    for (const PlotTrace &trace : m_traces) {
        if (!trace.group) continue;
        trace.group->envelope(trace.channel, t0, t1, columns, m_bins);
        p.setPen(QPen(trace.color, 0));
        QPolygonF line;
        int emptyRun = 0;
        for (int c = 0; c < columns; ++c) {
            const TimeSeriesGroup::Bin &b = m_bins[static_cast<size_t>(c)];
            if (b.count == 0) {
                ++emptyRun;
                continue;
            }
            // A long hole in the data is not bridged.
            if (emptyRun > 0 && emptyRun * m_view->window() / columns > 1.0 && !line.isEmpty()) {
                p.drawPolyline(line);
                line.clear();
            }
            emptyRun = 0;
            const double x = area.left() + c + 0.5;
            line << QPointF(x, yOf(b.first));
            if (b.count > 1) line << QPointF(x, yOf(b.min)) << QPointF(x, yOf(b.max)) << QPointF(x, yOf(b.last));
        }
        if (line.size() == 1) p.drawPoint(line.front());
        else if (!line.isEmpty()) p.drawPolyline(line);
    }
    p.restore();

    // Title and legend.
    p.setPen(pal.color(QPalette::Text));
    QString title = m_title;
    if (!m_unit.isEmpty()) title += QStringLiteral(", ") + m_unit;
    QFont bold = font();
    bold.setBold(true);
    p.setFont(bold);
    // The legend must fit: when title and legend together are wider than the plot, the
    // title is shortened first (the legend is what tells the traces apart).
    double legendWidth = 0.0;
    for (const PlotTrace &trace : m_traces) legendWidth += 18 + fm.horizontalAdvance(trace.label) + 14;
    const QFontMetrics boldMetrics(bold);
    const double room = area.right() - area.left() - legendWidth - 16;
    if (boldMetrics.horizontalAdvance(title) > room)
        title = boldMetrics.elidedText(title, Qt::ElideRight, static_cast<int>(std::max(room, 40.0)));
    p.drawText(QPointF(area.left(), fm.ascent() + 2), title);
    double x = area.left() + boldMetrics.horizontalAdvance(title) + 16;
    p.setFont(font());
    for (const PlotTrace &trace : m_traces) {
        // Still too wide (a very narrow plot): the last labels are shortened, not cut.
        const double left = area.right() - x - 18;
        if (left < 20) break;
        const QString label = fm.elidedText(trace.label, Qt::ElideRight, static_cast<int>(left));
        p.fillRect(QRectF(x, fm.ascent() / 2.0, 14, 4), trace.color);
        x += 18;
        p.drawText(QPointF(x, fm.ascent() + 2), label);
        x += fm.horizontalAdvance(label) + 14;
    }
    if (m_view->paused()) {
        const QString text = tr("paused");
        p.setPen(ui::levelColor(CheckLevel::Warning));
        p.drawText(QPointF(area.right() - fm.horizontalAdvance(text), fm.ascent() + 2), text);
    }

    // Cursor.
    if (auto cursor = m_view->cursor(); cursor && *cursor >= t0 && *cursor <= t1) {
        const double cx = area.left() + (*cursor - t0) / m_view->window() * area.width();
        p.setPen(QPen(pal.color(QPalette::Text), 0, Qt::DashLine));
        p.drawLine(QPointF(cx, area.top()), QPointF(cx, area.bottom()));
        QStringList rows;
        rows.append(QStringLiteral("%1 s").arg(QLocale().toString(*cursor - latest, 'f', 3)));
        QList<QColor> colors{pal.color(QPalette::Text)};
        for (const PlotTrace &trace : m_traces) {
            if (!trace.group) continue;
            const auto i = trace.group->nearest(*cursor);
            if (!i) continue;
            const double v = trace.group->value(trace.channel, *i);
            rows.append(QStringLiteral("%1: %2").arg(trace.label, QLocale().toString(v, 'f', std::max(ydec + 1, 2))));
            colors.append(trace.color);
        }
        double w = 0;
        for (const QString &r : rows) w = std::max(w, static_cast<double>(fm.horizontalAdvance(r)));
        const double h = rows.size() * fm.height() + 6;
        double bx = cx + 8;
        if (bx + w + 10 > area.right()) bx = cx - w - 18;
        const QRectF box(bx, area.top() + 4, w + 10, h);
        QColor background = pal.color(QPalette::ToolTipBase);
        background.setAlpha(230);
        p.fillRect(box, background);
        p.setPen(pal.color(QPalette::Mid));
        p.drawRect(box);
        for (qsizetype i = 0; i < rows.size(); ++i) {
            p.setPen(i == 0 ? pal.color(QPalette::ToolTipText) : colors[i]);
            p.drawText(QPointF(box.left() + 5, box.top() + 3 + fm.ascent() + i * fm.height()), rows[i]);
        }
    }
}

void PlotWidget::mouseMoveEvent(QMouseEvent *event) {
    const QPointF pos = event->position();
    if (m_dragX) {
        const double dt = -(pos.x() - *m_dragX) / plotArea().width() * m_view->window();
        m_view->pan(m_dragEnd + dt - m_view->end());
    }
    if (plotArea().contains(pos)) m_view->setCursor(timeAt(pos.x()));
    else m_view->setCursor(std::nullopt);
}

void PlotWidget::mousePressEvent(QMouseEvent *event) {
    if (event->button() == Qt::LeftButton) {
        m_dragX = event->position().x();
        m_dragEnd = m_view->end();
    }
}

void PlotWidget::mouseReleaseEvent(QMouseEvent *) { m_dragX.reset(); }

void PlotWidget::leaveEvent(QEvent *) { m_view->setCursor(std::nullopt); }

void PlotWidget::wheelEvent(QWheelEvent *event) {
    const double steps = event->angleDelta().y() / 120.0;
    m_view->setWindow(m_view->window() * std::pow(0.8, steps));
    event->accept();
}

} // namespace ins10
