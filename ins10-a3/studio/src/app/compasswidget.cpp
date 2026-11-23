#include "app/compasswidget.h"

#include "app/uiutil.h"

#include <QLocale>
#include <QPainter>
#include <QPainterPath>

#include <cmath>

namespace ins10 {

CompassWidget::CompassWidget(QWidget *parent) : QWidget(parent) {
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    ui::translated(this, [this] {
        setToolTip(tr("Compass heading of the board's X axis. Grey: not referenced to the magnetic field "
                      "(magnetometer not calibrated or disabled), so it is relative to the heading at power-up."));
    });
}

void CompassWidget::setHeading(std::optional<double> headingDeg, bool referenced) {
    m_heading = headingDeg;
    m_referenced = referenced;
    update();
}

void CompassWidget::paintEvent(QPaintEvent *) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    const QPalette pal = palette();
    const QRectF r = rect().adjusted(1, 1, -1, -1);
    p.fillRect(r, pal.color(QPalette::Base));
    p.setPen(pal.color(QPalette::Mid));
    p.drawRect(r);
    if (!m_heading) {
        p.setPen(pal.color(QPalette::PlaceholderText));
        p.drawText(r, Qt::AlignCenter, tr("no heading"));
        return;
    }
    const double heading = *m_heading;
    const double pxPerDeg = r.width() / 120.0; // 120 degrees visible
    const double cx = r.center().x();
    const QColor ink = m_referenced ? pal.color(QPalette::Text) : pal.color(QPalette::PlaceholderText);
    QFont small = font();
    small.setPointSizeF(small.pointSizeF() * 0.9);
    p.setFont(small);
    const int first = static_cast<int>(std::floor((heading - 62.0) / 5.0)) * 5;
    for (int d = first; d <= heading + 62.0; d += 5) {
        const double x = cx + (d - heading) * pxPerDeg;
        const int deg = ((d % 360) + 360) % 360;
        const bool major = deg % 30 == 0;
        p.setPen(QPen(ink, major ? 1.6 : 1.0));
        p.drawLine(QPointF(x, r.top()), QPointF(x, r.top() + (major ? 14 : 8)));
        if (major) {
            QString label = QString::number(deg);
            if (deg == 0) label = tr("N");
            else if (deg == 90) label = tr("E");
            else if (deg == 180) label = tr("S");
            else if (deg == 270) label = tr("W");
            const bool cardinal = deg % 90 == 0;
            QFont f = small;
            f.setBold(cardinal);
            p.setFont(f);
            p.setPen(cardinal && deg == 0 ? QColor(205, 50, 45) : ink);
            p.drawText(QRectF(x - 30, r.top() + 15, 60, 18), Qt::AlignHCenter | Qt::AlignTop, label);
            p.setFont(small);
        }
    }
    // Lubber line and the value.
    QPainterPath marker;
    marker.moveTo(cx, r.top() + 2);
    marker.lineTo(cx - 7, r.top() - 0);
    marker.lineTo(cx + 7, r.top() - 0);
    marker.closeSubpath();
    const QColor accent = m_referenced ? QColor(205, 50, 45) : pal.color(QPalette::PlaceholderText);
    p.setPen(QPen(accent, 2.0));
    p.drawLine(QPointF(cx, r.top()), QPointF(cx, r.bottom() - 18));
    QFont big = font();
    big.setBold(true);
    p.setFont(big);
    const QString text = QStringLiteral("%1°").arg(QLocale().toString(heading, 'f', 1));
    const QRectF box(cx - 40, r.bottom() - 19, 80, 18);
    p.fillRect(box, pal.color(QPalette::Base));
    p.setPen(accent);
    p.drawText(box, Qt::AlignCenter, text);
}

} // namespace ins10
