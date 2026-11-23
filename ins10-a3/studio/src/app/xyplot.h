// A plain x-y plot (scatter and curves) drawn with QPainter, for the temperature
// wizard: bias against temperature with the fitted polynomials on top.
#pragma once

#include <QColor>
#include <QList>
#include <QPointF>
#include <QWidget>

namespace ins10 {

class XyPlot : public QWidget {
    Q_OBJECT
public:
    explicit XyPlot(QWidget *parent = nullptr);

    struct Series {
        QString label;
        QColor color;
        QList<QPointF> points;
        bool line = false;   // a curve instead of dots
    };
    void setSeries(const QList<Series> &series);
    void setTitles(const QString &title, const QString &xLabel, const QString &yLabel);
    QStringList titles() const { return {m_title, m_xLabel, m_yLabel}; }
    QSize sizeHint() const override { return {420, 260}; }

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    QList<Series> m_series;
    QString m_title, m_xLabel, m_yLabel;
};

} // namespace ins10
