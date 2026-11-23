// A horizontal compass tape: heading of the board's X axis, clockwise from north.
#pragma once

#include <QWidget>

#include <optional>

namespace ins10 {

class CompassWidget : public QWidget {
    Q_OBJECT
public:
    explicit CompassWidget(QWidget *parent = nullptr);
    void setHeading(std::optional<double> headingDeg, bool referenced);
    QSize sizeHint() const override { return {600, 64}; }
    QSize minimumSizeHint() const override { return {200, 56}; }

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    std::optional<double> m_heading;
    bool m_referenced = false;
};

} // namespace ins10
