// Orientation at a glance: the board in 3D, the angles and filter state as tiles, the
// compass tape and the status flags.
#pragma once

#include "device/boardmodel.h"

#include <QWidget>

class QGridLayout;
class QLabel;

namespace ins10 {

class BoardView;
class CompassWidget;

class OverviewPage : public QWidget {
    Q_OBJECT
public:
    explicit OverviewPage(BoardModel *board, QWidget *parent = nullptr);
    BoardView *boardView() const { return m_view; }

private:
    void refresh();
    // title and tip untranslated (QT_TR_NOOP).
    QLabel *addTile(const char *title, const char *tip, int row, int column, int columnSpan = 1);

    BoardModel *m_board;
    BoardView *m_view;
    CompassWidget *m_compass;
    QGridLayout *m_tiles = nullptr;
    QLabel *m_roll, *m_pitch, *m_yaw, *m_heading, *m_tiltSigma, *m_headingSigma, *m_bias, *m_temp;
    QList<QLabel *> m_flagRows;
    QLabel *m_age;
    bool m_dirty = false;
};

} // namespace ins10
