// The first-run checklist (README of the firmware, items 2-5 and 7), evaluated live
// from 'info' and 'status' polled once a second.
#pragma once

#include "device/boardmodel.h"

#include <QWidget>

class QLabel;
class QTreeWidget;

namespace ins10 {

class FirstRunPage : public QWidget {
    Q_OBJECT
public:
    explicit FirstRunPage(BoardModel *board, QWidget *parent = nullptr);

protected:
    void showEvent(QShowEvent *event) override;
    void hideEvent(QHideEvent *event) override;

private:
    void evaluate();
    void copyReport();

    BoardModel *m_board;
    QTreeWidget *m_checks;
    QLabel *m_summary;
    QList<CheckResult> m_results;
};

} // namespace ins10
