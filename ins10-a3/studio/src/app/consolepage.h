// Raw terminal: a command line with history, the whole exchange, and a filter that
// hides stream records.
#pragma once

#include "device/boardmodel.h"

#include <QWidget>

class QCheckBox;
class QLineEdit;
class QPlainTextEdit;

namespace ins10 {

class ConsolePage : public QWidget {
    Q_OBJECT
public:
    explicit ConsolePage(BoardModel *board, QWidget *parent = nullptr);

protected:
    bool eventFilter(QObject *watched, QEvent *event) override;

private:
    void send();
    void onTraffic(const TrafficBatch &batch);
    void append(const QString &html);

    BoardModel *m_board;
    QPlainTextEdit *m_log;
    QLineEdit *m_input;
    QCheckBox *m_hideStreams;
    QStringList m_history;
    int m_historyPos = -1;
};

} // namespace ins10
