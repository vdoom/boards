// Always-visible strip: port selection with Pico auto-detection, connect button,
// firmware identity, IMU clock source, the eleven status flags, and the counters of
// damaged lines and lost samples.
#pragma once

#include "device/boardmodel.h"

#include <QWidget>

class QComboBox;
class QLabel;
class QPushButton;
class QTimer;
class QToolButton;

namespace ins10 {

class ConnectionPanel : public QWidget {
    Q_OBJECT
public:
    explicit ConnectionPanel(BoardModel *board, QWidget *parent = nullptr);

    // Selects the simulated board and connects.
    void connectDemo();
    void refreshPorts();

private:
    void retranslate();
    void toggleConnection();
    void updateState(ConnectionState state, const QString &detail);
    void updateIdentity();
    void updateFlags(quint32 flags, bool valid);
    void updateCounters(const SessionCounters &counters);

    BoardModel *m_board;
    QComboBox *m_ports;
    QToolButton *m_refresh;
    QPushButton *m_connect;
    QLabel *m_state;
    QLabel *m_identity;
    QList<QLabel *> m_flags;
    QLabel *m_damaged;
    QLabel *m_lost;
    QTimer *m_portTimer;
};

} // namespace ins10
