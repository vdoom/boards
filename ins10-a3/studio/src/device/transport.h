// A byte channel to the board: the serial port or the simulated board.
#pragma once

#include <QByteArray>
#include <QObject>
#include <QString>

namespace ins10 {

class Transport : public QObject {
    Q_OBJECT
public:
    using QObject::QObject;
    ~Transport() override = default;

    virtual QString portName() const = 0;
    // Opens the channel and asserts DTR (without it the firmware prints nothing).
    virtual bool open(QString *error) = 0;
    virtual void close() = 0;
    virtual bool isOpen() const = 0;
    virtual bool write(const QByteArray &data) = 0;
    // Discards everything received and not yet delivered.
    virtual void clearInput() = 0;

signals:
    void dataReceived(const QByteArray &data);
    // The channel went away underneath us: cable pulled, board rebooted.
    void lost(const QString &reason);
};

// Creates the transport for a port name; keeps long-lived ones (the simulated board
// keeps its "flash" across reconnections).
class TransportFactory {
public:
    virtual ~TransportFactory() = default;
    // Called in the session's thread. The factory keeps ownership.
    virtual Transport *acquire(const QString &portName) = 0;
    virtual void release(Transport *transport) = 0;
    // Whether the port is present now (for reconnecting after 'reboot').
    virtual bool isAvailable(const QString &portName) = 0;
    // Destroys what the factory keeps, in the session's thread, before it ends.
    virtual void shutdown() {}
};

} // namespace ins10
