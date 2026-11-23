#include "device/serial_transport.h"

#include <QSerialPort>

namespace ins10 {

SerialTransport::SerialTransport(const QString &portName, QObject *parent)
    : Transport(parent), m_name(portName), m_port(new QSerialPort(this)) {
    m_port->setPortName(portName);
    connect(m_port, &QSerialPort::readyRead, this, [this] {
        const QByteArray data = m_port->readAll();
        if (!data.isEmpty()) emit dataReceived(data);
    });
    connect(m_port, &QSerialPort::errorOccurred, this, [this](QSerialPort::SerialPortError error) {
        // A USB CDC port that disappears reports a resource error (or, on some
        // systems, a permission or I/O error); the port is unusable after that.
        const bool fatal = error == QSerialPort::ResourceError || error == QSerialPort::PermissionError ||
                           error == QSerialPort::DeviceNotFoundError || error == QSerialPort::ReadError ||
                           error == QSerialPort::WriteError;
        if (!fatal || !m_port->isOpen() || m_lostReported) return;
        m_lostReported = true;
        const QString reason = m_port->errorString();
        m_port->close();
        emit lost(reason);
    });
}

SerialTransport::~SerialTransport() { close(); }

bool SerialTransport::open(QString *error) {
    m_lostReported = false;
    // The baud rate means nothing to USB CDC; DTR tells the firmware that a terminal
    // is attached.
    m_port->setBaudRate(115200);
    if (!m_port->open(QIODevice::ReadWrite)) {
        if (error) *error = m_port->errorString();
        return false;
    }
    m_port->setDataTerminalReady(true);
    return true;
}

void SerialTransport::close() {
    if (m_port->isOpen()) {
        m_port->setDataTerminalReady(false);
        m_port->close();
    }
}

bool SerialTransport::isOpen() const { return m_port->isOpen(); }

bool SerialTransport::write(const QByteArray &data) {
    if (!m_port->isOpen()) return false;
    return m_port->write(data) == data.size();
}

void SerialTransport::clearInput() {
    if (!m_port->isOpen()) return;
    m_port->clear(QSerialPort::Input);
    m_port->readAll();
}

} // namespace ins10
