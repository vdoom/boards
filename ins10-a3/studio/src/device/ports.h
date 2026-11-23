// Serial port discovery. Raspberry Pi Pico boards enumerate with USB vendor 0x2E8A.
#pragma once

#include <QList>
#include <QString>

namespace ins10 {

inline constexpr quint16 kPicoUsbVendorId = 0x2E8A;

struct PortInfo {
    QString name;         // what QSerialPort::setPortName expects ("COM5", "ttyACM0")
    QString description;  // human-readable
    bool isPico = false;
};

// All serial ports, Picos first.
QList<PortInfo> availablePorts();

// The fake board used by --demo and the tests appears under this name.
inline const QString kDemoPortName = QStringLiteral("demo");

} // namespace ins10
