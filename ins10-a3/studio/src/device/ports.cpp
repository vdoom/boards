#include "device/ports.h"

#include <QSerialPortInfo>

#include <algorithm>

namespace ins10 {

QList<PortInfo> availablePorts() {
    QList<PortInfo> out;
    for (const QSerialPortInfo &info : QSerialPortInfo::availablePorts()) {
        PortInfo port;
        port.name = info.portName();
        port.isPico = info.hasVendorIdentifier() && info.vendorIdentifier() == kPicoUsbVendorId;
        QString text = info.description();
        if (text.isEmpty()) text = info.manufacturer();
        port.description = text.isEmpty() ? port.name : QStringLiteral("%1 (%2)").arg(port.name, text);
        out.append(port);
    }
    std::stable_sort(out.begin(), out.end(),
                     [](const PortInfo &a, const PortInfo &b) { return a.isPico && !b.isPico; });
    return out;
}

} // namespace ins10
