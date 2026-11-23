#pragma once

#include "device/transport.h"

class QSerialPort;

namespace ins10 {

class SerialTransport : public Transport {
    Q_OBJECT
public:
    explicit SerialTransport(const QString &portName, QObject *parent = nullptr);
    ~SerialTransport() override;

    QString portName() const override { return m_name; }
    bool open(QString *error) override;
    void close() override;
    bool isOpen() const override;
    bool write(const QByteArray &data) override;
    void clearInput() override;

private:
    QString m_name;
    QSerialPort *m_port;
    bool m_lostReported = false;
};

} // namespace ins10
