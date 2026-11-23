// What the pages know about the board: the latest 'info', 'status', 'thermal show',
// attitude record and calibration, with polling of 'status' while a page needs it.
#pragma once

#include "core/calibration_fields.h"
#include "core/health.h"
#include "core/protocol.h"
#include "core/status.h"
#include "device/controller.h"

#include <QElapsedTimer>
#include <QObject>
#include <QSet>

#include <functional>
#include <optional>

class QTimer;

namespace ins10 {

class BoardModel : public QObject {
    Q_OBJECT
public:
    explicit BoardModel(DeviceController *device, QObject *parent = nullptr);

    DeviceController *device() const { return m_device; }
    bool isConnected() const { return m_device->isConnected(); }

    const InfoReport &info() const { return m_info; }
    const StatusReport &status() const { return m_status; }
    const StatusHistory &statusHistory() const { return m_history; }
    const ThermalReport &thermal() const { return m_thermal; }
    const std::optional<AttRecord> &lastAtt() const { return m_lastAtt; }
    double clock() const;   // s since the application started

    // 'status' is polled once a second while at least one consumer wants it.
    void wantStatus(const QObject *consumer, bool on);

    void refreshInfo(std::function<void(bool)> done = {});
    void refreshStatus();
    void refreshThermal(std::function<void(bool)> done = {});

    // Calibration as last read with 'cal show', and the snapshot taken when the
    // connection was made (or after the last save): what "discard changes" returns to.
    const CalibrationSet &calibration() const { return m_calibration; }
    const CalibrationSet &baseline() const { return m_baseline; }
    bool hasCalibration() const { return !m_calibration.isEmpty(); }
    void readCalibration(std::function<void(bool ok, const QString &error)> done = {});
    void setBaseline(const CalibrationSet &set) { m_baseline = set; emit calibrationChanged(); }

    // Writes fields the way device.py does: every field with 'cal set', then the flag
    // word OR-ed with setFlags (read back first), then optionally 'cal save'. The
    // calibration is read back afterwards.
    void writeCalibration(const CalibrationSet &fields, quint32 setFlags, bool save,
                          std::function<void(bool ok, const QString &error)> done);
    // Sends back the fields of `snapshot` that differ from the board's current values
    // (RAM only): undoes a trial calibration.
    void restoreCalibration(const CalibrationSet &snapshot, std::function<void(bool ok, const QString &error)> done);
    // 'cal save' (the session puts the owner's $ATT rate in place around it).
    void saveCalibration(std::function<void(bool ok, const QString &error)> done);

signals:
    void connectionChanged(ins10::ConnectionState state, const QString &detail);
    void infoChanged();
    void statusUpdated();
    void thermalUpdated();
    void attitude(const ins10::AttRecord &record);
    void calibrationChanged();

private:
    void onState(ConnectionState state, const QString &detail);
    void onBatch(const StreamBatch &batch);
    void poll();

    DeviceController *m_device;
    InfoReport m_info;
    StatusReport m_status;
    StatusHistory m_history;
    ThermalReport m_thermal;
    std::optional<AttRecord> m_lastAtt;
    CalibrationSet m_calibration, m_baseline;
    QSet<const QObject *> m_statusConsumers;
    QTimer *m_pollTimer;
    bool m_statusPending = false;
    QElapsedTimer m_clock;
};

} // namespace ins10
