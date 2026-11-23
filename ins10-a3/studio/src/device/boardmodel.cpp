#include "device/boardmodel.h"

#include <QTimer>

#include <memory>

namespace ins10 {

BoardModel::BoardModel(DeviceController *device, QObject *parent)
    : QObject(parent), m_device(device), m_pollTimer(new QTimer(this)) {
    m_clock.start();
    m_pollTimer->setInterval(1000);
    connect(m_pollTimer, &QTimer::timeout, this, &BoardModel::poll);
    connect(device, &DeviceController::stateChanged, this, &BoardModel::onState);
    connect(device, &DeviceController::identified, this, [this](const InfoReport &info) {
        m_info = info;
        emit infoChanged();
    });
    connect(device, &DeviceController::streamBatch, this, &BoardModel::onBatch);
}

double BoardModel::clock() const { return static_cast<double>(m_clock.elapsed()) * 1e-3; }

void BoardModel::onState(ConnectionState state, const QString &detail) {
    if (state == ConnectionState::Connected) {
        m_history.clear();
        m_status = StatusReport();
        m_thermal = ThermalReport();
        // The calibration as found is what "discard changes" goes back to.
        readCalibration([this](bool ok, const QString &) {
            if (ok) setBaseline(m_calibration);
        });
        refreshThermal();
        if (!m_statusConsumers.isEmpty()) m_pollTimer->start();
    } else if (state != ConnectionState::Closing) {
        m_pollTimer->stop();
        m_statusPending = false;
    }
    if (state == ConnectionState::Disconnected || state == ConnectionState::Lost) {
        m_lastAtt.reset();
    }
    emit connectionChanged(state, detail);
}

void BoardModel::onBatch(const StreamBatch &batch) {
    for (auto it = batch.crbegin(); it != batch.crend(); ++it) {
        if (it->kind == LineKind::Att) {
            m_lastAtt = it->as<AttRecord>();
            emit attitude(*m_lastAtt);
            break;
        }
    }
}

void BoardModel::wantStatus(const QObject *consumer, bool on) {
    if (on) m_statusConsumers.insert(consumer);
    else m_statusConsumers.remove(consumer);
    if (!m_statusConsumers.isEmpty() && isConnected()) {
        if (!m_pollTimer->isActive()) {
            m_pollTimer->start();
            poll();
        }
    } else {
        m_pollTimer->stop();
    }
}

void BoardModel::poll() {
    if (!isConnected() || m_statusPending) return;
    refreshStatus();
}

void BoardModel::refreshStatus() {
    if (!isConnected()) return;
    m_statusPending = true;
    m_device->command(QStringLiteral("status"), [this](const CommandResult &r) {
        m_statusPending = false;
        if (!r.ok) return;
        const StatusReport s = parseStatus(r.lines);
        if (!s.any()) return;
        m_status = s;
        m_history.add(clock(), s);
        emit statusUpdated();
    });
}

void BoardModel::refreshInfo(std::function<void(bool)> done) {
    m_device->command(QStringLiteral("info"), [this, done](const CommandResult &r) {
        if (r.ok) {
            const InfoReport info = parseInfo(r.lines);
            if (info.isAhrs()) {
                m_info = info;
                emit infoChanged();
            }
        }
        if (done) done(r.ok);
    });
}

void BoardModel::refreshThermal(std::function<void(bool)> done) {
    m_device->command(QStringLiteral("thermal show"), [this, done](const CommandResult &r) {
        if (r.ok) {
            m_thermal = parseThermal(r.lines);
            emit thermalUpdated();
        }
        if (done) done(r.ok);
    });
}

void BoardModel::readCalibration(std::function<void(bool, const QString &)> done) {
    m_device->command(QStringLiteral("cal show"), [this, done](const CommandResult &r) {
        if (r.ok) {
            const CalibrationSet set = parseCalShow(r.lines.join(u'\n'));
            if (!set.isEmpty()) {
                m_calibration = set;
                emit calibrationChanged();
            }
        }
        if (done) done(r.ok, r.error);
    });
}

void BoardModel::writeCalibration(const CalibrationSet &fields, quint32 setFlags, bool save,
                                  std::function<void(bool, const QString &)> done) {
    QStringList commands;
    for (const CalFieldDef &f : calibrationFields()) {
        const QString name = QLatin1String(f.name);
        if (name == QLatin1String("flags") || !fields.contains(name)) continue;
        commands.append(calSetCommand(name, fields.value(name)));
    }
    if (fields.contains(QStringLiteral("flags"))) commands.append(calSetCommand(QStringLiteral("flags"), fields.value(QStringLiteral("flags"))));
    auto finish = [this, save, done](bool ok, const QString &error) {
        if (!ok) {
            readCalibration();
            if (done) done(false, error);
            return;
        }
        auto afterSave = [this, save, done](bool saved, const QString &why) {
            readCalibration([this, save, saved, why, done](bool, const QString &) {
                if (saved && save) setBaseline(m_calibration);
                refreshInfo();
                if (done) done(saved, why);
            });
        };
        if (save) saveCalibration(afterSave);
        else afterSave(true, QString());
    };
    m_device->commandSequence(commands, [this, setFlags, finish](bool ok, const QString &error, const QList<CommandResult> &) {
        if (!ok || setFlags == 0) {
            finish(ok, error);
            return;
        }
        // The flag word is read back so that bits set by other calibrations survive.
        m_device->command(QStringLiteral("cal show"), [this, setFlags, finish](const CommandResult &r) {
            if (!r.ok) {
                finish(false, r.error);
                return;
            }
            const quint32 flags = calFlags(parseCalShow(r.lines.join(u'\n'))) | setFlags;
            m_device->command(QStringLiteral("cal set flags %1").arg(flags),
                              [finish](const CommandResult &set) { finish(set.ok, set.error); });
        });
    });
}

void BoardModel::restoreCalibration(const CalibrationSet &snapshot, std::function<void(bool, const QString &)> done) {
    readCalibration([this, snapshot, done](bool ok, const QString &error) {
        if (!ok) {
            if (done) done(false, error);
            return;
        }
        const QStringList changed = changedCalFields(m_calibration, snapshot);
        CalibrationSet fields;
        for (const QString &name : changed) fields.insert(name, snapshot.value(name));
        if (fields.isEmpty()) {
            if (done) done(true, QString());
            return;
        }
        writeCalibration(fields, 0, false, done);
    });
}

void BoardModel::saveCalibration(std::function<void(bool, const QString &)> done) {
    m_device->command(QStringLiteral("cal save"), [this, done](const CommandResult &r) {
        if (r.ok) refreshInfo();
        if (done) done(r.ok, r.error);
    });
}

} // namespace ins10
