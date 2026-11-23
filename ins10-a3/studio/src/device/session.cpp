#include "device/session.h"

#include <QCoreApplication>
#include <QDateTime>
#include <QRegularExpression>
#include <QSettings>
#include <QTimer>

#include <algorithm>
#include <limits>

namespace ins10 {

namespace {

struct Text {
    Q_DECLARE_TR_FUNCTIONS(ins10::DeviceSession)
};

const QRegularExpression &spaces() {
    static const QRegularExpression re(QStringLiteral("[ \t]+"));
    return re;
}

QStringList words(const QString &command) { return command.split(spaces(), Qt::SkipEmptyParts); }

bool isCalSave(const QStringList &w) { return w.size() == 2 && w[0] == u"cal" && w[1] == u"save"; }

// "stream att 25" -> 25; -1 if the command is something else.
int streamValue(const QStringList &w, const char *what) {
    if (w.size() != 3 || w[0] != u"stream" || w[1] != QLatin1String(what)) return -1;
    bool ok = false;
    const int v = w[2].toInt(&ok);
    return ok && v >= 0 ? v : -1;
}

void registerTypes() {
    static const bool done = [] {
        qRegisterMetaType<ins10::ConnectionState>();
        qRegisterMetaType<ins10::CommandOrigin>();
        qRegisterMetaType<ins10::CommandResult>();
        qRegisterMetaType<ins10::StreamRequest>();
        qRegisterMetaType<ins10::SessionCounters>();
        qRegisterMetaType<ins10::InfoReport>();
        qRegisterMetaType<ins10::StreamBatch>();
        qRegisterMetaType<ins10::TrafficBatch>();
        return true;
    }();
    (void)done;
}

} // namespace

QString connectionStateText(ConnectionState state) {
    switch (state) {
    case ConnectionState::Disconnected: return Text::tr("Disconnected");
    case ConnectionState::Opening: return Text::tr("Opening port");
    case ConnectionState::Identifying: return Text::tr("Identifying firmware");
    case ConnectionState::Connected: return Text::tr("Connected");
    case ConnectionState::NotAhrs: return Text::tr("Not the AHRS firmware");
    case ConnectionState::Closing: return Text::tr("Restoring board state");
    case ConnectionState::Rebooting: return Text::tr("Rebooting");
    case ConnectionState::Lost: return Text::tr("Connection lost");
    }
    return {};
}

DeviceSession::DeviceSession(TransportFactory *factory, const SessionConfig &config, QObject *parent)
    : QObject(parent), m_factory(factory), m_config(config), m_commandTimer(new QTimer(this)),
      m_flushTimer(new QTimer(this)), m_reconnectTimer(new QTimer(this)), m_closeTimer(new QTimer(this)) {
    registerTypes();
    m_viewAtt = config.viewAttRateHz;
    m_commandTimer->setSingleShot(true);
    connect(m_commandTimer, &QTimer::timeout, this, [this] {
        if (m_inFlight) {
            finish(false,
                   Text::tr("no reply within %1 s (is this the AHRS firmware?)")
                       .arg(m_inFlight->timeoutMs / 1000.0, 0, 'g', 3),
                   true);
        }
    });
    m_flushTimer->setInterval(config.batchIntervalMs);
    connect(m_flushTimer, &QTimer::timeout, this, &DeviceSession::flush);
    m_reconnectTimer->setInterval(config.reconnectPollMs);
    connect(m_reconnectTimer, &QTimer::timeout, this, &DeviceSession::pollReconnect);
    m_closeTimer->setSingleShot(true);
    connect(m_closeTimer, &QTimer::timeout, this, &DeviceSession::finishClose);
}

DeviceSession::~DeviceSession() { releaseTransport(); }

void DeviceSession::setState(ConnectionState state, const QString &detail) {
    if (state == m_state && detail.isEmpty()) return;
    m_state = state;
    emit stateChanged(state, detail);
}

// ---- Connecting --------------------------------------------------------------------

void DeviceSession::connectTo(const QString &portName) {
    if (m_transport) {
        if (portName == m_port && m_state != ConnectionState::Lost && m_state != ConnectionState::NotAhrs) return;
        releaseTransport();
    }
    m_reconnectTimer->stop();
    m_port = portName;
    m_expectReboot = m_expectBootsel = false;
    openPort();
}

void DeviceSession::openPort() {
    m_transport = m_factory->acquire(m_port);
    if (!m_transport) {
        setState(ConnectionState::Disconnected, Text::tr("cannot use port %1").arg(m_port));
        return;
    }
    connect(m_transport, &Transport::dataReceived, this, &DeviceSession::onData);
    connect(m_transport, &Transport::lost, this, &DeviceSession::onLost);
    setState(ConnectionState::Opening, m_port);
    QString error;
    if (!m_transport->open(&error)) {
        releaseTransport();
        if (m_reconnectTimer->isActive()) return; // still waiting for the port to come back
        setState(ConnectionState::Disconnected, Text::tr("cannot open %1: %2").arg(m_port, error));
        return;
    }
    m_reconnectTimer->stop();
    m_rx.clear();
    m_rawExpected = -1;
    m_info = InfoReport();
    m_ownerAtt = m_appliedAtt = m_appliedRaw = m_appliedMag = m_appliedBaro = -1;
    m_flushTimer->start();
    // The firmware keeps an unfinished input line across port openings: a terminal that
    // sent text without a line ending, or a program that died mid-command, leaves its
    // characters there, and our first command would be glued to them ("infostatusinfo"
    // -> ERR unknown command). An empty line ends whatever is pending; the firmware
    // ignores an empty line by itself.
    m_transport->write(QByteArrayLiteral("\n"));
    // The firmware greets a new terminal, and may have answered the leftovers; whatever
    // it printed before we were ready is of no use. Wait, discard, then ask who it is.
    QTimer::singleShot(m_config.settleMs, this, [this, transport = m_transport] {
        if (transport != m_transport || m_state != ConnectionState::Opening) return;
        m_transport->clearInput();
        m_rx.clear();
        startIdentify();
    });
}

void DeviceSession::startIdentify() {
    setState(ConnectionState::Identifying, m_port);
    m_identifyAttempts = 0;
    sendIdentify();
}

void DeviceSession::sendIdentify() {
    ++m_identifyAttempts;
    pushInternal(QStringLiteral("info"), [this](const CommandResult &r) { onIdentified(r); }, false,
                 m_config.identifyTimeoutMs);
}

void DeviceSession::onIdentified(const CommandResult &result) {
    if (m_dropping || m_state != ConnectionState::Identifying) return;
    const InfoReport info = parseInfo(result.lines);
    if (!result.ok || !info.isAhrs()) {
        // One bad answer does not make it another firmware: the reply may have been to
        // leftovers in the firmware's input line, damaged on the way, or late because
        // the board was busy (a flash write, a sensor restart). Ask again before giving
        // up; silence gets fewer attempts, since each one costs a full timeout.
        const int allowed = result.timedOut ? m_config.identifySilentAttempts : m_config.identifyAttempts;
        if (m_identifyAttempts < allowed) {
            QTimer::singleShot(m_config.settleMs, this, [this, transport = m_transport] {
                if (transport != m_transport || m_state != ConnectionState::Identifying) return;
                m_transport->clearInput();
                m_rx.clear();
                sendIdentify();
            });
            return;
        }
        const QString why = result.ok ? Text::tr("'info' did not report fw=ins10-ahrs")
                                      : result.error;
        setState(ConnectionState::NotAhrs,
                 Text::tr("This is not the ins10_ahrs firmware (%1). The bench test ins10_test, for one, "
                          "does not speak this protocol; the console still works.")
                     .arg(why));
        return;
    }
    m_info = info;
    m_appliedAtt = static_cast<int>(info.attRateHz());
    m_appliedRaw = static_cast<int>(info.rawDiv());
    m_ownerAtt = m_appliedAtt;
    // A connection that broke while the board ran at our rate: the owner's rate was
    // remembered and the board still runs at ours.
    if (!m_config.persistGroup.isEmpty()) {
        QSettings settings;
        settings.beginGroup(m_config.persistGroup + u'/' + m_port);
        if (settings.contains(QStringLiteral("owner")) &&
            settings.value(QStringLiteral("applied")).toInt() == m_appliedAtt) {
            m_ownerAtt = settings.value(QStringLiteral("owner")).toInt();
        }
    }
    if (m_ownerAtt < 0) m_ownerAtt = 0;
    setState(ConnectionState::Connected, m_port);
    emit identified(m_info);
    emit attRatesChanged(m_ownerAtt, m_viewAtt, m_appliedAtt);
    applyAttRate();
    applyStreams();
}

// ---- Commands ----------------------------------------------------------------------

int DeviceSession::defaultTimeout(const QString &command) const {
    const QStringList w = words(command);
    if (w.size() == 2 && w[0] == u"cal" && (w[1] == u"save" || w[1] == u"erase")) return m_config.longCommandTimeoutMs;
    return m_config.commandTimeoutMs;
}

void DeviceSession::enqueue(quint64 id, const QString &command, CommandOrigin origin, int timeoutMs) {
    const QString text = command.trimmed();
    auto refuse = [&](const QString &why) {
        CommandResult r;
        r.id = id;
        r.command = text;
        r.origin = origin;
        r.error = why;
        emit commandFinished(r);
    };
    // The console may talk to whatever is on the port; everything else needs the AHRS.
    const bool usable = m_state == ConnectionState::Connected ||
                        (origin == CommandOrigin::User && m_state == ConnectionState::NotAhrs);
    if (!usable) return refuse(Text::tr("not connected"));
    if (text.isEmpty()) return refuse(Text::tr("empty command"));
    if (text.size() > kMaxCommandLength || words(text).size() > kMaxCommandWords)
        return refuse(Text::tr("command longer than the firmware accepts (%1 characters, %2 words)")
                          .arg(kMaxCommandLength)
                          .arg(kMaxCommandWords));
    Pending p;
    p.id = id;
    p.text = text;
    p.origin = origin;
    p.timeoutMs = timeoutMs > 0 ? timeoutMs : defaultTimeout(text);
    m_queue.enqueue(p);
    startNext();
}

void DeviceSession::pushInternal(const QString &text, std::function<void(const CommandResult &)> done, bool front,
                                 int timeoutMs) {
    Pending p;
    p.id = m_nextInternalId++;
    p.text = text;
    p.origin = CommandOrigin::Internal;
    p.timeoutMs = timeoutMs > 0 ? timeoutMs : defaultTimeout(text);
    p.internalDone = std::move(done);
    if (front) m_queue.prepend(p);
    else m_queue.enqueue(p);
    startNext();
}

void DeviceSession::startNext() {
    if (m_inFlight || m_queue.isEmpty() || !m_transport || !m_transport->isOpen()) return;
    Pending &head = m_queue.head();
    if (isCalSave(words(head.text)) && !head.saveWrapped) {
        head.saveWrapped = true;
        // 'cal save' stores the current $ATT rate as the power-up rate: it must be the
        // owner's, not ours.
        if (m_appliedAtt != m_ownerAtt && m_ownerAtt >= 0) {
            Pending restore;
            restore.id = m_nextInternalId++;
            restore.text = QStringLiteral("stream att %1").arg(m_ownerAtt);
            restore.origin = CommandOrigin::Internal;
            restore.timeoutMs = m_config.commandTimeoutMs;
            m_queue.prepend(restore);
        }
    }
    m_inFlight = m_queue.dequeue();
    m_reply.clear();
    const QByteArray bytes = m_inFlight->text.toLatin1() + '\n';
    m_trafficBatch.append({true, m_inFlight->text, LineKind::Other, false});
    if (!m_transport->write(bytes)) {
        finish(false, Text::tr("write to the port failed"));
        return;
    }
    m_commandTimer->start(m_inFlight->timeoutMs);
}

void DeviceSession::track(const Pending &command, bool ok) {
    if (!ok) return;
    const QStringList w = words(command.text);
    if (w.size() == 2 && w[0] == u"stream" && w[1] == u"off") {
        m_appliedAtt = m_appliedRaw = m_appliedMag = m_appliedBaro = 0;
        if (command.origin == CommandOrigin::User) m_ownerAtt = m_viewAtt = 0;
    }
    if (const int v = streamValue(w, "att"); v >= 0) {
        m_appliedAtt = v;
        if (command.origin == CommandOrigin::User) m_ownerAtt = m_viewAtt = v; // the user decides
    }
    if (const int v = streamValue(w, "raw"); v >= 0) m_appliedRaw = v;
    if (const int v = streamValue(w, "mag"); v >= 0) m_appliedMag = v;
    if (const int v = streamValue(w, "baro"); v >= 0) m_appliedBaro = v;
    if (w.size() == 1 && w[0] == u"reboot") m_expectReboot = true;
    if (w.size() == 1 && w[0] == u"bootsel") m_expectBootsel = true;
    if (w.size() >= 1 && w[0] == u"stream") {
        emit attRatesChanged(m_ownerAtt, m_viewAtt, m_appliedAtt);
        rememberRates();
    }
}

void DeviceSession::finish(bool ok, const QString &error, bool timedOut) {
    if (!m_inFlight) return;
    m_commandTimer->stop();
    const Pending command = *m_inFlight;
    m_inFlight.reset();
    CommandResult r;
    r.id = command.id;
    r.command = command.text;
    r.origin = command.origin;
    r.ok = ok;
    r.timedOut = timedOut;
    r.lines = m_reply;
    r.error = error;
    m_reply.clear();
    track(command, ok);
    if (command.origin != CommandOrigin::Internal) emit commandFinished(r);
    if (command.internalDone) command.internalDone(r);
    // After a save, back to the viewing rate.
    if (isCalSave(words(command.text)) && m_state == ConnectionState::Connected) applyAttRate();
    if (m_state == ConnectionState::Closing && !m_inFlight && m_queue.isEmpty()) {
        finishClose();
        return;
    }
    startNext();
}

void DeviceSession::failAll(const QString &error) {
    m_commandTimer->stop();
    if (m_inFlight) {
        const Pending command = *m_inFlight;
        m_inFlight.reset();
        CommandResult r;
        r.id = command.id;
        r.command = command.text;
        r.origin = command.origin;
        r.lines = m_reply;
        r.error = error;
        if (command.origin != CommandOrigin::Internal) emit commandFinished(r);
        if (command.internalDone) command.internalDone(r);
    }
    m_reply.clear();
    const QQueue<Pending> queue = m_queue;
    m_queue.clear();
    for (const Pending &command : queue) {
        CommandResult r;
        r.id = command.id;
        r.command = command.text;
        r.origin = command.origin;
        r.error = error;
        if (command.origin != CommandOrigin::Internal) emit commandFinished(r);
        if (command.internalDone) command.internalDone(r);
    }
}

// ---- Rates and streams -------------------------------------------------------------

void DeviceSession::setViewAttRate(int hz) {
    m_viewAtt = std::clamp(hz, 0, 200);
    emit attRatesChanged(m_ownerAtt, m_viewAtt, m_appliedAtt);
    if (m_state == ConnectionState::Connected) applyAttRate();
}

void DeviceSession::applyAttRate() {
    if (m_appliedAtt == m_viewAtt) return;
    pushInternal(QStringLiteral("stream att %1").arg(m_viewAtt));
}

void DeviceSession::setStreamRequest(const QString &owner, const StreamRequest &request) {
    if (request.isEmpty()) m_requests.remove(owner);
    else m_requests.insert(owner, request);
    if (m_state == ConnectionState::Connected) applyStreams();
}

StreamRequest DeviceSession::effectiveRequest() const {
    StreamRequest out;
    int bestPriority = std::numeric_limits<int>::min();
    for (const StreamRequest &r : m_requests) {
        out.mag = out.mag || r.mag;
        out.baro = out.baro || r.baro;
        if (r.rawDiv <= 0) continue;
        if (r.priority > bestPriority || (r.priority == bestPriority && r.rawDiv < out.rawDiv)) {
            bestPriority = r.priority;
            out.rawDiv = r.rawDiv;
        }
    }
    return out;
}

void DeviceSession::applyStreams() {
    const StreamRequest want = effectiveRequest();
    if (m_appliedRaw != want.rawDiv) pushInternal(QStringLiteral("stream raw %1").arg(want.rawDiv));
    if (m_appliedMag != (want.mag ? 1 : 0)) pushInternal(QStringLiteral("stream mag %1").arg(want.mag ? 1 : 0));
    if (m_appliedBaro != (want.baro ? 1 : 0)) pushInternal(QStringLiteral("stream baro %1").arg(want.baro ? 1 : 0));
}

void DeviceSession::rememberRates() {
    if (m_config.persistGroup.isEmpty() || m_port.isEmpty()) return;
    QSettings settings;
    settings.beginGroup(m_config.persistGroup + u'/' + m_port);
    if (m_appliedAtt != m_ownerAtt && m_ownerAtt >= 0) {
        settings.setValue(QStringLiteral("owner"), m_ownerAtt);
        settings.setValue(QStringLiteral("applied"), m_appliedAtt);
    } else {
        settings.remove(QString());
    }
}

void DeviceSession::forgetRates() {
    if (m_config.persistGroup.isEmpty() || m_port.isEmpty()) return;
    QSettings settings;
    settings.remove(m_config.persistGroup + u'/' + m_port);
}

// ---- Closing -----------------------------------------------------------------------

void DeviceSession::disconnectFromBoard() {
    m_reconnectTimer->stop();
    if (m_state != ConnectionState::Connected) {
        releaseTransport();
        setState(ConnectionState::Disconnected);
        return;
    }
    beginClose();
}

void DeviceSession::beginClose() {
    // Queued work is dropped; the command in flight finishes; then the board is put
    // back as we found it.
    const QQueue<Pending> queued = m_queue;
    m_queue.clear();
    for (const Pending &command : queued) {
        if (command.origin == CommandOrigin::Internal) continue;
        CommandResult r;
        r.id = command.id;
        r.command = command.text;
        r.origin = command.origin;
        r.error = Text::tr("disconnecting");
        emit commandFinished(r);
    }
    setState(ConnectionState::Closing, m_port);
    if (m_appliedRaw != 0) pushInternal(QStringLiteral("stream raw 0"));
    // Neither stream shows in 'info', so after a broken connection their state is
    // unknown (-1): switch them off to be sure. Both are off at power-up.
    if (m_appliedMag != 0) pushInternal(QStringLiteral("stream mag 0"));
    if (m_appliedBaro != 0) pushInternal(QStringLiteral("stream baro 0"));
    if (m_ownerAtt >= 0 && m_appliedAtt != m_ownerAtt) pushInternal(QStringLiteral("stream att %1").arg(m_ownerAtt));
    m_closeTimer->start(m_config.closeTimeoutMs);
    if (!m_inFlight && m_queue.isEmpty()) finishClose();
}

void DeviceSession::finishClose() {
    m_closeTimer->stop();
    if (m_state != ConnectionState::Closing) return;
    const bool restored = m_appliedAtt == m_ownerAtt && m_appliedRaw == 0 && m_appliedMag == 0 && m_appliedBaro == 0;
    failAll(Text::tr("disconnected"));
    if (restored) forgetRates();
    flush();
    releaseTransport();
    setState(ConnectionState::Disconnected,
             restored ? QString() : Text::tr("the board's stream settings could not all be restored"));
}

void DeviceSession::releaseTransport() {
    m_flushTimer->stop();
    m_commandTimer->stop();
    m_closeTimer->stop();
    if (!m_transport) return;
    disconnect(m_transport, nullptr, this, nullptr);
    m_transport->close();
    m_factory->release(m_transport);
    m_transport = nullptr;
    m_rx.clear();
}

// ---- Port loss and reconnection ----------------------------------------------------

void DeviceSession::onLost(const QString &reason) {
    flush();
    const bool reboot = m_expectReboot;
    const bool bootsel = m_expectBootsel;
    m_dropping = true;
    failAll(Text::tr("connection lost: %1").arg(reason));
    m_dropping = false;
    releaseTransport();
    m_expectReboot = m_expectBootsel = false;
    if (reboot) {
        // The board resets its streams at power-up; nothing of ours is left on it.
        forgetRates();
        setState(ConnectionState::Rebooting, m_port);
        m_reconnectDeadline = QDateTime::currentMSecsSinceEpoch() + m_config.reconnectTimeoutMs;
        m_reconnectTimer->start();
    } else if (bootsel) {
        forgetRates();
        setState(ConnectionState::Disconnected, Text::tr("the board is in the USB bootloader"));
    } else if (m_state == ConnectionState::Closing) {
        setState(ConnectionState::Disconnected, reason);
    } else {
        setState(ConnectionState::Lost, reason);
    }
}

void DeviceSession::pollReconnect() {
    if (QDateTime::currentMSecsSinceEpoch() > m_reconnectDeadline) {
        m_reconnectTimer->stop();
        setState(ConnectionState::Disconnected, Text::tr("the board did not come back after the reboot"));
        return;
    }
    if (!m_factory->isAvailable(m_port) || m_transport) return;
    openPort();
}

// ---- Receiving ---------------------------------------------------------------------

void DeviceSession::onData(const QByteArray &data) {
    m_rx += data;
    qsizetype start = 0;
    for (;;) {
        const qsizetype end = m_rx.indexOf('\n', start);
        if (end < 0) break;
        QByteArray line = m_rx.mid(start, end - start);
        if (line.endsWith('\r')) line.chop(1);
        start = end + 1;
        handleLine(QString::fromLatin1(line));
    }
    m_rx.remove(0, start);
    if (m_rx.size() > 4096) {
        // No terminator for this long: noise, or not a line protocol at all.
        ++m_counters.damagedLines;
        m_countersDirty = true;
        m_rx.clear();
    }
}

void DeviceSession::handleLine(const QString &text) {
    const ParsedLine line = parseLine(text);
    ++m_counters.linesReceived;
    m_countersDirty = true;
    if (line.damaged) ++m_counters.damagedLines;
    m_trafficBatch.append({false, line.text, line.kind, line.damaged});
    if (line.isStream()) {
        switch (line.kind) {
        case LineKind::Att: ++m_counters.attRecords; break;
        case LineKind::Mag: ++m_counters.magRecords; break;
        case LineKind::Baro: ++m_counters.baroRecords; break;
        case LineKind::Raw: {
            ++m_counters.rawRecords;
            const RawRecord &r = line.as<RawRecord>();
            if (m_rawExpected >= 0 && r.seq > m_rawExpected) m_counters.lostSamples += static_cast<quint64>(r.seq - m_rawExpected);
            m_rawExpected = r.seq + r.n;
            break;
        }
        default: break;
        }
        m_streamBatch.append(line);
        return;
    }
    if (line.kind == LineKind::Banner) return; // a greeting, not part of any reply
    if (!m_inFlight) return;
    if (line.kind == LineKind::Ok) finish(true, QString());
    else if (line.kind == LineKind::Err) {
        const QString reason = line.as<ErrLine>().reason;
        finish(false, reason.isEmpty() ? Text::tr("error") : reason);
    } else {
        m_reply.append(line.text);
    }
}

void DeviceSession::flush() {
    if (!m_streamBatch.isEmpty()) {
        StreamBatch batch;
        batch.swap(m_streamBatch);
        emit streamBatch(batch);
    }
    if (!m_trafficBatch.isEmpty()) {
        TrafficBatch batch;
        batch.swap(m_trafficBatch);
        emit traffic(batch);
    }
    if (m_countersDirty) {
        m_countersDirty = false;
        emit countersChanged(m_counters);
    }
}

void DeviceSession::resetCounters() {
    m_counters = SessionCounters();
    m_countersDirty = true;
}

} // namespace ins10
