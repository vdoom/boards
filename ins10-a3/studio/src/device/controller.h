// The GUI's handle on the board: runs a DeviceSession in a worker thread (or, for
// tests, in the caller's thread), forwards its signals and turns command replies into
// callbacks in the caller's thread.
#pragma once

#include "device/session.h"

#include <QHash>
#include <QObject>

#include <functional>
#include <memory>

class QThread;

namespace ins10 {

class DeviceController : public QObject {
    Q_OBJECT
public:
    enum class Threading { Worker, SameThread };
    using Callback = std::function<void(const CommandResult &)>;

    explicit DeviceController(std::unique_ptr<TransportFactory> factory, Threading threading = Threading::Worker,
                              const SessionConfig &config = {}, QObject *parent = nullptr);
    ~DeviceController() override;

    void connectTo(const QString &portName);
    void disconnectFromBoard();
    // Restores the board and closes the port, waiting up to timeoutMs (runs a local
    // event loop). For application exit.
    bool shutdown(int timeoutMs = 4000);

    quint64 command(const QString &text, Callback done = {}, CommandOrigin origin = CommandOrigin::App,
                    int timeoutMs = 0);
    // Sends the commands one after another, stopping at the first failure.
    void commandSequence(const QStringList &commands,
                         std::function<void(bool ok, const QString &error, const QList<CommandResult> &)> done);

    void setViewAttRate(int hz);
    void setStreamRequest(const QString &owner, const StreamRequest &request);
    void releaseStreams(const QString &owner) { setStreamRequest(owner, StreamRequest()); }
    void resetCounters();

    ConnectionState state() const { return m_state; }
    bool isConnected() const { return m_state == ConnectionState::Connected; }
    QString portName() const { return m_port; }
    QString stateDetail() const { return m_detail; }
    const InfoReport &info() const { return m_info; }
    const SessionCounters &counters() const { return m_counters; }
    int ownerAttRate() const { return m_ownerAtt; }
    int viewAttRate() const { return m_viewAtt; }

    // The session (tests, SameThread only) and the transport factory.
    DeviceSession *session() const { return m_session; }
    TransportFactory *factory() const { return m_factory.get(); }

signals:
    void stateChanged(ins10::ConnectionState state, const QString &detail);
    void identified(const ins10::InfoReport &info);
    void streamBatch(const ins10::StreamBatch &batch);
    void traffic(const ins10::TrafficBatch &batch);
    void countersChanged(const ins10::SessionCounters &counters);
    void commandFinished(const ins10::CommandResult &result);
    void attRatesChanged(int owner, int view, int applied);

private:
    void onFinished(const CommandResult &result);

    std::unique_ptr<TransportFactory> m_factory;
    QThread *m_thread = nullptr;
    DeviceSession *m_session = nullptr;
    QHash<quint64, Callback> m_callbacks;
    quint64 m_nextId = 1;
    ConnectionState m_state = ConnectionState::Disconnected;
    QString m_port, m_detail;
    InfoReport m_info;
    SessionCounters m_counters;
    int m_ownerAtt = -1, m_viewAtt = 50;
};

} // namespace ins10
