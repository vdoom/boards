#include "device/controller.h"

#include <QEventLoop>
#include <QThread>
#include <QTimer>

namespace ins10 {

DeviceController::DeviceController(std::unique_ptr<TransportFactory> factory, Threading threading,
                                   const SessionConfig &config, QObject *parent)
    : QObject(parent), m_factory(std::move(factory)), m_viewAtt(config.viewAttRateHz) {
    m_session = new DeviceSession(m_factory.get(), config);
    if (threading == Threading::Worker) {
        m_thread = new QThread(this);
        m_thread->setObjectName(QStringLiteral("ins10-device"));
        m_session->moveToThread(m_thread);
        m_thread->start();
    } else {
        m_session->setParent(this);
    }
    connect(m_session, &DeviceSession::stateChanged, this, [this](ConnectionState state, const QString &detail) {
        m_state = state;
        m_detail = detail;
        if (state == ConnectionState::Opening && !detail.isEmpty()) m_port = detail;
        emit stateChanged(state, detail);
    });
    connect(m_session, &DeviceSession::identified, this, [this](const InfoReport &info) {
        m_info = info;
        emit identified(info);
    });
    connect(m_session, &DeviceSession::countersChanged, this, [this](const SessionCounters &c) {
        m_counters = c;
        emit countersChanged(c);
    });
    connect(m_session, &DeviceSession::attRatesChanged, this, [this](int owner, int view, int applied) {
        m_ownerAtt = owner;
        m_viewAtt = view;
        emit attRatesChanged(owner, view, applied);
    });
    connect(m_session, &DeviceSession::streamBatch, this, &DeviceController::streamBatch);
    connect(m_session, &DeviceSession::traffic, this, &DeviceController::traffic);
    connect(m_session, &DeviceSession::commandFinished, this, &DeviceController::onFinished);
}

DeviceController::~DeviceController() {
    // The session and its transports (QSerialPort, timers) are destroyed in the thread
    // they live in. Call shutdown() first to give the board its settings back.
    auto destroy = [this] {
        delete m_session;
        m_factory->shutdown();
    };
    if (m_thread) {
        QMetaObject::invokeMethod(m_session, destroy, Qt::BlockingQueuedConnection);
        m_thread->quit();
        m_thread->wait();
    } else {
        destroy();
    }
    m_session = nullptr;
}

void DeviceController::connectTo(const QString &portName) {
    m_port = portName;
    QMetaObject::invokeMethod(m_session, [s = m_session, portName] { s->connectTo(portName); });
}

void DeviceController::disconnectFromBoard() {
    QMetaObject::invokeMethod(m_session, [s = m_session] { s->disconnectFromBoard(); });
}

bool DeviceController::shutdown(int timeoutMs) {
    if (m_state == ConnectionState::Disconnected) return true;
    QEventLoop loop;
    QTimer timeout;
    timeout.setSingleShot(true);
    connect(&timeout, &QTimer::timeout, &loop, &QEventLoop::quit);
    connect(this, &DeviceController::stateChanged, &loop, [&loop](ConnectionState state) {
        if (state == ConnectionState::Disconnected) loop.quit();
    });
    disconnectFromBoard();
    timeout.start(timeoutMs);
    loop.exec();
    return m_state == ConnectionState::Disconnected;
}

quint64 DeviceController::command(const QString &text, Callback done, CommandOrigin origin, int timeoutMs) {
    const quint64 id = m_nextId++;
    if (done) m_callbacks.insert(id, std::move(done));
    QMetaObject::invokeMethod(m_session,
                              [s = m_session, id, text, origin, timeoutMs] { s->enqueue(id, text, origin, timeoutMs); });
    return id;
}

void DeviceController::commandSequence(
    const QStringList &commands, std::function<void(bool, const QString &, const QList<CommandResult> &)> done) {
    struct State {
        QStringList commands;
        QList<CommandResult> results;
        std::function<void(bool, const QString &, const QList<CommandResult> &)> done;
        std::function<void()> next;
    };
    auto state = std::make_shared<State>();
    state->commands = commands;
    state->done = std::move(done);
    state->next = [this, weak = std::weak_ptr<State>(state)] {
        auto s = weak.lock();
        if (!s) return;
        if (s->results.size() == s->commands.size()) {
            if (s->done) s->done(true, QString(), s->results);
            s->next = nullptr;
            return;
        }
        const QString text = s->commands[s->results.size()];
        command(text, [s](const CommandResult &r) {
            s->results.append(r);
            if (!r.ok) {
                if (s->done) s->done(false, QStringLiteral("'%1': %2").arg(r.command, r.error), s->results);
                s->next = nullptr;
                return;
            }
            if (s->next) s->next();
        });
    };
    // The callbacks keep the state alive until the last one.
    auto keep = state;
    keep->next();
}

void DeviceController::setViewAttRate(int hz) {
    QMetaObject::invokeMethod(m_session, [s = m_session, hz] { s->setViewAttRate(hz); });
}

void DeviceController::setStreamRequest(const QString &owner, const StreamRequest &request) {
    QMetaObject::invokeMethod(m_session, [s = m_session, owner, request] { s->setStreamRequest(owner, request); });
}

void DeviceController::resetCounters() {
    QMetaObject::invokeMethod(m_session, [s = m_session] { s->resetCounters(); });
}

void DeviceController::onFinished(const CommandResult &result) {
    emit commandFinished(result);
    const Callback callback = m_callbacks.take(result.id);
    if (callback) callback(result);
}

} // namespace ins10
