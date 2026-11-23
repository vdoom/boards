// A simulated INS10 board running ins10_ahrs, behind the Transport interface.
//
// It answers the whole command set with the firmware's formats (app/console.c), keeps
// a RAM and a "flash" copy of the calibration, and streams $ATT/$RAW/$MAG/$BARO
// generated from a motion and sensor model with built-in errors. $ATT is the true
// attitude. Used by the tests and by --demo.
//
// Time runs either in real time (a timer, for --demo) or only when advance() is
// called (tests). Replies and stream data are always delivered asynchronously
// through the event loop, like a serial port.
#pragma once

#include "core/calibration_fields.h"
#include "device/simulation.h"
#include "device/transport.h"

#include <QElapsedTimer>
#include <QStringList>

#include <algorithm>
#include <functional>
#include <memory>
#include <optional>
#include <random>

class QTimer;

namespace ins10 {

struct FakeBoardHealth {
    quint64 imuGaps = 0, overflows = 0, badPackets = 0, busErrors = 0, imuRestarts = 0;
    quint64 magErrors = 0, magRestarts = 0, baroErrors = 0;
    double loadPercent = 14.2;
    quint32 largestBatch = 2;
    bool externalClock = true;
    bool watchdogReset = false;
    bool imuOk = true, magOk = true, baroOk = true;
    // Multiplies the accelerometer counts: 4 or 0.25 imitates a wrong FIFO scale.
    double accelCountScale = 1.0;
    double magRateHz = 50.0;   // magnetometer samples per second (pairs: one per 50)
};

struct FakeBoardConfig {
    enum class MotionKind { Demo, Still, MultiPosition, Tumble };
    sim::SensorModel sensors = sim::SensorModel::typical();
    MotionKind motion = MotionKind::Demo;
    int positions = 16;
    double tumbleRateDps = 55.0;
    quint64 motionSeed = 5;
    double tempC = 30.0;                     // MultiPosition, Tumble
    double stillT0C = 25.0, stillT1C = 25.0, stillTauS = 600.0;
    double odrHz = 512.0;
    int magEvery = 10;                       // IMU samples per magnetometer sample
    int baroEvery = 20;
    bool realTime = true;
    double speed = 1.0;                      // real-time factor
    quint64 noiseSeed = 7;
    QString board = QStringLiteral("pico2");
    QString version = QStringLiteral("2.0.0");
    QString library = QStringLiteral("2.0.0");
    QString firmware = QStringLiteral("ins10-ahrs"); // another value imitates a different firmware
    int bootAttRateHz = 50;                  // flash default
    int bannerDelayMs = 2;
    int rebootDownMs = 1500;                 // how long the port is gone after 'reboot'
};

class FakeBoard : public Transport {
    Q_OBJECT
public:
    explicit FakeBoard(const FakeBoardConfig &config = {}, QObject *parent = nullptr);
    ~FakeBoard() override;

    // ---- Transport ----
    QString portName() const override;
    bool open(QString *error) override;
    void close() override;
    bool isOpen() const override { return m_open; }
    bool write(const QByteArray &data) override;
    void clearInput() override;

    // ---- Simulation ----
    // Runs the board for this much board time (any mode) and queues the output.
    void advance(double seconds);
    double boardTime() const { return m_time; }
    bool isAvailable() const { return m_available; }
    void unplug();                 // the cable is pulled: the port vanishes at once
    void plugIn();                 // and comes back (the board kept running)
    void setRealTime(bool on);
    void setSpeed(double factor) { m_config.speed = std::clamp(factor, 0.1, 100.0); }
    Eigen::Vector4d trueAttitude() const { return m_sim->attitude(); }
    const sim::SensorModel &sensors() const { return m_sim->sensors(); }
    FakeBoardHealth &health() { return m_health; }
    const FakeBoardConfig &config() const { return m_config; }
    // Replaces the motion (same kinds as the configuration) without a reboot.
    void setMotion(FakeBoardConfig::MotionKind kind);

    // ---- Fault injection ----
    // What a terminal leaves behind when it sends text without a line ending: the
    // characters wait in the firmware's input line, across port openings, and the next
    // command is appended to them.
    void typeWithoutLineEnding(const QByteArray &text);
    void setReplyDelayMs(int ms) { m_replyDelayMs = ms; }
    void setMute(bool mute) { m_mute = mute; }            // never answer commands
    // Each output line is damaged with this probability: cut short (and glued to the
    // next), or its start or a piece of its middle lost.
    void setLineDamage(double probability, quint64 seed = 99);
    // Returns replacement reply lines for a command, or nothing for normal handling.
    void setCommandOverride(std::function<std::optional<QStringList>(const QString &)> fn) {
        m_override = std::move(fn);
    }

    // ---- Introspection ----
    const QStringList &commands() const { return m_commands; }
    void clearCommands() { m_commands.clear(); }
    int attRate() const { return m_attRate; }
    int rawDiv() const { return m_rawDiv; }
    bool magStream() const { return m_magStream; }
    bool baroStream() const { return m_baroStream; }
    bool hasStored() const { return m_stored.has_value(); }
    int storedAttRate() const { return m_stored ? m_stored->attRate : m_config.bootAttRateHz; }
    int saveCount() const { return m_saveCount; }
    CalibrationSet ramCalibration() const { return m_cal; }
    std::optional<CalibrationSet> storedCalibration() const;
    qint64 linesSent() const { return m_linesSent; }

signals:
    void commandExecuted(const QString &command);

private:
    struct Stored {
        CalibrationSet cal;
        bool magEnable = true;
        int attRate = 50;
        quint64 thermalCount = 0;
    };

    void tick();
    void simulate(double seconds);
    void processInput();
    void execute(const QString &line);
    QStringList run(const QStringList &argv);
    QStringList cmdInfo() const;
    QStringList cmdStatus() const;
    QStringList cmdThermal() const;
    QStringList cmdCal(const QStringList &argv);
    QStringList cmdStream(const QStringList &argv);
    void reply(const QStringList &lines);
    void output(const QString &line);
    void scheduleDelivery();
    void deliver();
    void bootState();
    void startMotion();
    QString attLine() const;
    QString damaged(const QString &line);

    // Calibrated readings, as the firmware's filter sees them.
    Eigen::Vector3d calibratedAccel() const;
    Eigen::Vector3d calibratedMag() const;
    Eigen::Vector3d residualBias() const;
    quint32 statusFlags() const;
    bool still() const;
    bool firmwareIsAhrs() const;

    FakeBoardConfig m_config;
    FakeBoardHealth m_health;
    std::unique_ptr<sim::Motion> m_motion;
    std::unique_ptr<sim::Simulator> m_sim;
    QTimer *m_timer;
    QElapsedTimer m_wall;
    double m_time = 0.0;            // board time since boot, s
    double m_nextAtt = 0.0;
    double m_stillFor = 0.0;
    double m_prevTemp = 25.0, m_tempRate = 0.0;
    sim::SensorReading m_last;
    Eigen::Vector3d m_lastMag = Eigen::Vector3d::Zero();

    // Firmware state.
    bool m_open = false, m_available = true;
    bool m_mute = false;
    int m_replyDelayMs = 0;
    int m_attRate = 50, m_rawDiv = 0;
    bool m_magStream = false, m_baroStream = false, m_magEnable = true;
    CalibrationSet m_cal;
    std::optional<Stored> m_stored;
    int m_saveCount = 0;
    quint64 m_thermalCount = 0;
    double m_thermalMin = 0.0, m_thermalMax = 0.0;
    quint64 m_magSamples = 0, m_baroSamples = 0;
    qint64 m_rawFirst = 0;
    int m_rawN = 0;
    qint64 m_rawSum[7] = {};
    double m_baroHpa = 1009.87;

    QByteArray m_rx, m_tx;
    bool m_processPending = false, m_deliverPending = false;
    QStringList m_commands;
    qint64 m_linesSent = 0;
    double m_damageProbability = 0.0;
    std::mt19937_64 m_damageRng;
    std::function<std::optional<QStringList>(const QString &)> m_override;
};

// Serial ports through QSerialPort, plus the simulated board under kDemoPortName.
class DefaultTransportFactory : public TransportFactory {
public:
    explicit DefaultTransportFactory(const FakeBoardConfig &demo = demoConfig());
    // The simulated board of --demo: real time, warm-up from 18 to 42 C when lying still.
    static FakeBoardConfig demoConfig();
    ~DefaultTransportFactory() override;
    Transport *acquire(const QString &portName) override;
    void release(Transport *transport) override;
    bool isAvailable(const QString &portName) override;
    void shutdown() override { m_demo.reset(); }
    FakeBoard *demoBoard() const { return m_demo.get(); }

private:
    FakeBoardConfig m_demoConfig;
    std::unique_ptr<FakeBoard> m_demo;
};

// Only the given simulated board (tests).
class FakeTransportFactory : public TransportFactory {
public:
    explicit FakeTransportFactory(FakeBoard *board) : m_board(board) {}
    Transport *acquire(const QString &) override { return m_board; }
    void release(Transport *) override {}
    bool isAvailable(const QString &) override { return m_board->isAvailable(); }

private:
    FakeBoard *m_board;
};

} // namespace ins10
