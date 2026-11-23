// First-run checklist and status history, on reports from the simulated board with
// healthy and with faulty hardware.
#include "core/health.h"
#include "device/boardmodel.h"
#include "device/fakeboard.h"
#include "device/ports.h"

#include <QTest>

using namespace ins10;

namespace {

StatusReport reportWith(quint64 magSamples, quint64 pairs, quint64 gaps = 0) {
    StatusReport s;
    s.magCounters = {true, magSamples, pairs, 0, 0, 0, 0};
    s.imu = {true, 1000, gaps, 0, 0, 0, 0};
    return s;
}

const CheckResult &find(const QList<CheckResult> &checks, const QString &id) {
    for (const CheckResult &c : checks) {
        if (c.id == id) return c;
    }
    static CheckResult none;
    return none;
}

// Runs the simulated board through a BoardModel for `seconds` of board time while
// polling 'status' once a (board) second, and returns the checklist.
QList<CheckResult> runBoard(FakeBoard &board, double seconds) {
    FakeTransportFactory *factory = new FakeTransportFactory(&board);
    SessionConfig config;
    config.settleMs = 5;
    config.batchIntervalMs = 5;
    DeviceController device(std::unique_ptr<TransportFactory>(factory), DeviceController::Threading::SameThread, config);
    BoardModel model(&device);
    device.connectTo(kDemoPortName);
    if (!QTest::qWaitFor([&] { return device.isConnected(); }, 3000)) return {};
    (void)QTest::qWaitFor([&] { return device.session()->queuedCommands() == 0; }, 3000);
    // Status is polled against board time here, not the PC clock.
    StatusHistory history;
    for (int i = 0; i <= static_cast<int>(seconds); ++i) {
        board.advance(1.0);
        bool done = false;
        device.command(QStringLiteral("status"), [&](const CommandResult &r) {
            history.add(i, parseStatus(r.lines));
            done = true;
        });
        if (!QTest::qWaitFor([&] { return done; }, 3000)) return {};
    }
    return evaluateFirstRun(device.info(), history);
}

} // namespace

class TestHealth : public QObject {
    Q_OBJECT
private slots:
    void historyRates() {
        StatusHistory h(100);
        StatusHistory::Pick samples = [](const StatusReport &s) -> std::optional<double> {
            return s.magCounters.valid ? std::optional<double>(static_cast<double>(s.magCounters.samples)) : std::nullopt;
        };
        QVERIFY(!h.rate(samples, 10).has_value());
        for (int i = 0; i <= 10; ++i) h.add(i, reportWith(100 + 50 * i, 2 * i));
        QCOMPARE(*h.rate(samples, 10), 50.0);
        QCOMPARE(*h.growth(samples, 4), 200.0);
        // Too short a history for the window: no answer rather than a wrong one.
        StatusHistory shortHistory;
        shortHistory.add(0, reportWith(0, 0));
        shortHistory.add(1, reportWith(50, 1));
        QVERIFY(!shortHistory.rate(samples, 10).has_value());
        // A counter that went backwards (restart) gives no rate.
        h.add(11, reportWith(10, 0));
        QVERIFY(!h.rate(samples, 3).has_value());
        // Capacity is respected.
        for (int i = 0; i < 200; ++i) h.add(20 + i, reportWith(i, i));
        QCOMPARE(h.samples().size(), size_t(100));
    }

    void healthyBoardPasses() {
        FakeBoardConfig config;
        config.realTime = false;
        config.motion = FakeBoardConfig::MotionKind::Still;
        // Sensor errors within what the checks expect of an uncalibrated board, plus
        // noise. (The typical model's z reading sits right at 1.02 g, where the
        // noise of one standard library or another decides.)
        config.sensors = sim::SensorModel::ideal();
        config.sensors.gyroBias = {0.31, -0.22, 0.14};
        config.sensors.accelBias = {0.004, -0.006, 0.008};
        config.sensors.gyroNoiseDps = 0.045;
        config.sensors.accelNoiseG = 0.0011;
        config.sensors.magNoiseUt = 0.05;
        FakeBoard board(config);
        const QList<CheckResult> checks = runBoard(board, 12);
        QCOMPARE(checks.size(), 10);
        for (const CheckResult &c : checks) {
            if (c.id == QLatin1String("stored")) {
                QCOMPARE(c.level, CheckLevel::Warning); // nothing saved yet on a fresh board
                continue;
            }
            QVERIFY2(c.level == CheckLevel::Ok, qPrintable(c.id + u": " + c.detail + u" / " + c.advice));
        }
    }

    void faultsAreReported() {
        FakeBoardConfig config;
        config.realTime = false;
        config.motion = FakeBoardConfig::MotionKind::Still;
        FakeBoard board(config);
        board.health().externalClock = false;
        board.health().accelCountScale = 0.25;  // wrong FIFO scale
        board.health().loadPercent = 72.0;
        board.health().watchdogReset = true;
        board.health().busErrors = 3;
        const QList<CheckResult> checks = runBoard(board, 12);
        QCOMPARE(find(checks, QStringLiteral("clock")).level, CheckLevel::Warning);
        QCOMPARE(find(checks, QStringLiteral("accel")).level, CheckLevel::Fail);
        QVERIFY(find(checks, QStringLiteral("accel")).advice.contains(QStringLiteral("icm42688.h")));
        QCOMPARE(find(checks, QStringLiteral("load")).level, CheckLevel::Fail);
        QCOMPARE(find(checks, QStringLiteral("reset")).level, CheckLevel::Warning);
        QCOMPARE(find(checks, QStringLiteral("imu_counters")).level, CheckLevel::Warning); // non-zero, not growing
    }

    // As on the real board (2026-11-15): aligned, lying untouched, but the filter
    // reports 'still' almost never and 'acc-disturbed' every second or two. The two
    // checks that wait for 'still' must say so instead of waiting for ever.
    void neverStillIsExplained() {
        auto report = [](quint32 flags) {
            StatusReport s;
            s.flags = {true, flags, {}};
            s.accel = {true, {0.605, 0.004, -0.794}, 0.9973, 0.004};
            s.gyroBias = {true, {-0.3175, -0.1124, 0.3952}, 0.01};
            return s;
        };
        StatusHistory h;
        h.add(0, report(0x045)); // still for a moment after alignment
        for (int i = 1; i <= 10; ++i) h.add(i, report(i % 2 ? 0x041 : 0x051));
        // Ten seconds: still worth waiting.
        QCOMPARE(find(evaluateFirstRun(InfoReport(), h), QStringLiteral("accel")).level, CheckLevel::Pending);
        QCOMPARE(find(evaluateFirstRun(InfoReport(), h), QStringLiteral("gyro")).level, CheckLevel::Pending);
        for (int i = 11; i <= 20; ++i) h.add(i, report(i % 2 ? 0x041 : 0x051));
        const QList<CheckResult> checks = evaluateFirstRun(InfoReport(), h);
        for (const char *id : {"accel", "gyro"}) {
            const CheckResult &c = find(checks, QLatin1String(id));
            QCOMPARE(c.level, CheckLevel::Warning);
            QVERIFY2(c.advice.contains(QStringLiteral("'still' for 20 s")), qPrintable(c.advice));
        }
        // Once the board does come to rest, the checks judge the readings as before.
        h.add(21, report(0x045));
        QCOMPARE(find(evaluateFirstRun(InfoReport(), h), QStringLiteral("accel")).level, CheckLevel::Ok);
        QCOMPARE(find(evaluateFirstRun(InfoReport(), h), QStringLiteral("gyro")).level, CheckLevel::Ok);
    }

    void growingCountersFail() {
        StatusHistory h;
        for (int i = 0; i <= 12; ++i) h.add(i, reportWith(50 * i, i, i / 3));
        const QList<CheckResult> checks = evaluateFirstRun(InfoReport(), h);
        QCOMPARE(find(checks, QStringLiteral("imu_counters")).level, CheckLevel::Fail);
        QCOMPARE(find(checks, QStringLiteral("mag_rate")).level, CheckLevel::Ok);
        QCOMPARE(find(checks, QStringLiteral("imu")).level, CheckLevel::Pending); // no 'info'
    }
};

QTEST_GUILESS_MAIN(TestHealth)
#include "tst_health.moc"
