// Replay through lib/ins10: the same result as the firmware's own ins10_replay
// (tests/replay.c, built here from the firmware sources) on the same log.
#include "core/logfile.h"
#include "device/fakeboard.h"
#include "device/ports.h"
#include "device/session.h"
#include "fixture_dir.h"
#include "replay/replay.h"

#include <QProcess>
#include <QTemporaryDir>
#include <QTest>

#include <cmath>

using namespace ins10;

namespace {

QStringList referenceCsv(const QString &log, const QString &calibration = QString()) {
    QProcess p;
    QStringList args{log};
    if (!calibration.isEmpty()) args.append(calibration);
    p.start(QString::fromUtf8(INS10_REPLAY_REFERENCE), args);
    if (!p.waitForFinished(120000) || p.exitCode() != 0) return {};
    QStringList lines = QString::fromLatin1(p.readAllStandardOutput()).split(u'\n', Qt::SkipEmptyParts);
    for (QString &l : lines) l = l.trimmed();
    return lines;
}

// Same rows; every number within one unit of its last printed digit (the two printf
// implementations may round the last digit differently).
bool sameCsv(const QStringList &a, const QStringList &b, QString *why) {
    if (a.size() != b.size()) {
        *why = QStringLiteral("%1 rows vs %2").arg(a.size()).arg(b.size());
        return false;
    }
    if (a.isEmpty() || a.front() != b.front()) {
        *why = QStringLiteral("header differs");
        return false;
    }
    for (qsizetype i = 1; i < a.size(); ++i) {
        const QStringList x = a[i].split(u','), y = b[i].split(u',');
        if (x.size() != y.size()) {
            *why = QStringLiteral("row %1: field count").arg(i);
            return false;
        }
        for (qsizetype k = 0; k < x.size(); ++k) {
            if (x[k] == y[k]) continue;
            if (x[k].startsWith(QStringLiteral("0x"))) {
                *why = QStringLiteral("row %1: flags %2 vs %3").arg(i).arg(x[k], y[k]);
                return false;
            }
            const qsizetype dot = x[k].indexOf(u'.');
            const int decimals = dot < 0 ? 0 : static_cast<int>(x[k].size() - dot - 1);
            if (std::abs(x[k].toDouble() - y[k].toDouble()) > 1.01 * std::pow(10.0, -decimals)) {
                *why = QStringLiteral("row %1 field %2: %3 vs %4").arg(i).arg(k).arg(x[k], y[k]);
                return false;
            }
        }
    }
    return true;
}

// A log recorded from the simulated board at raw 1 with the magnetometer, as the
// recording page writes it.
QStringList demoLog(double seconds) {
    FakeBoardConfig config;
    config.realTime = false;
    config.motion = FakeBoardConfig::MotionKind::Demo;
    FakeBoard board(config);
    FakeTransportFactory factory(&board);
    SessionConfig sc;
    sc.settleMs = 5;
    sc.batchIntervalMs = 5;
    DeviceSession session(&factory, sc);
    QStringList lines;
    QObject::connect(&session, &DeviceSession::traffic, [&](const TrafficBatch &batch) {
        for (const TrafficLine &l : batch) {
            if (!l.outgoing && (l.kind == LineKind::Raw || l.kind == LineKind::Mag)) lines.append(l.text);
        }
    });
    session.connectTo(kDemoPortName);
    if (!QTest::qWaitFor([&] { return session.state() == ConnectionState::Connected; }, 3000)) return {};
    session.setStreamRequest(QStringLiteral("test"), {1, true, false, 0});
    (void)QTest::qWaitFor([&] { return session.queuedCommands() == 0; }, 3000);
    lines.clear();
    for (double t = 0; t < seconds; t += 0.5) {
        board.advance(0.5);
        QCoreApplication::processEvents();
    }
    QTest::qWait(50);
    // The header the recording page writes.
    QStringList log = session.info().lines;
    log.append(QStringLiteral("cal set declination 3.5"));
    log.append(lines);
    return log;
}

} // namespace

class TestReplay : public QObject {
    Q_OBJECT
private slots:
    void sameAsInsReplayOnADemoRecording() {
        const QStringList log = demoLog(60.0);
        QVERIFY(log.size() > 25000);
        QTemporaryDir dir;
        const QString path = dir.filePath(QStringLiteral("demo.log"));
        QVERIFY(writeLogFile(path, log));
        const ReplayResult result = replayLog(log);
        QVERIFY(result.rows.size() > 25000);
        QCOMPARE(result.lostSamples, 0);
        const QStringList reference = referenceCsv(path);
        QVERIFY(!reference.isEmpty());
        QString why;
        QVERIFY2(sameCsv(replayCsv(result), reference, &why), qPrintable(why));
        // The filter, on the simulated data, follows the true trajectory.
        bool heading = false;
        for (const ReplayRow &r : result.rows) heading |= (r.flags & 0x008) != 0;
        qInfo("%zu rows over %.1f s; heading referenced: %s", result.rows.size(), result.seconds, heading ? "yes" : "no");
    }

    void sameAsInsReplayWithACalibrationFile() {
        QTemporaryDir dir;
        const QString log = QString::fromUtf8(INS10_FIXTURE_DIR) + QStringLiteral("/imu.log");
        const QString cal = dir.filePath(QStringLiteral("cal.txt"));
        const QStringList calibration = {QStringLiteral("cal set flags 3"),
                                         QStringLiteral("cal set accel_bias 0.012 -0.018 0.025"),
                                         QStringLiteral("cal set gyro_bias 0.31 -0.22 0.14"),
                                         QStringLiteral("cal set bogus 1")};
        QVERIFY(writeLogFile(cal, calibration));
        ReplayOptions options;
        options.calibration = calibration;
        const ReplayResult result = replayLog(*readLogFile(log), options);
        QCOMPARE(result.ignoredCalibration.size(), 1);
        QVERIFY(!result.rows.empty());
        QString why;
        QVERIFY2(sameCsv(replayCsv(result), referenceCsv(log, cal), &why), qPrintable(why));
    }

    void cancelStops() {
        std::atomic<bool> cancel{true};
        ReplayOptions options;
        options.cancel = &cancel;
        const ReplayResult result = replayLog({QStringLiteral("$RAW,0,1,0,0,0,0,0,32768,0")}, options);
        QVERIFY(result.cancelled);
        QCOMPARE(result.imuRecords, 0);
    }
};

QTEST_GUILESS_MAIN(TestReplay)
#include "tst_replay.moc"
