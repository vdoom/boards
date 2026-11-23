// Recording (records.py) and log files.
#include "core/logfile.h"
#include "core/recording.h"

#include <QTemporaryDir>
#include <QTest>

using namespace ins10;

class TestRecording : public QObject {
    Q_OBJECT
private slots:
    // The same lines and checks as test_protocol in tools/tests/test_calib.py.
    void pythonProtocolTest() {
        Recording rec;
        QCOMPARE(rec.feed(u"# gyro_dps_per_lsb=0.5 accel_g_per_lsb=0.25 temp_lsb_per_c=100 temp_offset_c=20"),
                 Recording::Fed::Info);
        QCOMPARE(rec.feed(u"# imu=ok clock=internal odr_hz=500.000 measured_hz=499.8"), Recording::Fed::Info);
        QCOMPARE(rec.feed(u"$RAW,10,2,4,-8,12,8,0,-4,600"), Recording::Fed::Raw);
        QCOMPARE(rec.feed(u"$MAG,123,1.5,-2.5,3.5"), Recording::Fed::Mag);
        QCOMPARE(rec.feed(u"$RAW,14,2,0,0,0,0,0,0,0"), Recording::Fed::Raw); // 12 and 13 never arrived
        QCOMPARE(rec.feed(u"cal set accel_bias 0.001 -0.002 0.003"), Recording::Fed::Cal);
        QCOMPARE(rec.feed(u"$RAW,garbage"), Recording::Fed::Nothing);
        QCOMPARE(rec.feed(u"OK"), Recording::Fed::Nothing);

        const ImuArrays a = rec.imuArrays();
        QCOMPARE(a.rows(), 2);
        QCOMPARE(a.gyro(0, 0), 1.0);
        QCOMPARE(a.gyro(0, 1), -2.0);
        QCOMPARE(a.gyro(0, 2), 3.0);
        QCOMPARE(a.accel(0, 0), 1.0);
        QCOMPARE(a.accel(0, 1), 0.0);
        QCOMPARE(a.accel(0, 2), -0.5);
        QCOMPARE(a.temp[0], 23.0);
        QCOMPARE(a.dt[0], 2 / 500.0);
        QCOMPARE(a.t[0], 11 / 500.0);
        QCOMPARE(rec.lostSamples(), 2);
        QCOMPARE(rec.breaks(), 1);
        QCOMPARE(rec.calibration().value(QStringLiteral("accel_bias")), QList<double>({0.001, -0.002, 0.003}));
        const MagWithImu m = rec.magWithImu();
        QCOMPARE(m.mag.rows(), 1);
        QCOMPARE(m.mag(0, 0), 1.5);
        QCOMPARE(m.mag(0, 1), -2.5);
        QCOMPARE(m.mag(0, 2), 3.5);
        QCOMPARE(m.accel(0, 0), 0.5);
        QCOMPARE(m.accel(0, 1), 0.0);
        QCOMPARE(m.accel(0, 2), -0.25);
        QVERIFY(rec.contiguity() == std::vector<bool>({false, true}));
    }

    void defaultScales() {
        Recording rec;
        rec.feed(u"$RAW,0,1,524288,0,0,0,524288,0,1324");
        const ImuArrays a = rec.imuArrays();
        QCOMPARE(a.gyro(0, 0), 2000.0);
        QCOMPARE(a.accel(0, 1), 16.0);
        QVERIFY(std::abs(a.temp[0] - (1324 / 132.48 + 25.0)) < 1e-12);
        QCOMPARE(a.t[0], 0.5 / 512.0);
    }

    void magRowsAtTheEdgesAreDropped() {
        Recording rec;
        rec.feed(u"$MAG,1,1,1,1");                  // before any IMU row
        rec.feed(u"$RAW,0,2,0,0,0,0,0,2,0");
        rec.feed(u"$MAG,2,2,2,2");
        rec.feed(u"$RAW,2,2,0,0,0,0,0,4,0");
        rec.feed(u"$MAG,3,3,3,3");                  // after the last IMU row
        QCOMPARE(rec.magRows(), 3);
        const MagWithImu m = rec.magWithImu();
        QCOMPARE(m.mag.rows(), 1);
        QCOMPARE(m.mag(0, 0), 2.0);
        QCOMPARE(m.accel(0, 2), 0.5 * (16.0 / 524288.0 + 2 * 16.0 / 524288.0));
    }

    void invalidScalesAreIgnored() {
        Recording rec;
        rec.feed(u"# odr_hz=0 gyro_dps_per_lsb=nan temp_offset_c=-5");
        QCOMPARE(rec.scales().odrHz, 512.0);
        QCOMPARE(rec.scales().gyroDpsPerLsb, 2000.0 / 524288.0);
        QCOMPARE(rec.scales().tempOffsetC, -5.0);
    }

    void tailWindow() {
        Recording rec;
        for (int i = 0; i < 10; ++i) rec.feed(QStringLiteral("$RAW,%1,1,%2,0,0,0,0,0,0").arg(i).arg(i));
        const ImuArrays a = rec.imuArrays(7, 10);
        QCOMPARE(a.rows(), 3);
        QCOMPARE(a.gyro(0, 0), 7 * 2000.0 / 524288.0);
        QVERIFY(rec.isContiguous());
    }

    void logRoundTrip() {
        QTemporaryDir dir;
        QVERIFY(dir.isValid());
        const QString path = dir.filePath(QStringLiteral("run.log"));
        const QStringList lines = {
            QStringLiteral("# fw=ins10-ahrs version=2.0.0 board=pico lib=2.0.0"),
            QStringLiteral("# imu=ok clock=external odr_hz=512.000 measured_hz=511.987"),
            QStringLiteral("$RAW,0,8,1,2,3,4,5,6,7"),
            QStringLiteral("$MAG,12,1.250,-2.500,3.750"),
            QStringLiteral("$RAW,8,8,-1,-2,-3,-4,-5,-6,-7"),
            QStringLiteral("cal set flags 7"),
        };
        LogWriter writer;
        QVERIFY(writer.open(path));
        writer.writeLines(lines.mid(0, 2));
        for (const QString &line : lines.mid(2)) writer.writeLine(line);
        QCOMPARE(writer.linesWritten(), lines.size());
        QVERIFY(writer.close());

        const auto back = readLogFile(path);
        QVERIFY(back.has_value());
        QCOMPARE(*back, lines);
        const auto rec = loadLog(path);
        QVERIFY(rec.has_value());
        QCOMPARE(rec->imuRows(), 2);
        QCOMPARE(rec->magRows(), 1);
        QCOMPARE(rec->calibration().value(QStringLiteral("flags")), QList<double>({7.0}));

        Recording direct;
        for (const QString &line : lines) direct.feed(line);
        QVERIFY(rec->imuArrays().gyro == direct.imuArrays().gyro);

        // The whole-file writer gives the same bytes.
        const QString path2 = dir.filePath(QStringLiteral("run2.log"));
        QVERIFY(writeLogFile(path2, lines));
        QFile a(path), b(path2);
        QVERIFY(a.open(QIODevice::ReadOnly) && b.open(QIODevice::ReadOnly));
        QCOMPARE(a.readAll(), b.readAll());
    }

    void logToleratesWindowsLineEnds() {
        QTemporaryDir dir;
        const QString path = dir.filePath(QStringLiteral("crlf.log"));
        QFile f(path);
        QVERIFY(f.open(QIODevice::WriteOnly));
        f.write("# odr_hz=256\r\n$RAW,0,2,0,0,0,0,0,0,0\r\n$RAW,2,2,0,0,0,0,0,0,0");
        f.close();
        const auto rec = loadLog(path);
        QVERIFY(rec.has_value());
        QCOMPARE(rec->imuRows(), 2);
        QCOMPARE(rec->scales().odrHz, 256.0);
    }

    void missingLog() {
        QString error;
        QVERIFY(!loadLog(QStringLiteral("no/such/file.log"), &error).has_value());
        QVERIFY(!error.isEmpty());
    }
};

QTEST_GUILESS_MAIN(TestRecording)
#include "tst_recording.moc"
