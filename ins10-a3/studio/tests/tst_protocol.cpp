// Line parser: every example in PROTOCOL.md and every format printed by console.c and
// main.c, plus the damaged lines a congested USB link produces.
#include "core/attitude.h"
#include "core/protocol.h"
#include "core/scene.h"
#include "fixture_dir.h"

#include <QFile>
#include <QSet>
#include <QTest>

using namespace ins10;

namespace {
const char *const kAttExample =
    "$ATT,81234,0.999990,0.001200,-0.003100,0.002900,0.14,-0.36,0.33,89.67,0.2104,-0.1532,0.0871,0.03,0.21,31.42,00D";
}

class TestProtocol : public QObject {
    Q_OBJECT
private slots:
    void attitudeExample() {
        const ParsedLine line = parseLine(QString::fromLatin1(kAttExample));
        QCOMPARE(line.kind, LineKind::Att);
        QVERIFY(!line.damaged);
        const AttRecord &r = line.as<AttRecord>();
        QCOMPARE(r.tMs, 81234);
        QCOMPARE(r.q[0], 0.999990);
        QCOMPARE(r.q[2], -0.003100);
        QCOMPARE(r.rollDeg, 0.14);
        QCOMPARE(r.pitchDeg, -0.36);
        QCOMPARE(r.yawDeg, 0.33);
        QCOMPARE(r.headingDeg, 89.67);
        QCOMPARE(r.biasDps[1], -0.1532);
        QCOMPARE(r.tiltSigmaDeg, 0.03);
        QCOMPARE(r.headingSigmaDeg, 0.21);
        QCOMPARE(r.tempC, 31.42);
        QCOMPARE(r.flags, 0x00Du);
        QCOMPARE(line.text, QString::fromLatin1(kAttExample));
    }

    void attitudeAllFlags() {
        const ParsedLine line = parseLine(
            u"$ATT,1000,1.000000,0.000000,0.000000,0.000000,1.50,-2.50,3.50,86.50,0.2100,-0.1500,0.0900,0.03,0.21,31.40,7FF");
        QCOMPARE(line.kind, LineKind::Att);
        QCOMPARE(line.as<AttRecord>().flags, 0x7FFu);
    }

    void rawRecord() {
        const ParsedLine line = parseLine(u"$RAW,10,2,4,-8,12,8,0,-4,600");
        QCOMPARE(line.kind, LineKind::Raw);
        const RawRecord &r = line.as<RawRecord>();
        QCOMPARE(r.seq, 10);
        QCOMPARE(r.n, 2);
        QCOMPARE(r.gyro[1], -8);
        QCOMPARE(r.accel[2], -4);
        QCOMPARE(r.temp, 600);
        // Full-scale sums of 1024 samples still fit.
        const QString big = QStringLiteral("$RAW,4294967295,1024,%1,-%1,0,0,0,0,0").arg(qint64(1024) * 524288);
        QCOMPARE(parseLine(big).kind, LineKind::Raw);
    }

    void magAndBaro() {
        const ParsedLine mag = parseLine(u"$MAG,123,1.5,-2.5,3.5");
        QCOMPARE(mag.kind, LineKind::Mag);
        QCOMPARE(mag.as<MagRecord>().ut[1], -2.5);
        const ParsedLine baro = parseLine(u"$BARO,81234,1009.871,27.85,28.37");
        QCOMPARE(baro.kind, LineKind::Baro);
        QCOMPARE(baro.as<BaroRecord>().pressureHpa, 1009.871);
        QCOMPARE(baro.as<BaroRecord>().altitudeM, 28.37);
    }

    void infoExample() {
        const char *lines[] = {
            "# fw=ins10-ahrs version=2.0.0 board=pico lib=2.0.0",
            "# imu=ok clock=external odr_hz=512.000 measured_hz=511.987",
            "# gyro_dps_per_lsb=0.00381469727 accel_g_per_lsb=3.05175781e-05 temp_lsb_per_c=132.48 temp_offset_c=25",
            "# mag=ok mag_enabled=1 baro=ok",
            "# att_rate_hz=50 raw_div=0",
            "# last_reset=normal",
            "# cal_flags=7 stored=1 thermal_count=1840",
        };
        int pairs = 0;
        for (const char *text : lines) {
            const ParsedLine line = parseLine(QString::fromLatin1(text));
            QCOMPARE(line.kind, LineKind::Info);
            pairs += int(line.as<InfoLine>().pairs.size());
        }
        QCOMPARE(pairs, 4 + 4 + 4 + 3 + 2 + 1 + 3);
        const auto first = parseLine(QString::fromLatin1(lines[0])).as<InfoLine>().pairs;
        QCOMPARE(first[0].first, QStringLiteral("fw"));
        QCOMPARE(first[0].second, QStringLiteral("ins10-ahrs"));
    }

    void infoPairsAsPython() {
        // records.parse_info_line: trailing commas stripped, tokens without '=' skipped.
        const auto pairs = parseInfoPairs(u"# a=1, b=two c =x d=");
        QCOMPARE(pairs.size(), 3);
        QCOMPARE(pairs[0].second, QStringLiteral("1"));
        QCOMPARE(pairs[1].second, QStringLiteral("two"));
        QCOMPARE(pairs[2].first, QStringLiteral("d"));
        QCOMPARE(pairs[2].second, QString());
        QVERIFY(parseInfoPairs(u"no hash=1").isEmpty());
    }

    void banner() {
        const ParsedLine line = parseLine(u"# INS10 AHRS 2.0.0 (pico2), library 2.0.0. Type 'help'.");
        QCOMPARE(line.kind, LineKind::Banner);
        const BannerLine &b = line.as<BannerLine>();
        QCOMPARE(b.name, QStringLiteral("INS10 AHRS"));
        QCOMPARE(b.version, QStringLiteral("2.0.0"));
        QCOMPARE(b.board, QStringLiteral("pico2"));
        QCOMPARE(b.library, QStringLiteral("2.0.0"));
    }

    void helpLinesAreInfo() {
        const char *lines[] = {
            "# info                     firmware, sensors, scale factors",
            "# stream raw <n>           raw IMU: sum of n samples per line (0 = off, 1 = every sample)",
            "# reboot | bootsel         restart the Pico / enter the USB bootloader",
        };
        for (const char *text : lines) {
            const ParsedLine line = parseLine(QString::fromLatin1(text));
            QCOMPARE(line.kind, LineKind::Info);
            QVERIFY(!line.damaged);
        }
    }

    void replies() {
        QCOMPARE(parseLine(u"OK").kind, LineKind::Ok);
        QCOMPARE(parseLine(u"OK\r").kind, LineKind::Ok);
        const ParsedLine err = parseLine(u"ERR unknown command; try 'help'");
        QCOMPARE(err.kind, LineKind::Err);
        QCOMPARE(err.as<ErrLine>().reason, QStringLiteral("unknown command; try 'help'"));
        QCOMPARE(parseLine(u"ERR nothing stored").as<ErrLine>().reason, QStringLiteral("nothing stored"));
        QCOMPARE(parseLine(u"ERR").kind, LineKind::Err);
        QCOMPARE(parseLine(u"OKAY").kind, LineKind::Other);
    }

    void calLines() {
        const ParsedLine line = parseLine(u"cal set gyro_matrix 1 0 0 0 1.00400001 0 0 0 1");
        QCOMPARE(line.kind, LineKind::Cal);
        const CalLine &cal = line.as<CalLine>();
        QCOMPARE(cal.name, QStringLiteral("gyro_matrix"));
        QCOMPARE(cal.values.size(), 9);
        QCOMPARE(cal.values[4], 1.00400001);
        QCOMPARE(parseLine(u"cal set flags 7").as<CalLine>().values.front(), 7.0);
        QCOMPARE(parseLine(u"cal set mag_bias 1e-05 -2.5 3").as<CalLine>().values.front(), 1e-5);
        QVERIFY(parseLine(u"cal set gyro_bias 1 x 3").damaged);
        QVERIFY(parseLine(u"cal set gyro_bias").damaged);
    }

    void damagedLines_data() {
        QTest::addColumn<QString>("text");
        const QString att = QString::fromLatin1(kAttExample);
        QTest::newRow("truncated att") << att.left(40);
        QTest::newRow("att flags cut") << att.left(att.size() - 1);
        QTest::newRow("att glued to raw") << att.left(70) + QStringLiteral("$RAW,10,2,4,-8,12,8,0,-4,600");
        QTest::newRow("raw glued to att") << QStringLiteral("$RAW,10,2,4,-8,12,8,0,-4,600") + att;
        QTest::newRow("two raw lines") << QStringLiteral("$RAW,10,2,4,-8,12,8,0,-4,600$RAW,12,2,4,-8,12,8,0,-4,600");
        QTest::newRow("raw garbage") << QStringLiteral("$RAW,garbage");
        QTest::newRow("raw n zero") << QStringLiteral("$RAW,10,0,4,-8,12,8,0,-4,600");
        QTest::newRow("raw n too big") << QStringLiteral("$RAW,10,2048,4,-8,12,8,0,-4,600");
        QTest::newRow("raw negative seq") << QStringLiteral("$RAW,-1,2,4,-8,12,8,0,-4,600");
        QTest::newRow("raw float") << QStringLiteral("$RAW,10,2,4.5,-8,12,8,0,-4,600");
        QTest::newRow("raw over range") << QStringLiteral("$RAW,10,1,600000,0,0,0,0,0,0");
        QTest::newRow("raw missing field") << QStringLiteral("$RAW,10,2,4,-8,12,8,0,-4");
        QTest::newRow("raw extra field") << QStringLiteral("$RAW,10,2,4,-8,12,8,0,-4,600,1");
        QTest::newRow("raw empty field") << QStringLiteral("$RAW,10,2,,-8,12,8,0,-4,600");
        QTest::newRow("mag nan") << QStringLiteral("$MAG,123,nan,-2.5,3.5");
        QTest::newRow("mag inf") << QStringLiteral("$MAG,123,1.5,inf,3.5");
        QTest::newRow("mag absurd") << QStringLiteral("$MAG,123,1.5,-2.5,35000");
        QTest::newRow("mag truncated") << QStringLiteral("$MAG,123,1.5,-2.");
        QTest::newRow("baro absurd pressure") << QStringLiteral("$BARO,81234,5.0,27.85,28.37");
        QTest::newRow("baro nan") << QStringLiteral("$BARO,81234,1009.871,nan,28.37");
        QTest::newRow("att quaternion norm") << QStringLiteral(
            "$ATT,81234,1.999990,0.001200,-0.003100,0.002900,0.14,-0.36,0.33,89.67,0.2104,-0.1532,0.0871,0.03,0.21,31.42,00D");
        QTest::newRow("att heading range") << QStringLiteral(
            "$ATT,81234,0.999990,0.001200,-0.003100,0.002900,0.14,-0.36,0.33,489.67,0.2104,-0.1532,0.0871,0.03,0.21,31.42,00D");
        QTest::newRow("att pitch range") << QStringLiteral(
            "$ATT,81234,0.999990,0.001200,-0.003100,0.002900,0.14,-96.36,0.33,89.67,0.2104,-0.1532,0.0871,0.03,0.21,31.42,00D");
        QTest::newRow("att temperature") << QStringLiteral(
            "$ATT,81234,0.999990,0.001200,-0.003100,0.002900,0.14,-0.36,0.33,89.67,0.2104,-0.1532,0.0871,0.03,0.21,931.42,00D");
        QTest::newRow("att negative sigma") << QStringLiteral(
            "$ATT,81234,0.999990,0.001200,-0.003100,0.002900,0.14,-0.36,0.33,89.67,0.2104,-0.1532,0.0871,-0.03,0.21,31.42,00D");
        QTest::newRow("att nan") << QStringLiteral(
            "$ATT,81234,0.999990,0.001200,-0.003100,0.002900,nan,-0.36,0.33,89.67,0.2104,-0.1532,0.0871,0.03,0.21,31.42,00D");
        QTest::newRow("att bad hex") << QStringLiteral(
            "$ATT,81234,0.999990,0.001200,-0.003100,0.002900,0.14,-0.36,0.33,89.67,0.2104,-0.1532,0.0871,0.03,0.21,31.42,0G1");
        QTest::newRow("att space") << QStringLiteral(
            "$ATT,81234,0.999990,0.001200,-0.003100,0.002900,0.14,-0.36,0.33,89.67,0.2104,-0.1532,0.0871,0.03,0.21,31.42, 00D");
        QTest::newRow("tail of a line") << QStringLiteral("0.0871,0.03,0.21,31.42,00D");
        QTest::newRow("unknown record") << QStringLiteral("$GPS,1,2,3");
        QTest::newRow("binary noise") << QStringLiteral("$RAW,1") + QChar(0x00e9) + QStringLiteral(",2");
        QTest::newRow("control character") << QStringLiteral("# fw=ins10-ahrs\x01");
        QTest::newRow("bench firmware") << QStringLiteral("ICM-42688-P WHO_AM_I = 0x47 PASS");
    }

    void damagedLines() {
        QFETCH(QString, text);
        const ParsedLine line = parseLine(text);
        QCOMPARE(line.kind, LineKind::Other);
        QVERIFY(line.damaged);
        QVERIFY(!line.isStream());
    }

    // 15 s of what the real INS10-A3 (Pico, firmware 2.0.0) printed on 2026-11-15, the
    // board lying upside down on a slope: every line must parse, and the angles the
    // firmware printed must follow from the quaternion it printed (the 3D view draws the
    // quaternion; the tiles show the printed angles).
    void realBoardRecording() {
        QFile file(QStringLiteral(INS10_FIXTURE_DIR "/real_board_att.log"));
        QVERIFY2(file.open(QIODevice::ReadOnly), qPrintable(file.fileName()));
        int att = 0, banners = 0;
        QSet<quint32> flags;
        qint64 previous = -1;
        while (!file.atEnd()) {
            const QString text = QString::fromLatin1(file.readLine()).trimmed();
            if (text.isEmpty()) continue;
            const ParsedLine line = parseLine(text);
            QVERIFY2(!line.damaged, qPrintable(text));
            if (line.kind == LineKind::Banner) {
                ++banners;
                QCOMPARE(line.as<BannerLine>().name, QStringLiteral("INS10 AHRS"));
                QCOMPARE(line.as<BannerLine>().board, QStringLiteral("pico"));
                continue;
            }
            QCOMPARE(line.kind, LineKind::Att);
            const AttRecord &r = line.as<AttRecord>();
            ++att;
            flags.insert(r.flags);
            QVERIFY(r.tMs > previous);
            previous = r.tMs;
            const SceneAttitude scene = sceneAttitude(r);
            QVERIFY(scene.valid);
            QVERIFY2(std::abs(angleDifference(scene.euler.rollDeg, r.rollDeg)) < 0.03, qPrintable(text));
            QVERIFY2(std::abs(scene.euler.pitchDeg - r.pitchDeg) < 0.03, qPrintable(text));
            QVERIFY2(std::abs(angleDifference(scene.euler.yawDeg, r.yawDeg)) < 0.03, qPrintable(text));
            QVERIFY2(std::abs(angleDifference(scene.headingDeg, r.headingDeg)) < 0.03, qPrintable(text));
        }
        QCOMPARE(att, 763);
        QCOMPARE(banners, 1);
        // aligned + mag-uncal, with and without 'still' and 'acc-disturbed'.
        QCOMPARE(flags, (QSet<quint32>{0x041u, 0x045u, 0x051u}));
    }

    void emptyLineIsHarmless() {
        const ParsedLine line = parseLine(u"");
        QCOMPARE(line.kind, LineKind::Other);
        QVERIFY(!line.damaged);
        QCOMPARE(parseLine(u"\r").kind, LineKind::Other);
    }
};

QTEST_GUILESS_MAIN(TestProtocol)
#include "tst_protocol.moc"
