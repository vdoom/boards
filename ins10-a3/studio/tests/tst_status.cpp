// Reports of 'info', 'status' and 'thermal show', written out from the printf formats
// of cmd_info, cmd_status and cmd_thermal in app/console.c.
#include "core/status.h"

#include <QTest>

using namespace ins10;

namespace {
QStringList statusLines() {
    return {
        QStringLiteral("# flags=0x00D: aligned still heading"),
        QStringLiteral("# roll=0.14 pitch=-0.36 yaw=0.33 heading=89.67 deg, sigma tilt=0.03 heading=0.21 deg"),
        QStringLiteral("# gyro_bias=0.2104 -0.1532 0.0871 dps (sigma 0.0012)"),
        QStringLiteral("# accel=0.0025 -0.0063 1.0001 g (norm 1.0001), disturbance 0.002 g"),
        QStringLiteral("# mag=12.34 -20.11 -45.67 uT (norm 51.39), reference 51.20 uT dip 66.2 deg, deviation 0.31 uT"),
        QStringLiteral("# temp=31.42 C, rate 0.0012 C/s"),
        QStringLiteral("# baro=1009.871 hPa, 27.85 C, altitude 28.4 m"),
        QStringLiteral("# imu: samples=1234567 gaps=1 overflows=2 bad=3 bus_errors=4 restarts=5"),
        QStringLiteral("# load: imu+filter 17.3 % of CPU, largest batch 3 samples (last 10 s)"),
        QStringLiteral("# mag: samples=123456 pairs=2468 errors=7 restarts=8, baro: samples=61728 errors=9"),
        QStringLiteral("# filter: watchdog recoveries=10 rejected samples=11 restarts=12"),
    };
}
} // namespace

class TestStatus : public QObject {
    Q_OBJECT
private slots:
    void info() {
        const InfoReport info = parseInfo({
            QStringLiteral("# fw=ins10-ahrs version=2.0.0 board=pico lib=2.0.0"),
            QStringLiteral("# imu=ok clock=external odr_hz=512.000 measured_hz=511.987"),
            QStringLiteral("# gyro_dps_per_lsb=0.00381469727 accel_g_per_lsb=3.05175781e-05 temp_lsb_per_c=132.48 temp_offset_c=25"),
            QStringLiteral("# mag=ok mag_enabled=1 baro=ok"),
            QStringLiteral("# att_rate_hz=50 raw_div=0"),
            QStringLiteral("# last_reset=normal"),
            QStringLiteral("# cal_flags=7 stored=1 thermal_count=1840"),
            QStringLiteral("$ATT,not part of the report"),
        });
        QVERIFY(info.isAhrs());
        QCOMPARE(info.lines.size(), 7);
        QCOMPARE(info.firmwareVersion(), QStringLiteral("2.0.0"));
        QCOMPARE(info.board(), QStringLiteral("pico"));
        QCOMPARE(info.clock(), QStringLiteral("external"));
        QCOMPARE(info.odrHz(), 512.0);
        QCOMPARE(info.attRateHz(), 50);
        QCOMPARE(info.rawDiv(), 0);
        QCOMPARE(info.calFlags(), 7);
        QCOMPARE(info.text(QStringLiteral("last_reset")), QStringLiteral("normal"));
        QVERIFY(std::isnan(info.number(QStringLiteral("missing"))));
        QCOMPARE(info.integer(QStringLiteral("board")), -1);
        QVERIFY(!parseInfo({QStringLiteral("# fw=ins10-test")}).isAhrs());
    }

    void status() {
        const StatusReport s = parseStatus(statusLines());
        QVERIFY(s.flags.valid);
        QCOMPARE(s.flags.value, 0x00Du);
        QCOMPARE(s.flags.names, QStringList({"aligned", "still", "heading"}));
        QVERIFY(s.angles.valid);
        QCOMPARE(s.angles.headingDeg, 89.67);
        QCOMPARE(s.angles.headingSigmaDeg, 0.21);
        QCOMPARE(s.angles.tiltSigmaDeg, 0.03);
        QVERIFY(s.gyroBias.valid);
        QCOMPARE(s.gyroBias.dps[2], 0.0871);
        QCOMPARE(s.gyroBias.sigmaDps, 0.0012);
        QVERIFY(s.accel.valid);
        QCOMPARE(s.accel.norm, 1.0001);
        QCOMPARE(s.accel.disturbanceG, 0.002);
        QVERIFY(s.mag.valid);
        QCOMPARE(s.mag.norm, 51.39);
        QCOMPARE(s.mag.referenceUt, 51.20);
        QCOMPARE(s.mag.dipDeg, 66.2);
        QCOMPARE(s.mag.deviationUt, 0.31);
        QVERIFY(s.temp.valid);
        QCOMPARE(s.temp.rateCps, 0.0012);
        QVERIFY(s.baro.valid);
        QCOMPARE(s.baro.altitudeM, 28.4);
        QVERIFY(s.imu.valid);
        QCOMPARE(s.imu.samples, 1234567u);
        QCOMPARE(s.imu.busErrors, 4u);
        QCOMPARE(s.imu.restarts, 5u);
        QVERIFY(s.load.valid);
        QCOMPARE(s.load.cpuPercent, 17.3);
        QCOMPARE(s.load.largestBatch, 3u);
        QVERIFY(s.magCounters.valid);
        QCOMPARE(s.magCounters.pairs, 2468u);
        QCOMPARE(s.magCounters.baroErrors, 9u);
        QVERIFY(s.filter.valid);
        QCOMPARE(s.filter.rejected, 11u);
        QCOMPARE(s.filter.restarts, 12u);
    }

    void statusAllFlagNames() {
        const StatusReport s = parseStatus({QStringLiteral(
            "# flags=0x7FF: aligned coarse still heading acc-disturbed mag-disturbed mag-uncalibrated saturated "
            "thermal-transient recovery mag-stale")});
        QCOMPARE(s.flags.value, 0x7FFu);
        QCOMPARE(s.flags.names.size(), 11);
        QCOMPARE(parseStatus({QStringLiteral("# flags=0x000:")}).flags.names.size(), 0);
    }

    void statusWithoutBarometerOrDamaged() {
        QStringList lines = statusLines();
        lines.removeAt(6);                                    // baro not valid: line omitted
        lines[1] = QStringLiteral("# roll=nan pitch=-0.36 yaw=0.33 heading=89.67 deg, sigma tilt=0.03 heading=0.21 deg");
        lines[3] = lines[3].left(20);                         // truncated
        const StatusReport s = parseStatus(lines);
        QVERIFY(!s.baro.valid);
        QVERIFY(!s.angles.valid);
        QVERIFY(!s.accel.valid);
        QVERIFY(s.mag.valid && s.imu.valid && s.filter.valid);
        QVERIFY(s.any());
        QVERIFY(!parseStatus({QStringLiteral("garbage")}).any());
    }

    void thermal() {
        const ThermalReport t = parseThermal({
            QStringLiteral("# observations=1840 range=21.3..35.0 C"),
            QStringLiteral("# x: c0=0.21040 c1=0.01200 c2=-0.00030 dps"),
            QStringLiteral("# y: c0=-0.15320 c1=0.00800 c2=0.00010 dps"),
            QStringLiteral("# z: c0=0.08710 c1=-0.00500 c2=0.00020 dps"),
            QStringLiteral("# tempco now: 0.00120 0.00080 -0.00050 dps/C (sigma 0.00030)"),
        });
        QVERIFY(t.valid && t.rangeValid);
        QCOMPARE(t.observations, 1840u);
        QCOMPARE(t.tMinC, 21.3);
        QCOMPARE(t.tMaxC, 35.0);
        QVERIFY(t.axisValid[0] && t.axisValid[1] && t.axisValid[2]);
        QCOMPARE(t.coef[1][0], -0.1532);
        QCOMPARE(t.coef[2][2], 0.0002);
        QVERIFY(t.slopeValid);
        QCOMPARE(t.slopeDpsPerC[2], -0.0005);
        QCOMPARE(t.slopeSigma, 0.0003);
        // u = (35 - 25) / 10 = 1: c0 + c1 + c2
        QVERIFY(std::abs(t.biasAt(35.0)[0] - (0.2104 + 0.012 - 0.0003)) < 1e-12);
    }

    void thermalEmptyModel() {
        const ThermalReport t = parseThermal({QStringLiteral("# observations=0 range=0.0..0.0 C"),
                                              QStringLiteral("# x: c0=0.00000 c1=0.00000 c2=0.00000 dps")});
        QVERIFY(t.valid && t.rangeValid);
        QCOMPARE(t.observations, 0u);
        QVERIFY(t.axisValid[0] && !t.axisValid[1]);
        QVERIFY(!t.slopeValid);
    }

    void flagNames() {
        QCOMPARE(statusFlagNames().size(), kStatusFlagCount);
        QCOMPARE(statusFlagNames()[6], QStringLiteral("mag-uncal"));
    }
};

QTEST_GUILESS_MAIN(TestStatus)
#include "tst_status.moc"
