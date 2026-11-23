// Calibration fields: 'cal show' text, JSON as written by ins10.py, command limits.
#include "core/calibration_fields.h"
#include "core/protocol.h"

#include <QTest>

#include <limits>

using namespace ins10;

class TestCalFields : public QObject {
    Q_OBJECT
private slots:
    void tableMatchesFirmware() {
        // lib/ins10/src/calib.c: flags first, then the field table in this order.
        const QStringList expected = {"flags", "gyro_bias", "gyro_matrix", "accel_bias", "accel_matrix",
                                      "mag_bias", "mag_matrix", "gyro_tc_tref", "gyro_tc_x", "gyro_tc_y",
                                      "gyro_tc_z", "accel_tc_tref", "accel_tc_x", "accel_tc_y", "accel_tc_z",
                                      "declination", "mag_field", "mag_dip"};
        QStringList names;
        for (const CalFieldDef &f : calibrationFields()) names.append(QLatin1String(f.name));
        QCOMPARE(names, expected);
        QCOMPARE(findCalibrationField(QStringLiteral("mag_matrix"))->count, 9);
        QCOMPARE(findCalibrationField(QStringLiteral("accel_tc_y"))->count, 3);
        QVERIFY(!findCalibrationField(QStringLiteral("bogus")));
        QCOMPARE(defaultCalibration().size(), expected.size());
    }

    void formatIsPrintfG9() {
        QCOMPARE(formatCalValue(1.0), QStringLiteral("1"));
        QCOMPARE(formatCalValue(0.0), QStringLiteral("0"));
        QCOMPARE(formatCalValue(-0.25), QStringLiteral("-0.25"));
        QCOMPARE(formatCalValue(1e-5), QStringLiteral("1e-05"));
        QCOMPARE(formatCalValue(1.0 / 3.0), QStringLiteral("0.333333333"));
        QCOMPARE(formatCalValue(123456789012.0), QStringLiteral("1.23456789e+11"));
        QCOMPARE(formatCalValue(2000.0 / 524288.0), QStringLiteral("0.00381469727"));
    }

    void commandsFitTheConsole() {
        QList<double> worst(9, -1.23456789e-15);
        for (const CalFieldDef &f : calibrationFields()) {
            const QString cmd = calSetCommand(QLatin1String(f.name), worst.mid(0, f.count));
            QVERIFY2(cmd.size() <= kMaxCommandLength, qPrintable(cmd));
            QVERIFY(cmd.split(u' ').size() <= kMaxCommandWords);
            // And the firmware's reply parses back to the same values.
            const ParsedLine line = parseLine(cmd);
            QCOMPARE(line.kind, LineKind::Cal);
            QCOMPARE(line.as<CalLine>().values.size(), f.count);
        }
    }

    void validation() {
        QString why;
        QVERIFY(validateCalField(QStringLiteral("flags"), {7}));
        QVERIFY(!validateCalField(QStringLiteral("flags"), {256}, &why));
        QVERIFY(!validateCalField(QStringLiteral("flags"), {1.5}));
        QVERIFY(!validateCalField(QStringLiteral("flags"), {-1}));
        QVERIFY(!validateCalField(QStringLiteral("gyro_bias"), {1, 2}, &why));
        QVERIFY(!why.isEmpty());
        QVERIFY(!validateCalField(QStringLiteral("gyro_bias"), {1, std::numeric_limits<double>::infinity(), 3}));
        QVERIFY(!validateCalField(QStringLiteral("nothing"), {1}));
        QVERIFY(validateCalField(QStringLiteral("declination"), {8.5}));
    }

    void calShowRoundTrip() {
        CalibrationSet set = defaultCalibration();
        set[QStringLiteral("flags")] = {7};
        set[QStringLiteral("mag_bias")] = {23.0154991, -41.0024986, 12.0003004};
        set[QStringLiteral("declination")] = {8.5};
        const QStringList lines = calShowLines(set);
        QCOMPARE(lines.size(), set.size());
        QCOMPARE(lines.front(), QStringLiteral("cal set flags 7"));
        QCOMPARE(lines[5], QStringLiteral("cal set mag_bias 23.0154991 -41.0024986 12.0003004"));
        const CalibrationSet back = parseCalShow(lines.join(u'\n') + QStringLiteral("\nOK\n"));
        QCOMPARE(back, set);
        QCOMPARE(calFlags(back), 7u);
        QVERIFY(changedCalFields(set, back).isEmpty());
    }

    void changedFields() {
        CalibrationSet a = defaultCalibration();
        CalibrationSet b = a;
        b[QStringLiteral("accel_bias")] = {0.01, 0, 0};
        b[QStringLiteral("flags")] = {2};
        // Differences below float resolution are not changes: the board stores floats.
        b[QStringLiteral("gyro_tc_tref")] = {25.0000000001};
        QCOMPARE(changedCalFields(a, b), QStringList({"flags", "accel_bias"}));
        CalibrationSet partial;
        partial[QStringLiteral("mag_dip")] = {0};
        QCOMPARE(changedCalFields(partial, a).size(), a.size() - 1);
    }

    void jsonRoundTrip() {
        CalibrationSet set = defaultCalibration();
        set[QStringLiteral("gyro_bias")] = {0.31, -0.22, 1e-5};
        const QByteArray json = calibrationToJson(set);
        QVERIFY(json.startsWith("{\n  \"flags\": [0]"));
        const auto back = calibrationFromJson(json);
        QVERIFY(back.has_value());
        QCOMPARE(*back, set);
    }

    void pythonExportIsAccepted() {
        // json.dump(dev.read_calibration(), handle, indent=2) as ins10.py cal export writes it.
        const QByteArray python =
            "{\n  \"flags\": [\n    7.0\n  ],\n  \"gyro_bias\": [\n    0.2104,\n    -0.1532,\n    0.0871\n  ],\n"
            "  \"mag_matrix\": [\n    1.0,\n    0.0,\n    0.0,\n    0.0,\n    1.0,\n    0.0,\n    0.0,\n    0.0,\n    1.0\n  ]\n}";
        const auto set = parseCalibrationFile(python);
        QVERIFY(set.has_value());
        QCOMPARE(set->size(), 3);
        QCOMPARE(calFlags(*set), 7u);
        QCOMPARE(set->value(QStringLiteral("gyro_bias"))[1], -0.1532);
    }

    void calShowFileIsAccepted() {
        const auto set = parseCalibrationFile("> cal show\ncal set flags 3\ncal set accel_bias 0.01 0.015 -0.02\nOK\n");
        QVERIFY(set.has_value());
        QCOMPARE(set->size(), 2);
        QString error;
        QVERIFY(!parseCalibrationFile("nothing here", &error).has_value());
        QVERIFY(!error.isEmpty());
        QVERIFY(!parseCalibrationFile("{\"gyro_bias\": [1, 2]}", &error).has_value());
        QVERIFY(!parseCalibrationFile("{\"bogus\": [1]}", &error).has_value());
        QVERIFY(!parseCalibrationFile("{\"flags\": \"7\"}", &error).has_value());
        QVERIFY(!parseCalibrationFile("cal set gyro_bias 1 2\n", &error).has_value());
    }
};

QTEST_GUILESS_MAIN(TestCalFields)
#include "tst_calfields.moc"
