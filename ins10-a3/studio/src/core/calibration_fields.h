// The firmware's calibration record as "cal set" fields (lib/ins10/src/calib.c).
//
// A CalibrationSet maps field names to their values. It converts to and from the
// 'cal show' listing and to and from the JSON written by `ins10.py cal export`.
#pragma once

#include <QByteArray>
#include <QList>
#include <QMap>
#include <QString>
#include <QStringList>

#include <optional>

namespace ins10 {

using CalibrationSet = QMap<QString, QList<double>>;

// Bits of the "flags" field.
enum CalFlag : quint32 {
    CalGyro = 1u << 0,
    CalAccel = 1u << 1,
    CalMag = 1u << 2,
    CalGyroTc = 1u << 3,
    CalAccelTc = 1u << 4,
};
inline constexpr int kCalFlagCount = 5;

struct CalFieldDef {
    const char *name;
    int count;
    const char *unit;         // display only
    const char *description;  // English, translated through QCoreApplication::translate
};

// All fields in the order 'cal show' prints them, "flags" first.
const QList<CalFieldDef> &calibrationFields();
const CalFieldDef *findCalibrationField(const QString &name);

// Names of the flag bits ("gyro", "accel", ...), untranslated.
const char *calFlagName(int bit);

// Factory values: identity matrices, zero biases, t_ref 25 C (ins10_cal_defaults).
CalibrationSet defaultCalibration();

// %.9g in the C locale, the format of 'cal show' and of ins10.py.
QString formatCalValue(double value);

// Checks a field as the firmware would: known name, right count, finite values, flags
// a whole number in 0..255. On failure *why (if given) says what is wrong.
bool validateCalField(const QString &name, const QList<double> &values, QString *why = nullptr);

// "cal set <name> <v...>". The result always fits the console's line limit.
QString calSetCommand(const QString &name, const QList<double> &values);

// 'cal show' listing of the fields present, in the firmware's order.
QStringList calShowLines(const CalibrationSet &set);

// Fields from any "cal set" lines in the text; other lines are ignored.
CalibrationSet parseCalShow(const QString &text);

// JSON object {name: [values...]}, as ins10.py cal export.
QByteArray calibrationToJson(const CalibrationSet &set);
// Accepts that JSON. Unknown fields and wrong counts are errors.
std::optional<CalibrationSet> calibrationFromJson(const QByteArray &json, QString *error = nullptr);

// A file given to 'cal import': JSON when it starts with '{', otherwise a 'cal show'
// listing (ins10.py cal import).
std::optional<CalibrationSet> parseCalibrationFile(const QByteArray &content, QString *error = nullptr);

quint32 calFlags(const CalibrationSet &set);

// Fields whose values in `to` differ from `from` (or are missing in `from`), in the
// firmware's order.
QStringList changedCalFields(const CalibrationSet &from, const CalibrationSet &to);

// Equality as the firmware stores it: values compared after rounding to float.
bool sameCalValues(const QList<double> &a, const QList<double> &b);

} // namespace ins10
