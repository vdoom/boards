#include "core/calibration_fields.h"

#include "core/protocol.h"

#include <QCoreApplication>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

#include <charconv>
#include <cmath>

namespace ins10 {

namespace {
struct Text {
    Q_DECLARE_TR_FUNCTIONS(ins10::CalibrationFields)
};
} // namespace

const QList<CalFieldDef> &calibrationFields() {
    static const QList<CalFieldDef> fields = {
        {"flags", 1, "", QT_TRANSLATE_NOOP("ins10::CalibrationFields", "Which calibrations are present (bit mask)")},
        {"gyro_bias", 3, "dps", QT_TRANSLATE_NOOP("ins10::CalibrationFields", "Gyro bias at the reference temperature")},
        {"gyro_matrix", 9, "", QT_TRANSLATE_NOOP("ins10::CalibrationFields", "Gyro scale and misalignment, by rows")},
        {"accel_bias", 3, "g", QT_TRANSLATE_NOOP("ins10::CalibrationFields", "Accelerometer bias")},
        {"accel_matrix", 9, "", QT_TRANSLATE_NOOP("ins10::CalibrationFields", "Accelerometer scale and cross-axis terms")},
        {"mag_bias", 3, "uT", QT_TRANSLATE_NOOP("ins10::CalibrationFields", "Magnetometer hard iron")},
        {"mag_matrix", 9, "", QT_TRANSLATE_NOOP("ins10::CalibrationFields", "Soft iron and rotation into the IMU axes")},
        {"gyro_tc_tref", 1, "C", QT_TRANSLATE_NOOP("ins10::CalibrationFields", "Reference temperature of the gyro polynomial")},
        {"gyro_tc_x", 3, "dps/C^k", QT_TRANSLATE_NOOP("ins10::CalibrationFields", "Gyro X bias drift: terms in dT, dT^2, dT^3")},
        {"gyro_tc_y", 3, "dps/C^k", QT_TRANSLATE_NOOP("ins10::CalibrationFields", "Gyro Y bias drift: terms in dT, dT^2, dT^3")},
        {"gyro_tc_z", 3, "dps/C^k", QT_TRANSLATE_NOOP("ins10::CalibrationFields", "Gyro Z bias drift: terms in dT, dT^2, dT^3")},
        {"accel_tc_tref", 1, "C", QT_TRANSLATE_NOOP("ins10::CalibrationFields", "Reference temperature of the accelerometer polynomial")},
        {"accel_tc_x", 3, "g/C^k", QT_TRANSLATE_NOOP("ins10::CalibrationFields", "Accelerometer X bias drift: terms in dT, dT^2, dT^3")},
        {"accel_tc_y", 3, "g/C^k", QT_TRANSLATE_NOOP("ins10::CalibrationFields", "Accelerometer Y bias drift: terms in dT, dT^2, dT^3")},
        {"accel_tc_z", 3, "g/C^k", QT_TRANSLATE_NOOP("ins10::CalibrationFields", "Accelerometer Z bias drift: terms in dT, dT^2, dT^3")},
        {"declination", 1, "deg", QT_TRANSLATE_NOOP("ins10::CalibrationFields", "Magnetic declination, east positive (0: heading from magnetic north)")},
        {"mag_field", 1, "uT", QT_TRANSLATE_NOOP("ins10::CalibrationFields", "Field magnitude found by the calibration (informational)")},
        {"mag_dip", 1, "deg", QT_TRANSLATE_NOOP("ins10::CalibrationFields", "Dip angle found by the calibration (informational)")},
    };
    return fields;
}

const CalFieldDef *findCalibrationField(const QString &name) {
    for (const CalFieldDef &f : calibrationFields()) {
        if (name == QLatin1String(f.name)) return &f;
    }
    return nullptr;
}

const char *calFlagName(int bit) {
    static const char *const names[kCalFlagCount] = {"gyro", "accel", "mag", "gyro_tc", "accel_tc"};
    return bit >= 0 && bit < kCalFlagCount ? names[bit] : "";
}

CalibrationSet defaultCalibration() {
    const QList<double> identity = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    const QList<double> zero3 = {0, 0, 0};
    CalibrationSet set;
    set.insert(QStringLiteral("flags"), {0});
    set.insert(QStringLiteral("gyro_bias"), zero3);
    set.insert(QStringLiteral("gyro_matrix"), identity);
    set.insert(QStringLiteral("accel_bias"), zero3);
    set.insert(QStringLiteral("accel_matrix"), identity);
    set.insert(QStringLiteral("mag_bias"), zero3);
    set.insert(QStringLiteral("mag_matrix"), identity);
    set.insert(QStringLiteral("gyro_tc_tref"), {25});
    set.insert(QStringLiteral("gyro_tc_x"), zero3);
    set.insert(QStringLiteral("gyro_tc_y"), zero3);
    set.insert(QStringLiteral("gyro_tc_z"), zero3);
    set.insert(QStringLiteral("accel_tc_tref"), {25});
    set.insert(QStringLiteral("accel_tc_x"), zero3);
    set.insert(QStringLiteral("accel_tc_y"), zero3);
    set.insert(QStringLiteral("accel_tc_z"), zero3);
    set.insert(QStringLiteral("declination"), {0});
    set.insert(QStringLiteral("mag_field"), {0});
    set.insert(QStringLiteral("mag_dip"), {0});
    return set;
}

QString formatCalValue(double value) {
    char buffer[40];
    const auto result = std::to_chars(buffer, buffer + sizeof(buffer), value, std::chars_format::general, 9);
    return QString::fromLatin1(buffer, result.ptr - buffer);
}

bool validateCalField(const QString &name, const QList<double> &values, QString *why) {
    auto fail = [why](const QString &text) {
        if (why) *why = text;
        return false;
    };
    const CalFieldDef *def = findCalibrationField(name);
    if (!def) return fail(Text::tr("unknown field '%1'").arg(name));
    if (values.size() != def->count)
        return fail(Text::tr("%1 takes %2 values, not %3").arg(name).arg(def->count).arg(values.size()));
    for (double v : values) {
        if (!std::isfinite(v)) return fail(Text::tr("%1: values must be finite numbers").arg(name));
    }
    if (name == QLatin1String("flags")) {
        const double v = values.front();
        if (!(v >= 0.0 && v < 256.0) || v != std::floor(v))
            return fail(Text::tr("flags must be a whole number from 0 to 255"));
    }
    return true;
}

QString calSetCommand(const QString &name, const QList<double> &values) {
    QString text = QStringLiteral("cal set ") + name;
    for (double v : values) text += u' ' + formatCalValue(v);
    Q_ASSERT(text.size() <= kMaxCommandLength);
    Q_ASSERT(values.size() + 3 <= kMaxCommandWords);
    return text;
}

QStringList calShowLines(const CalibrationSet &set) {
    QStringList lines;
    for (const CalFieldDef &f : calibrationFields()) {
        const QString name = QLatin1String(f.name);
        if (set.contains(name)) lines.append(calSetCommand(name, set.value(name)));
    }
    return lines;
}

CalibrationSet parseCalShow(const QString &text) {
    CalibrationSet set;
    for (QStringView line : QStringView(text).split(u'\n')) {
        const ParsedLine parsed = parseLine(line.trimmed());
        if (parsed.kind == LineKind::Cal) {
            const CalLine &cal = parsed.as<CalLine>();
            set.insert(cal.name, cal.values);
        }
    }
    return set;
}

QByteArray calibrationToJson(const CalibrationSet &set) {
    QByteArray out = "{\n";
    bool first = true;
    auto writeField = [&](const QString &name) {
        if (!first) out += ",\n";
        first = false;
        out += "  \"" + name.toUtf8() + "\": [";
        const QList<double> values = set.value(name);
        for (qsizetype i = 0; i < values.size(); ++i) {
            if (i) out += ", ";
            out += formatCalValue(values[i]).toLatin1();
        }
        out += "]";
    };
    for (const CalFieldDef &f : calibrationFields()) {
        if (set.contains(QLatin1String(f.name))) writeField(QLatin1String(f.name));
    }
    for (auto it = set.cbegin(); it != set.cend(); ++it) {
        if (!findCalibrationField(it.key())) writeField(it.key());
    }
    out += "\n}\n";
    return out;
}

std::optional<CalibrationSet> calibrationFromJson(const QByteArray &json, QString *error) {
    QJsonParseError parseError;
    const QJsonDocument doc = QJsonDocument::fromJson(json, &parseError);
    if (doc.isNull() || !doc.isObject()) {
        if (error) *error = Text::tr("not a JSON object: %1").arg(parseError.errorString());
        return std::nullopt;
    }
    CalibrationSet set;
    const QJsonObject object = doc.object();
    for (auto it = object.begin(); it != object.end(); ++it) {
        if (!it.value().isArray()) {
            if (error) *error = Text::tr("%1: expected a list of numbers").arg(it.key());
            return std::nullopt;
        }
        QList<double> values;
        for (const QJsonValue &v : it.value().toArray()) {
            if (!v.isDouble()) {
                if (error) *error = Text::tr("%1: expected a list of numbers").arg(it.key());
                return std::nullopt;
            }
            values.append(v.toDouble());
        }
        if (!validateCalField(it.key(), values, error)) return std::nullopt;
        set.insert(it.key(), values);
    }
    return set;
}

std::optional<CalibrationSet> parseCalibrationFile(const QByteArray &content, QString *error) {
    const QByteArray trimmed = content.trimmed();
    CalibrationSet set;
    if (trimmed.startsWith('{')) {
        auto parsed = calibrationFromJson(trimmed, error);
        if (!parsed) return std::nullopt;
        set = *parsed;
    } else {
        set = parseCalShow(QString::fromUtf8(content));
        for (auto it = set.cbegin(); it != set.cend(); ++it) {
            if (!validateCalField(it.key(), it.value(), error)) return std::nullopt;
        }
    }
    if (set.isEmpty()) {
        if (error) *error = Text::tr("no calibration found in the file");
        return std::nullopt;
    }
    return set;
}

quint32 calFlags(const CalibrationSet &set) {
    const QList<double> v = set.value(QStringLiteral("flags"));
    if (v.isEmpty() || !(v.front() >= 0.0 && v.front() < 256.0)) return 0;
    return static_cast<quint32>(v.front());
}

bool sameCalValues(const QList<double> &a, const QList<double> &b) {
    if (a.size() != b.size()) return false;
    for (qsizetype i = 0; i < a.size(); ++i) {
        if (static_cast<float>(a[i]) != static_cast<float>(b[i])) return false;
    }
    return true;
}

QStringList changedCalFields(const CalibrationSet &from, const CalibrationSet &to) {
    QStringList out;
    for (const CalFieldDef &f : calibrationFields()) {
        const QString name = QLatin1String(f.name);
        if (!to.contains(name)) continue;
        if (!from.contains(name) || !sameCalValues(from.value(name), to.value(name))) out.append(name);
    }
    return out;
}

} // namespace ins10
