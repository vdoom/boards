// Multi-line reports of the firmware: 'info', 'status' and 'thermal show'.
//
// 'info' is a set of "key=value" pairs. 'status' and 'thermal show' are not: they
// carry several values per key ("gyro_bias=a b c dps (sigma s)") and repeat keys
// ("heading=" twice in one line), so each line has its own pattern, taken from
// cmd_status and cmd_thermal in app/console.c. Each group is optional: a line that is
// missing or does not match leaves its group invalid instead of failing the report.
#pragma once

#include <QMap>
#include <QString>
#include <QStringList>

#include <array>

namespace ins10 {

struct InfoReport {
    QStringList lines;              // the '#' lines as received, for log headers
    QMap<QString, QString> values;  // all key=value pairs

    bool isAhrs() const { return values.value(QStringLiteral("fw")) == QLatin1String("ins10-ahrs"); }
    bool has(const QString &key) const { return values.contains(key); }
    QString text(const QString &key) const { return values.value(key); }
    // NaN when missing or not a finite number.
    double number(const QString &key) const;
    // -1 when missing or not a non-negative integer.
    qint64 integer(const QString &key) const;

    QString firmwareVersion() const { return text(QStringLiteral("version")); }
    QString board() const { return text(QStringLiteral("board")); }
    QString library() const { return text(QStringLiteral("lib")); }
    QString clock() const { return text(QStringLiteral("clock")); }
    double odrHz() const { return number(QStringLiteral("odr_hz")); }
    qint64 attRateHz() const { return integer(QStringLiteral("att_rate_hz")); }
    qint64 rawDiv() const { return integer(QStringLiteral("raw_div")); }
    qint64 calFlags() const { return integer(QStringLiteral("cal_flags")); }
};

InfoReport parseInfo(const QStringList &lines);

struct StatusReport {
    struct Flags {
        bool valid = false;
        quint32 value = 0;
        QStringList names;
    } flags;
    struct Angles {
        bool valid = false;
        double rollDeg = 0, pitchDeg = 0, yawDeg = 0, headingDeg = 0;
        double tiltSigmaDeg = 0, headingSigmaDeg = 0;
    } angles;
    struct GyroBias {
        bool valid = false;
        std::array<double, 3> dps{};
        double sigmaDps = 0;
    } gyroBias;
    struct Accel {
        bool valid = false;
        std::array<double, 3> g{};
        double norm = 0, disturbanceG = 0;
    } accel;
    struct Mag {
        bool valid = false;
        std::array<double, 3> ut{};
        double norm = 0, referenceUt = 0, dipDeg = 0, deviationUt = 0;
    } mag;
    struct Temp {
        bool valid = false;
        double c = 0, rateCps = 0;
    } temp;
    struct Baro {
        bool valid = false;
        double hpa = 0, tempC = 0, altitudeM = 0;
    } baro;
    struct Imu {
        bool valid = false;
        quint64 samples = 0, gaps = 0, overflows = 0, bad = 0, busErrors = 0, restarts = 0;
    } imu;
    struct Load {
        bool valid = false;
        double cpuPercent = 0;
        quint64 largestBatch = 0;
    } load;
    struct MagCounters {
        bool valid = false;
        quint64 samples = 0, pairs = 0, errors = 0, restarts = 0;
        quint64 baroSamples = 0, baroErrors = 0;
    } magCounters;
    struct Filter {
        bool valid = false;
        quint64 recoveries = 0, rejected = 0, restarts = 0;
    } filter;

    // True when at least one group was recognised.
    bool any() const;
};

StatusReport parseStatus(const QStringList &lines);

struct ThermalReport {
    bool valid = false;            // the observations line was recognised
    quint64 observations = 0;
    bool rangeValid = false;
    double tMinC = 0, tMaxC = 0;
    // bias = c0 + c1 u + c2 u^2 with u = (T - 25) / 10, dps; [axis][k]
    std::array<bool, 3> axisValid{};
    std::array<std::array<double, 3>, 3> coef{};
    bool slopeValid = false;
    std::array<double, 3> slopeDpsPerC{};
    double slopeSigma = 0;

    // Bias predicted by the model at temperature t (deg C), dps.
    std::array<double, 3> biasAt(double tC) const;
};

ThermalReport parseThermal(const QStringList &lines);

// Names of the status flag bits 0..10, as in PROTOCOL.md.
const QStringList &statusFlagNames();
inline constexpr int kStatusFlagCount = 11;

} // namespace ins10
