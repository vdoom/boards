#include "core/status.h"

#include "core/protocol.h"

#include <QRegularExpression>

#include <cmath>
#include <limits>

namespace ins10 {

namespace {

// A printf %f / %g number, or what printf makes of NaN and infinity.
#define NUM R"((-?nan|[-+]?inf|[-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?))"
#define COUNT R"((\d+))"

QRegularExpression pattern(const char *text) {
    return QRegularExpression(QString::fromLatin1(text));
}

// Reads the captures first..first+count-1 as finite doubles.
bool numbers(const QRegularExpressionMatch &m, int first, int count, double *out) {
    for (int i = 0; i < count; ++i) {
        bool ok = false;
        out[i] = m.capturedView(first + i).toDouble(&ok);
        if (!ok || !std::isfinite(out[i])) return false;
    }
    return true;
}

bool counters(const QRegularExpressionMatch &m, int first, int count, quint64 *out) {
    for (int i = 0; i < count; ++i) {
        bool ok = false;
        out[i] = m.capturedView(first + i).toULongLong(&ok, 10);
        if (!ok) return false;
    }
    return true;
}

} // namespace

double InfoReport::number(const QString &key) const {
    if (!values.contains(key)) return std::numeric_limits<double>::quiet_NaN();
    bool ok = false;
    const double v = values.value(key).toDouble(&ok);
    return ok && std::isfinite(v) ? v : std::numeric_limits<double>::quiet_NaN();
}

qint64 InfoReport::integer(const QString &key) const {
    if (!values.contains(key)) return -1;
    bool ok = false;
    const qint64 v = values.value(key).toLongLong(&ok, 10);
    return ok && v >= 0 ? v : -1;
}

InfoReport parseInfo(const QStringList &lines) {
    InfoReport report;
    for (const QString &line : lines) {
        if (!line.startsWith(u'#')) continue;
        report.lines.append(line);
        for (const auto &[key, value] : parseInfoPairs(line)) report.values.insert(key, value);
    }
    return report;
}

bool StatusReport::any() const {
    return flags.valid || angles.valid || gyroBias.valid || accel.valid || mag.valid || temp.valid ||
           baro.valid || imu.valid || load.valid || magCounters.valid || filter.valid;
}

StatusReport parseStatus(const QStringList &lines) {
    static const QRegularExpression reFlags = pattern(R"(^# flags=0x([0-9A-Fa-f]+):(.*)$)");
    static const QRegularExpression reAngles = pattern(
        "^# roll=" NUM " pitch=" NUM " yaw=" NUM " heading=" NUM " deg, sigma tilt=" NUM " heading=" NUM " deg");
    static const QRegularExpression reBias = pattern("^# gyro_bias=" NUM " " NUM " " NUM R"( dps \(sigma )" NUM R"(\))");
    static const QRegularExpression reAccel =
        pattern("^# accel=" NUM " " NUM " " NUM R"( g \(norm )" NUM R"(\), disturbance )" NUM " g");
    static const QRegularExpression reMag = pattern("^# mag=" NUM " " NUM " " NUM R"( uT \(norm )" NUM
                                                    R"(\), reference )" NUM " uT dip " NUM " deg, deviation " NUM " uT");
    static const QRegularExpression reTemp = pattern("^# temp=" NUM " C, rate " NUM " C/s");
    static const QRegularExpression reBaro = pattern("^# baro=" NUM " hPa, " NUM " C, altitude " NUM " m");
    static const QRegularExpression reImu = pattern("^# imu: samples=" COUNT " gaps=" COUNT " overflows=" COUNT
                                                    " bad=" COUNT " bus_errors=" COUNT " restarts=" COUNT);
    static const QRegularExpression reLoad =
        pattern(R"(^# load: imu\+filter )" NUM " % of CPU, largest batch " COUNT " samples");
    static const QRegularExpression reMagCounters = pattern("^# mag: samples=" COUNT " pairs=" COUNT " errors=" COUNT
                                                            " restarts=" COUNT ", baro: samples=" COUNT " errors=" COUNT);
    static const QRegularExpression reFilter =
        pattern("^# filter: watchdog recoveries=" COUNT " rejected samples=" COUNT " restarts=" COUNT);

    StatusReport s;
    for (const QString &line : lines) {
        QRegularExpressionMatch m;
        double v[8];
        quint64 c[6];
        if ((m = reFlags.match(line)).hasMatch()) {
            bool ok = false;
            const uint value = m.capturedView(1).toUInt(&ok, 16);
            if (ok) {
                s.flags.valid = true;
                s.flags.value = value;
                s.flags.names = m.captured(2).split(u' ', Qt::SkipEmptyParts);
            }
        } else if ((m = reAngles.match(line)).hasMatch()) {
            if (numbers(m, 1, 6, v)) {
                s.angles = {true, v[0], v[1], v[2], v[3], v[4], v[5]};
            }
        } else if ((m = reBias.match(line)).hasMatch()) {
            if (numbers(m, 1, 4, v)) s.gyroBias = {true, {v[0], v[1], v[2]}, v[3]};
        } else if ((m = reAccel.match(line)).hasMatch()) {
            if (numbers(m, 1, 5, v)) s.accel = {true, {v[0], v[1], v[2]}, v[3], v[4]};
        } else if ((m = reMag.match(line)).hasMatch()) {
            if (numbers(m, 1, 7, v)) s.mag = {true, {v[0], v[1], v[2]}, v[3], v[4], v[5], v[6]};
        } else if ((m = reTemp.match(line)).hasMatch()) {
            if (numbers(m, 1, 2, v)) s.temp = {true, v[0], v[1]};
        } else if ((m = reBaro.match(line)).hasMatch()) {
            if (numbers(m, 1, 3, v)) s.baro = {true, v[0], v[1], v[2]};
        } else if ((m = reImu.match(line)).hasMatch()) {
            if (counters(m, 1, 6, c)) s.imu = {true, c[0], c[1], c[2], c[3], c[4], c[5]};
        } else if ((m = reLoad.match(line)).hasMatch()) {
            quint64 batch = 0;
            if (numbers(m, 1, 1, v) && counters(m, 2, 1, &batch)) s.load = {true, v[0], batch};
        } else if ((m = reMagCounters.match(line)).hasMatch()) {
            if (counters(m, 1, 6, c)) s.magCounters = {true, c[0], c[1], c[2], c[3], c[4], c[5]};
        } else if ((m = reFilter.match(line)).hasMatch()) {
            if (counters(m, 1, 3, c)) s.filter = {true, c[0], c[1], c[2]};
        }
    }
    return s;
}

std::array<double, 3> ThermalReport::biasAt(double tC) const {
    const double u = (tC - 25.0) / 10.0;
    std::array<double, 3> out{};
    for (size_t a = 0; a < 3; ++a) out[a] = coef[a][0] + coef[a][1] * u + coef[a][2] * u * u;
    return out;
}

ThermalReport parseThermal(const QStringList &lines) {
    static const QRegularExpression reObs = pattern(R"(^# observations=(\d+) range=(\S+?)\.\.(\S+) C)");
    static const QRegularExpression reAxis =
        pattern("^# ([xyz]): c0=" NUM " c1=" NUM " c2=" NUM " dps");
    static const QRegularExpression reSlope =
        pattern("^# tempco now: " NUM " " NUM " " NUM R"( dps/C \(sigma )" NUM R"(\))");
    ThermalReport t;
    for (const QString &line : lines) {
        QRegularExpressionMatch m;
        double v[4];
        if ((m = reObs.match(line)).hasMatch()) {
            bool ok = false;
            t.observations = m.capturedView(1).toULongLong(&ok);
            t.valid = ok;
            bool ok1 = false, ok2 = false;
            t.tMinC = m.capturedView(2).toDouble(&ok1);
            t.tMaxC = m.capturedView(3).toDouble(&ok2);
            t.rangeValid = ok1 && ok2 && std::isfinite(t.tMinC) && std::isfinite(t.tMaxC);
        } else if ((m = reAxis.match(line)).hasMatch()) {
            const size_t axis = static_cast<size_t>(m.capturedView(1).at(0).unicode() - u'x');
            if (axis < 3 && numbers(m, 2, 3, v)) {
                t.axisValid[axis] = true;
                t.coef[axis] = {v[0], v[1], v[2]};
            }
        } else if ((m = reSlope.match(line)).hasMatch()) {
            if (numbers(m, 1, 4, v)) {
                t.slopeValid = true;
                t.slopeDpsPerC = {v[0], v[1], v[2]};
                t.slopeSigma = v[3];
            }
        }
    }
    return t;
}

const QStringList &statusFlagNames() {
    static const QStringList names = {
        QStringLiteral("aligned"),       QStringLiteral("coarse"),        QStringLiteral("still"),
        QStringLiteral("heading"),       QStringLiteral("acc-disturbed"), QStringLiteral("mag-disturbed"),
        QStringLiteral("mag-uncal"),     QStringLiteral("saturated"),     QStringLiteral("thermal"),
        QStringLiteral("recovery"),      QStringLiteral("mag-stale")};
    return names;
}

#undef NUM
#undef COUNT

} // namespace ins10
