#include "device/fakeboard.h"

#include "core/attitude.h"
#include "core/protocol.h"
#include "core/status.h"
#include "device/ports.h"
#include "device/serial_transport.h"

#include <QRegularExpression>
#include <QSerialPortInfo>
#include <QTimer>

#include <algorithm>
#include <cmath>
#include <numbers>

namespace ins10 {

namespace {

constexpr double kGyroDpsPerLsb = static_cast<double>(2000.0f / 524288.0f);
constexpr double kAccelGPerLsb = static_cast<double>(16.0f / 524288.0f);
constexpr double kTempLsbPerC = static_cast<double>(132.48f);
constexpr double kTempOffsetC = 25.0;
constexpr qint64 kMaxCount = (1 << 19) - 1;

qint64 toCounts(double value, double perLsb) {
    const double counts = std::round(value / perLsb);
    return static_cast<qint64>(std::clamp(counts, -static_cast<double>(kMaxCount), static_cast<double>(kMaxCount)));
}

QString fmt(const char *format, auto... args) { return QString::asprintf(format, args...); }

float toFloat(double v) { return static_cast<float>(v); }

QList<double> asFloats(const QList<double> &values) {
    QList<double> out;
    for (double v : values) out.append(static_cast<double>(toFloat(v)));
    return out;
}

Eigen::Matrix3d matrixOf(const CalibrationSet &cal, const char *name) {
    const QList<double> v = cal.value(QLatin1String(name));
    Eigen::Matrix3d m = Eigen::Matrix3d::Identity();
    if (v.size() == 9) {
        for (int i = 0; i < 9; ++i) m(i / 3, i % 3) = v[i];
    }
    return m;
}

Eigen::Vector3d vectorOf(const CalibrationSet &cal, const char *name) {
    const QList<double> v = cal.value(QLatin1String(name));
    return v.size() == 3 ? Eigen::Vector3d(v[0], v[1], v[2]) : Eigen::Vector3d::Zero();
}

// The calibration's temperature polynomial of "gyro" or "accel" at temp.
Eigen::Vector3d calDrift(const CalibrationSet &cal, const QString &prefix, double temp) {
    const QList<double> ref = cal.value(prefix + QStringLiteral("_tc_tref"));
    const double d = temp - (ref.isEmpty() ? 25.0 : ref.front());
    Eigen::Vector3d out;
    const char *axes[3] = {"_tc_x", "_tc_y", "_tc_z"};
    for (int a = 0; a < 3; ++a) {
        const QList<double> c = cal.value(prefix + QLatin1String(axes[a]));
        out[a] = c.size() == 3 ? ((c[2] * d + c[1]) * d + c[0]) * d : 0.0;
    }
    return out;
}

const char *const kStatusFlagWords[kStatusFlagCount] = {
    "aligned", "coarse", "still", "heading", "acc-disturbed", "mag-disturbed", "mag-uncalibrated",
    "saturated", "thermal-transient", "recovery", "mag-stale"};

} // namespace

FakeBoard::FakeBoard(const FakeBoardConfig &config, QObject *parent)
    : Transport(parent), m_config(config), m_timer(new QTimer(this)), m_damageRng(99) {
    m_timer->setInterval(10);
    connect(m_timer, &QTimer::timeout, this, &FakeBoard::tick);
    startMotion();
    bootState();
}

FakeBoard::~FakeBoard() = default;

void FakeBoard::startMotion() {
    using K = FakeBoardConfig::MotionKind;
    switch (m_config.motion) {
    case K::Demo: m_motion = std::make_unique<sim::DemoMotion>(); break;
    case K::Still:
        m_motion = std::make_unique<sim::StillMotion>(m_config.stillT0C, m_config.stillT1C, m_config.stillTauS);
        break;
    case K::MultiPosition:
        m_motion = std::make_unique<sim::MultiPositionMotion>(m_config.positions, m_config.motionSeed, m_config.tempC);
        break;
    case K::Tumble:
        m_motion = std::make_unique<sim::TumbleMotion>(m_config.tumbleRateDps, m_config.motionSeed, m_config.tempC);
        break;
    }
    const sim::SensorModel sensors = m_sim ? m_sim->sensors() : m_config.sensors;
    m_sim = std::make_unique<sim::Simulator>(m_motion.get(), sensors, m_config.odrHz, m_config.noiseSeed);
}

void FakeBoard::setMotion(FakeBoardConfig::MotionKind kind) {
    m_config.motion = kind;
    const Eigen::Vector4d q = m_sim->attitude();
    startMotion();
    m_sim->setAttitude(q);
    m_time = m_nextAtt = 0.0;
}

// Power-up: the stored record (or factory defaults), streams off, filter restarted.
void FakeBoard::bootState() {
    m_cal = m_stored ? m_stored->cal : defaultCalibration();
    m_magEnable = m_stored ? m_stored->magEnable : true;
    m_attRate = m_stored ? m_stored->attRate : m_config.bootAttRateHz;
    m_thermalCount = m_stored ? m_stored->thermalCount : 0;
    m_rawDiv = 0;
    m_rawN = 0;
    m_magStream = m_baroStream = false;
    m_time = m_nextAtt = m_stillFor = 0.0;
    m_magSamples = m_baroSamples = 0;
    m_rx.clear(); // the console's line buffer starts empty
    m_sim->reset();
    m_prevTemp = m_sim->temperature();
}

std::optional<CalibrationSet> FakeBoard::storedCalibration() const {
    if (!m_stored) return std::nullopt;
    return m_stored->cal;
}

QString FakeBoard::portName() const { return kDemoPortName; }

bool FakeBoard::open(QString *error) {
    if (!m_available) {
        if (error) *error = QStringLiteral("the simulated board is not connected");
        return false;
    }
    m_open = true;
    // m_rx is kept: console_poll()'s line buffer does not know that the host closed
    // and reopened the port, so an unfinished line is still there.
    m_tx.clear();
    if (m_config.realTime) {
        m_wall.start();
        m_timer->start();
    }
    // connection_service(): the banner goes out when DTR rises.
    QTimer::singleShot(m_config.bannerDelayMs, this, [this] {
        if (firmwareIsAhrs()) {
            output(fmt("# INS10 AHRS %s (%s), library %s. Type 'help'.", qPrintable(m_config.version),
                       qPrintable(m_config.board), qPrintable(m_config.library)));
        }
    });
    return true;
}

void FakeBoard::close() {
    m_open = false;
    m_timer->stop();
    m_tx.clear();
}

void FakeBoard::typeWithoutLineEnding(const QByteArray &text) { m_rx += text; }

void FakeBoard::setRealTime(bool on) {
    m_config.realTime = on;
    if (on && m_open) {
        m_wall.start();
        m_timer->start();
    } else {
        m_timer->stop();
    }
}

bool FakeBoard::write(const QByteArray &data) {
    if (!m_open) return false;
    m_rx += data;
    if (!m_processPending) {
        m_processPending = true;
        QTimer::singleShot(0, this, &FakeBoard::processInput);
    }
    return true;
}

void FakeBoard::clearInput() { m_tx.clear(); }

void FakeBoard::unplug() {
    if (!m_available) return;
    m_available = false;
    const bool wasOpen = m_open;
    close();
    if (wasOpen) emit lost(QStringLiteral("device unplugged"));
}

void FakeBoard::plugIn() { m_available = true; }

void FakeBoard::setLineDamage(double probability, quint64 seed) {
    m_damageProbability = probability;
    m_damageRng.seed(seed);
}

// ---- Time --------------------------------------------------------------------------

void FakeBoard::tick() {
    const double elapsed = static_cast<double>(m_wall.restart()) * 1e-3 * m_config.speed;
    simulate(std::min(elapsed, 0.5));
}

void FakeBoard::advance(double seconds) { simulate(seconds); }

bool FakeBoard::still() const { return m_stillFor > 0.5; }

void FakeBoard::simulate(double seconds) {
    if (!(seconds > 0.0)) return;
    const double end = m_time + seconds;
    const double odr = m_config.odrHz;
    const bool ahrs = firmwareIsAhrs();
    while (static_cast<double>(m_sim->sampleIndex()) / odr <= end) {
        const qint64 index = m_sim->sampleIndex();
        m_last = m_sim->step();
        const double t = m_sim->time();
        const double dt = 1.0 / odr;
        m_stillFor = m_sim->trueRateDps().norm() < 0.5 ? m_stillFor + dt : 0.0;
        if (index % static_cast<qint64>(odr) == 0) {
            m_tempRate = m_last.tempC - m_prevTemp;
            m_prevTemp = m_last.tempC;
            if (still()) {
                if (m_thermalCount == 0) m_thermalMin = m_thermalMax = m_last.tempC;
                m_thermalMin = std::min(m_thermalMin, m_last.tempC);
                m_thermalMax = std::max(m_thermalMax, m_last.tempC);
                ++m_thermalCount;
            }
        }
        if (!ahrs) {
            if (index % static_cast<qint64>(odr) == 0) output(QStringLiteral("ICM-42688-P   WHO_AM_I 0x47   PASS"));
            continue;
        }
        // Magnetometer, at about 50 Hz.
        const int magEvery = std::max(1, static_cast<int>(std::lround(odr / m_health.magRateHz)));
        if (m_magEnable && index % (m_config.magEvery > 0 ? m_config.magEvery : magEvery) == 0) {
            m_lastMag = m_sim->magReading();
            ++m_magSamples;
            if (m_magStream) {
                output(fmt("$MAG,%lld,%.3f,%.3f,%.3f", static_cast<long long>(static_cast<double>(index) / odr * 1000.0),
                           m_lastMag[0], m_lastMag[1], m_lastMag[2]));
            }
        }
        // $RAW: sums of raw_div samples (raw_stream() in main.c).
        if (m_rawDiv == 0) {
            m_rawN = 0;
        } else {
            if (m_rawN == 0) {
                m_rawFirst = index;
                std::fill(std::begin(m_rawSum), std::end(m_rawSum), 0);
            }
            for (int a = 0; a < 3; ++a) {
                m_rawSum[a] += toCounts(m_last.gyroDps[a], kGyroDpsPerLsb);
                m_rawSum[a + 3] += toCounts(m_last.accelG[a] * m_health.accelCountScale, kAccelGPerLsb);
            }
            m_rawSum[6] += static_cast<qint64>(std::round((m_last.tempC - kTempOffsetC) * kTempLsbPerC));
            if (++m_rawN >= m_rawDiv) {
                output(fmt("$RAW,%lld,%d,%lld,%lld,%lld,%lld,%lld,%lld,%lld", static_cast<long long>(m_rawFirst), m_rawN,
                           static_cast<long long>(m_rawSum[0]), static_cast<long long>(m_rawSum[1]),
                           static_cast<long long>(m_rawSum[2]), static_cast<long long>(m_rawSum[3]),
                           static_cast<long long>(m_rawSum[4]), static_cast<long long>(m_rawSum[5]),
                           static_cast<long long>(m_rawSum[6])));
                m_rawN = 0;
            }
        }
        // Barometer, 25 Hz.
        if (index % std::max(1, m_config.baroEvery) == 0) {
            ++m_baroSamples;
            m_baroHpa = 1009.87 + 0.03 * std::sin(0.05 * t);
            if (m_baroStream) {
                const double altitude = 44330.0 * (1.0 - std::pow(m_baroHpa / 1013.25, 0.190295));
                output(fmt("$BARO,%lld,%.3f,%.2f,%.2f", static_cast<long long>(t * 1000.0), m_baroHpa,
                           m_last.tempC - 2.0, altitude));
            }
        }
        // $ATT at its own rate.
        if (m_attRate > 0 && t >= m_nextAtt) {
            const double period = 1.0 / m_attRate;
            m_nextAtt += period;
            if (t - m_nextAtt > period) m_nextAtt = t;
            output(attLine());
        }
    }
    m_time = end;
}

// ---- What the filter would report --------------------------------------------------

Eigen::Vector3d FakeBoard::calibratedAccel() const {
    const Eigen::Vector3d raw = m_last.accelG * m_health.accelCountScale;
    return matrixOf(m_cal, "accel_matrix") *
           (raw - vectorOf(m_cal, "accel_bias") - calDrift(m_cal, QStringLiteral("accel"), m_last.tempC));
}

Eigen::Vector3d FakeBoard::calibratedMag() const {
    return matrixOf(m_cal, "mag_matrix") * (m_lastMag - vectorOf(m_cal, "mag_bias"));
}

// Gyro bias left after the board's calibration: what the EKF tracks.
Eigen::Vector3d FakeBoard::residualBias() const {
    const sim::SensorModel &s = m_sim->sensors();
    const double temp = m_sim->temperature();
    const Eigen::Vector3d rawBias = s.gyroBias + s.gyroDrift(temp);
    return matrixOf(m_cal, "gyro_matrix") *
           (rawBias - vectorOf(m_cal, "gyro_bias") - calDrift(m_cal, QStringLiteral("gyro"), temp));
}

quint32 FakeBoard::statusFlags() const {
    quint32 flags = 0;
    if (m_time > 1.0) flags |= 0x001;               // aligned
    if (still()) flags |= 0x004;
    const bool magCal = (calFlags(m_cal) & CalMag) != 0;
    if (m_magEnable && magCal && m_time > 1.0) flags |= 0x008;
    if (m_magEnable && !magCal) flags |= 0x040;
    if (std::abs(m_tempRate) > 0.02) flags |= 0x100;
    return flags;
}

QString FakeBoard::attLine() const {
    const Eigen::Vector4d q = quatNormalized(m_sim->attitude());
    const EulerAngles e = quatToEuler(q);
    const double heading = quatToHeading(q);
    const Eigen::Vector3d bias = residualBias();
    const quint32 flags = statusFlags();
    return fmt("$ATT,%lld,%.6f,%.6f,%.6f,%.6f,%.2f,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%03X",
               static_cast<long long>(m_time * 1000.0), q[0], q[1], q[2], q[3], e.rollDeg, e.pitchDeg, e.yawDeg, heading,
               bias[0], bias[1], bias[2], 0.03, (flags & 0x008) ? 0.21 : 3.5, m_sim->temperature(), flags);
}

// ---- Console -----------------------------------------------------------------------

bool FakeBoard::firmwareIsAhrs() const { return m_config.firmware == QLatin1String("ins10-ahrs"); }

void FakeBoard::processInput() {
    m_processPending = false;
    // console_poll(): '\r' or '\n' ends a line; a line of 200 characters or more is
    // refused as a whole.
    for (;;) {
        qsizetype end = -1;
        for (qsizetype i = 0; i < m_rx.size(); ++i) {
            if (m_rx[i] == '\n' || m_rx[i] == '\r') {
                end = i;
                break;
            }
        }
        if (end < 0) break;
        const QByteArray line = m_rx.left(end);
        m_rx.remove(0, end + 1);
        if (line.isEmpty()) continue;
        execute(QString::fromLatin1(line));
    }
}

void FakeBoard::execute(const QString &line) {
    m_commands.append(line);
    emit commandExecuted(line);
    if (!firmwareIsAhrs() || m_mute) return;
    if (m_override) {
        if (auto lines = m_override(line)) {
            reply(*lines);
            return;
        }
    }
    if (line.size() > kMaxCommandLength) {
        reply({QStringLiteral("ERR line too long")});
        return;
    }
    QStringList argv = line.split(QRegularExpression(QStringLiteral("[ \t]+")), Qt::SkipEmptyParts);
    if (argv.isEmpty()) return;
    if (argv.size() > kMaxCommandWords) argv = argv.mid(0, kMaxCommandWords);
    reply(run(argv));
}

QStringList FakeBoard::run(const QStringList &argv) {
    const QString cmd = argv[0];
    if (cmd == QLatin1String("help") || cmd == QLatin1String("?")) {
        return {QStringLiteral("# info                     firmware, sensors, scale factors"),
                QStringLiteral("# status                   filter state and health counters"),
                QStringLiteral("# stream att <hz>          attitude output rate (0 = off, max 200)"),
                QStringLiteral("# stream raw <n>           raw IMU: sum of n samples per line (0 = off, 1 = every sample)"),
                QStringLiteral("# stream mag <0|1>         raw magnetometer"),
                QStringLiteral("# stream baro <0|1>        barometer"),
                QStringLiteral("# stream off               stop all streams"),
                QStringLiteral("# cal show                 print the calibration as 'cal set' lines"),
                QStringLiteral("# cal set <name> <v...>    change one calibration field (RAM only)"),
                QStringLiteral("# cal save                 store calibration, thermal model and settings in flash"),
                QStringLiteral("# cal load                 discard changes, reload from flash"),
                QStringLiteral("# cal erase                factory defaults, erase flash"),
                QStringLiteral("# thermal show | reset     learned gyro bias-vs-temperature model"),
                QStringLiteral("# set mag <0|1>            use the magnetometer (stored with 'cal save')"),
                QStringLiteral("# reset                    restart the filter from initial alignment"),
                QStringLiteral("# reboot | bootsel         restart the Pico / enter the USB bootloader"),
                QStringLiteral("OK")};
    }
    if (cmd == QLatin1String("info")) return cmdInfo();
    if (cmd == QLatin1String("status")) return cmdStatus();
    if (cmd == QLatin1String("stream")) return cmdStream(argv);
    if (cmd == QLatin1String("cal")) return cmdCal(argv);
    if (cmd == QLatin1String("thermal")) {
        if (argv.size() >= 2 && argv[1] == QLatin1String("reset")) {
            m_thermalCount = 0;
            return {QStringLiteral("OK")};
        }
        return cmdThermal();
    }
    if (cmd == QLatin1String("set")) {
        bool ok = false;
        const uint v = argv.size() == 3 ? argv[2].toUInt(&ok) : 0;
        if (argv.size() == 3 && argv[1] == QLatin1String("mag") && ok && v <= 1) {
            m_magEnable = v != 0;
            return {QStringLiteral("OK")};
        }
        return {QStringLiteral("ERR usage: set mag <0|1>")};
    }
    if (cmd == QLatin1String("reset")) {
        m_stillFor = 0.0;
        return {QStringLiteral("OK")};
    }
    if (cmd == QLatin1String("reboot") || cmd == QLatin1String("bootsel")) {
        const bool reboot = cmd == QLatin1String("reboot");
        QTimer::singleShot(50, this, [this, reboot] {
            m_available = false;
            const bool wasOpen = m_open;
            close();
            if (wasOpen) emit lost(reboot ? QStringLiteral("board rebooted") : QStringLiteral("board entered BOOTSEL"));
            bootState();
            if (reboot) QTimer::singleShot(m_config.rebootDownMs, this, [this] { m_available = true; });
        });
        return {QStringLiteral("OK")};
    }
    return {QStringLiteral("ERR unknown command; try 'help'")};
}

QStringList FakeBoard::cmdInfo() const {
    const FakeBoardHealth &h = m_health;
    return {
        fmt("# fw=%s version=%s board=%s lib=%s", "ins10-ahrs", qPrintable(m_config.version), qPrintable(m_config.board),
            qPrintable(m_config.library)),
        fmt("# imu=%s clock=%s odr_hz=%.3f measured_hz=%.3f", h.imuOk ? "ok" : "fault",
            h.externalClock ? "external" : "internal", m_config.odrHz,
            h.externalClock ? m_config.odrHz - 0.013 : m_config.odrHz * 0.991),
        fmt("# gyro_dps_per_lsb=%.9g accel_g_per_lsb=%.9g temp_lsb_per_c=%.6g temp_offset_c=%.6g", kGyroDpsPerLsb,
            kAccelGPerLsb, kTempLsbPerC, kTempOffsetC),
        fmt("# mag=%s mag_enabled=%u baro=%s", h.magOk ? "ok" : "fault", m_magEnable ? 1u : 0u, h.baroOk ? "ok" : "fault"),
        fmt("# att_rate_hz=%u raw_div=%u", static_cast<unsigned>(m_attRate), static_cast<unsigned>(m_rawDiv)),
        fmt("# last_reset=%s", h.watchdogReset ? "watchdog" : "normal"),
        fmt("# cal_flags=%u stored=%u thermal_count=%u", calFlags(m_cal), m_stored ? 1u : 0u,
            static_cast<unsigned>(m_thermalCount)),
        QStringLiteral("OK")};
}

QStringList FakeBoard::cmdStatus() const {
    const quint32 flags = statusFlags();
    QString flagLine = fmt("# flags=0x%03X:", flags);
    for (int bit = 0; bit < kStatusFlagCount; ++bit) {
        if (flags & (1u << bit)) flagLine += u' ' + QLatin1String(kStatusFlagWords[bit]);
    }
    const Eigen::Vector4d q = quatNormalized(m_sim->attitude());
    const EulerAngles e = quatToEuler(q);
    const Eigen::Vector3d bias = residualBias();
    const Eigen::Vector3d a = calibratedAccel();
    const Eigen::Vector3d m = calibratedMag();
    const bool magCal = (calFlags(m_cal) & CalMag) != 0;
    const double reference = magCal ? m_sim->sensors().fieldNav.norm() : 0.0;
    const double dip = magCal ? sim::kDipDeg : 0.0;
    const double altitude = 44330.0 * (1.0 - std::pow(m_baroHpa / 1013.25, 0.190295));
    const FakeBoardHealth &h = m_health;
    const quint64 samples = static_cast<quint64>(m_sim->sampleIndex());
    return {
        flagLine,
        fmt("# roll=%.2f pitch=%.2f yaw=%.2f heading=%.2f deg, sigma tilt=%.2f heading=%.2f deg", e.rollDeg, e.pitchDeg,
            e.yawDeg, quatToHeading(q), 0.03, (flags & 0x008) ? 0.21 : 3.5),
        fmt("# gyro_bias=%.4f %.4f %.4f dps (sigma %.4f)", bias[0], bias[1], bias[2], 0.0012),
        fmt("# accel=%.4f %.4f %.4f g (norm %.4f), disturbance %.3f g", a[0], a[1], a[2], a.norm(), still() ? 0.001 : 0.02),
        fmt("# mag=%.2f %.2f %.2f uT (norm %.2f), reference %.2f uT dip %.1f deg, deviation %.2f uT", m[0], m[1], m[2],
            m.norm(), reference, dip, magCal ? std::abs(m.norm() - reference) : 0.0),
        fmt("# temp=%.2f C, rate %.4f C/s", m_sim->temperature(), m_tempRate),
        fmt("# baro=%.3f hPa, %.2f C, altitude %.1f m", m_baroHpa, m_sim->temperature() - 2.0, altitude),
        fmt("# imu: samples=%llu gaps=%llu overflows=%llu bad=%llu bus_errors=%llu restarts=%llu",
            static_cast<unsigned long long>(samples), static_cast<unsigned long long>(h.imuGaps),
            static_cast<unsigned long long>(h.overflows), static_cast<unsigned long long>(h.badPackets),
            static_cast<unsigned long long>(h.busErrors), static_cast<unsigned long long>(h.imuRestarts)),
        fmt("# load: imu+filter %.1f %% of CPU, largest batch %u samples (last 10 s)", h.loadPercent, h.largestBatch),
        fmt("# mag: samples=%llu pairs=%llu errors=%llu restarts=%llu, baro: samples=%llu errors=%llu",
            static_cast<unsigned long long>(m_magSamples), static_cast<unsigned long long>(m_magSamples / 50),
            static_cast<unsigned long long>(h.magErrors), static_cast<unsigned long long>(h.magRestarts),
            static_cast<unsigned long long>(m_baroSamples), static_cast<unsigned long long>(h.baroErrors)),
        QStringLiteral("# filter: watchdog recoveries=0 rejected samples=0 restarts=0"),
        QStringLiteral("OK")};
}

QStringList FakeBoard::cmdThermal() const {
    // The learned model is of the bias left after calibration: c0 + c1 u + c2 u^2 with
    // u = (T - 25) / 10. Three points of the (quadratic) truth fix it.
    const sim::SensorModel &s = m_sim->sensors();
    auto residual = [&](double temp) {
        return Eigen::Vector3d(matrixOf(m_cal, "gyro_matrix") *
                               (s.gyroBias + s.gyroDrift(temp) - vectorOf(m_cal, "gyro_bias") -
                                calDrift(m_cal, QStringLiteral("gyro"), temp)));
    };
    const bool empty = m_thermalCount == 0;
    const Eigen::Vector3d f0 = residual(25.0), fp = residual(35.0), fm = residual(15.0);
    QStringList out = {fmt("# observations=%u range=%.1f..%.1f C", static_cast<unsigned>(m_thermalCount),
                           empty ? 0.0 : m_thermalMin, empty ? 0.0 : m_thermalMax)};
    const char axis[3] = {'x', 'y', 'z'};
    Eigen::Vector3d c1, c2;
    for (int a = 0; a < 3; ++a) {
        const double c0 = empty ? 0.0 : f0[a];
        c1[a] = empty ? 0.0 : 0.5 * (fp[a] - fm[a]);
        c2[a] = empty ? 0.0 : 0.5 * (fp[a] + fm[a]) - f0[a];
        out.append(fmt("# %c: c0=%.5f c1=%.5f c2=%.5f dps", axis[a], c0, c1[a], c2[a]));
    }
    const double u = (m_sim->temperature() - 25.0) / 10.0;
    const Eigen::Vector3d slope = (c1 + 2.0 * c2 * u) / 10.0;
    out.append(fmt("# tempco now: %.5f %.5f %.5f dps/C (sigma %.5f)", slope[0], slope[1], slope[2], empty ? 0.01 : 0.0003));
    out.append(QStringLiteral("OK"));
    return out;
}

QStringList FakeBoard::cmdStream(const QStringList &argv) {
    auto parseUint = [](const QString &text, uint max, uint &value) {
        bool ok = false;
        const qulonglong v = text.toULongLong(&ok, 10);
        if (!ok || v > max) return false;
        value = static_cast<uint>(v);
        return true;
    };
    uint value = 0;
    if (argv.size() == 2 && argv[1] == QLatin1String("off")) {
        m_attRate = 0;
        m_rawDiv = 0;
        m_magStream = m_baroStream = false;
    } else if (argv.size() == 3 && argv[1] == QLatin1String("att") && parseUint(argv[2], 200, value)) {
        m_attRate = static_cast<int>(value);
        m_nextAtt = m_time;
    } else if (argv.size() == 3 && argv[1] == QLatin1String("raw") && parseUint(argv[2], 1024, value)) {
        m_rawDiv = static_cast<int>(value);
    } else if (argv.size() == 3 && argv[1] == QLatin1String("mag") && parseUint(argv[2], 1, value)) {
        m_magStream = value != 0;
    } else if (argv.size() == 3 && argv[1] == QLatin1String("baro") && parseUint(argv[2], 1, value)) {
        m_baroStream = value != 0;
    } else {
        return {QStringLiteral("ERR usage: stream att <hz>|raw <n>|mag <0|1>|baro <0|1>|off")};
    }
    return {QStringLiteral("OK")};
}

QStringList FakeBoard::cmdCal(const QStringList &argv) {
    const QString usage = QStringLiteral("ERR usage: cal show|set|save|load|erase");
    if (argv.size() < 2) return {usage};
    const QString sub = argv[1];
    if (sub == QLatin1String("show")) {
        QStringList out;
        for (const CalFieldDef &f : calibrationFields()) {
            const QList<double> values = m_cal.value(QLatin1String(f.name));
            QString line = QStringLiteral("cal set ") + QLatin1String(f.name);
            for (double v : values) line += u' ' + fmt("%.9g", static_cast<double>(toFloat(v)));
            out.append(line);
        }
        out.append(QStringLiteral("OK"));
        return out;
    }
    if (sub == QLatin1String("set")) {
        const qsizetype count = argv.size() - 3;
        if (count < 1 || count > 9) return {QStringLiteral("ERR usage: cal set <name> <values>")};
        QList<double> values;
        for (qsizetype i = 0; i < count; ++i) {
            bool ok = false;
            const float v = argv[3 + i].toFloat(&ok);
            if (!ok) return {QStringLiteral("ERR not a number")};
            values.append(static_cast<double>(v));
        }
        const QString name = argv[2];
        const CalFieldDef *def = findCalibrationField(name);
        bool valid = def && def->count == count;
        for (double v : values) valid = valid && std::isfinite(v);
        if (valid && name == QLatin1String("flags")) valid = values.front() >= 0.0 && values.front() < 256.0;
        if (!valid) return {QStringLiteral("ERR unknown field or wrong number of values")};
        if (name == QLatin1String("flags")) values = {std::floor(values.front())};
        m_cal.insert(name, asFloats(values));
        return {QStringLiteral("OK")};
    }
    if (sub == QLatin1String("save")) {
        Stored s;
        s.cal = m_cal;
        s.magEnable = m_magEnable;
        s.attRate = m_attRate; // app_save(): the current $ATT rate becomes the power-up rate
        s.thermalCount = m_thermalCount;
        m_stored = s;
        ++m_saveCount;
        return {QStringLiteral("OK")};
    }
    if (sub == QLatin1String("load")) {
        if (!m_stored) return {QStringLiteral("ERR nothing stored")};
        m_cal = m_stored->cal;
        m_magEnable = m_stored->magEnable;
        return {QStringLiteral("OK")};
    }
    if (sub == QLatin1String("erase")) {
        m_stored.reset();
        m_cal = defaultCalibration();
        m_magEnable = true;
        m_thermalCount = 0;
        m_stillFor = 0.0;
        return {QStringLiteral("OK")};
    }
    return {usage};
}

// ---- Output ------------------------------------------------------------------------

void FakeBoard::reply(const QStringList &lines) {
    if (m_replyDelayMs <= 0) {
        for (const QString &line : lines) output(line);
        return;
    }
    QTimer::singleShot(m_replyDelayMs, this, [this, lines] {
        for (const QString &line : lines) output(line);
    });
}

QString FakeBoard::damaged(const QString &line) {
    std::uniform_int_distribution<int> mode(0, 2);
    const qsizetype n = line.size();
    if (n < 4) return line;
    std::uniform_int_distribution<qsizetype> pos(1, n - 2);
    const qsizetype a = pos(m_damageRng);
    switch (mode(m_damageRng)) {
    case 0: return line.left(a) + QChar(u'\x01'); // cut; the marker drops the terminator below
    case 1: return line.mid(a);
    default: {
        const qsizetype b = std::min(n - 1, a + 1 + pos(m_damageRng) % 8);
        return line.left(a) + line.mid(b);
    }
    }
}

void FakeBoard::output(const QString &line) {
    // USB CDC output only reaches a terminal that holds DTR (stdio_usb_connected()).
    if (!m_open) return;
    QString text = line;
    bool terminate = true;
    if (m_damageProbability > 0.0 && std::uniform_real_distribution<double>(0.0, 1.0)(m_damageRng) < m_damageProbability) {
        text = damaged(line);
        if (text.endsWith(QChar(u'\x01'))) {
            text.chop(1);
            terminate = false;
        }
    }
    m_tx += text.toLatin1();
    if (terminate) m_tx += "\r\n"; // stdio_usb translates '\n' to CR LF
    ++m_linesSent;
    scheduleDelivery();
}

void FakeBoard::scheduleDelivery() {
    if (m_deliverPending) return;
    m_deliverPending = true;
    QTimer::singleShot(0, this, &FakeBoard::deliver);
}

void FakeBoard::deliver() {
    m_deliverPending = false;
    while (m_open && !m_tx.isEmpty()) {
        const QByteArray chunk = m_tx.left(16384);
        m_tx.remove(0, chunk.size());
        emit dataReceived(chunk);
    }
    m_tx.clear();
}

// ---- Factories ---------------------------------------------------------------------

DefaultTransportFactory::DefaultTransportFactory(const FakeBoardConfig &demo) : m_demoConfig(demo) {}

FakeBoardConfig DefaultTransportFactory::demoConfig() {
    FakeBoardConfig c;
    c.realTime = true;
    c.stillT0C = 18.0;
    c.stillT1C = 42.0;
    c.stillTauS = 500.0;
    c.tumbleRateDps = 45.0;
    c.positions = 16;
    return c;
}

DefaultTransportFactory::~DefaultTransportFactory() = default;

Transport *DefaultTransportFactory::acquire(const QString &portName) {
    if (portName == kDemoPortName) {
        if (!m_demo) m_demo = std::make_unique<FakeBoard>(m_demoConfig);
        return m_demo.get();
    }
    return new SerialTransport(portName);
}

void DefaultTransportFactory::release(Transport *transport) {
    if (transport && transport != m_demo.get()) transport->deleteLater();
}

bool DefaultTransportFactory::isAvailable(const QString &portName) {
    if (portName == kDemoPortName) return !m_demo || m_demo->isAvailable();
    for (const QSerialPortInfo &info : QSerialPortInfo::availablePorts()) {
        if (info.portName() == portName) return true;
    }
    return false;
}

} // namespace ins10
