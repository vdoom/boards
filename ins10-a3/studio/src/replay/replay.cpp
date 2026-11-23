#include "replay/replay.h"

extern "C" {
#include "ins10/ins10.h"
}

#include <QByteArray>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>

namespace ins10 {

namespace {

// Scales of the ICM-42688-P high-resolution FIFO format, as in replay.c.
constexpr double kGyroLsbPerDps = 524288.0 / 2000.0;
constexpr double kAccelLsbPerG = 524288.0 / 16.0;
constexpr double kTempLsbPerC = 132.48;

// parse_floats() of replay.c: tokens split on space, comma, tab, CR, LF; atof.
int parseFloats(char *text, float *values, int max) {
    int n = 0;
    char *save = nullptr;
    (void)save;
    for (char *tok = std::strtok(text, " ,\t\r\n"); tok && n < max; tok = std::strtok(nullptr, " ,\t\r\n")) {
        values[n++] = static_cast<float>(std::atof(tok));
    }
    return n;
}

// apply_cal_line() of replay.c.
bool applyCalLine(ins10_calibration *cal, const QByteArray &line) {
    char name[32] = {};
    int used = 0;
    if (std::sscanf(line.constData(), "cal set %31s %n", name, &used) != 1) return true;
    QByteArray rest = line.mid(used);
    float values[9];
    const int n = parseFloats(rest.data(), values, 9);
    return ins10_cal_set_field(cal, name, values, n);
}

} // namespace

ReplayResult replayLog(const QStringList &lines, const ReplayOptions &options) {
    ReplayResult out;
    ins10_calibration cal;
    ins10_cal_defaults(&cal);
    for (const QString &text : options.calibration) {
        const QByteArray line = text.toLatin1();
        if (!applyCalLine(&cal, line)) out.ignoredCalibration.append(text);
    }
    ins10_fusion_config cfg;
    ins10_fusion_default_config(&cfg);
    cfg.sample_rate_hz = static_cast<float>(options.rateHz);
    // The filter state is large; keep it off the stack.
    auto fusion = std::make_unique<ins10_fusion>();
    ins10_fusion_init(fusion.get(), &cfg, &cal, nullptr);

    double t = 0.0;
    long expectedSeq = -1;
    for (const QString &text : lines) {
        if (options.cancel && options.cancel->load(std::memory_order_relaxed)) {
            out.cancelled = true;
            break;
        }
        QByteArray line = text.toLatin1();
        line.append('\n'); // replay.c reads lines with fgets, terminator included
        if (line.startsWith("cal set ")) {
            if (!applyCalLine(&cal, line)) out.ignoredCalibration.append(text);
            ins10_fusion_set_calibration(fusion.get(), &cal);
        } else if (line.startsWith("$RAW,")) {
            long seq, n, v[7];
            if (std::sscanf(line.constData() + 5, "%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld", &seq, &n, &v[0], &v[1], &v[2],
                            &v[3], &v[4], &v[5], &v[6]) != 9 ||
                n <= 0)
                continue;
            if (expectedSeq >= 0 && seq != expectedSeq) out.lostSamples += seq - expectedSeq;
            expectedSeq = seq + n;
            float gyro[3], accel[3];
            for (int i = 0; i < 3; ++i) {
                gyro[i] = static_cast<float>(static_cast<double>(v[i]) / static_cast<double>(n) / kGyroLsbPerDps);
                accel[i] = static_cast<float>(static_cast<double>(v[i + 3]) / static_cast<double>(n) / kAccelLsbPerG);
            }
            const float temp = static_cast<float>(static_cast<double>(v[6]) / static_cast<double>(n) / kTempLsbPerC + 25.0);
            const float dt = static_cast<float>(static_cast<double>(n) / options.rateHz);
            ins10_fusion_imu(fusion.get(), gyro, accel, temp, dt, false);
            t += static_cast<double>(dt);
            ++out.imuRecords;
            ins10_output o;
            ins10_fusion_get(fusion.get(), &o);
            if (!(o.flags & INS10_F_ALIGNED)) continue;
            ReplayRow row;
            row.t = t;
            row.q = {o.q[0], o.q[1], o.q[2], o.q[3]};
            row.rollDeg = o.roll_deg;
            row.pitchDeg = o.pitch_deg;
            row.yawDeg = o.yaw_deg;
            row.headingDeg = o.heading_deg;
            row.biasDps = {o.gyro_bias_dps[0], o.gyro_bias_dps[1], o.gyro_bias_dps[2]};
            row.tiltSigmaDeg = o.tilt_sigma_deg;
            row.headingSigmaDeg = o.heading_sigma_deg;
            row.flags = o.flags;
            out.rows.push_back(row);
        } else if (line.startsWith("$MAG,")) {
            QByteArray rest = line.mid(5);
            float v[4];
            if (parseFloats(rest.data(), v, 4) == 4) ins10_fusion_mag(fusion.get(), v + 1, 0.0f);
        }
    }
    out.seconds = t;
    return out;
}

QStringList replayCsv(const ReplayResult &result) {
    QStringList out;
    out.append(QStringLiteral("t_s,qw,qx,qy,qz,roll_deg,pitch_deg,yaw_deg,heading_deg,"
                              "bias_x_dps,bias_y_dps,bias_z_dps,tilt_sigma_deg,heading_sigma_deg,flags"));
    out.reserve(static_cast<qsizetype>(result.rows.size()) + 1);
    for (const ReplayRow &r : result.rows) {
        out.append(QString::asprintf("%.4f,%.6f,%.6f,%.6f,%.6f,%.3f,%.3f,%.3f,%.3f,%.5f,%.5f,%.5f,%.3f,%.3f,0x%03X", r.t,
                                     r.q[0], r.q[1], r.q[2], r.q[3], r.rollDeg, r.pitchDeg, r.yawDeg, r.headingDeg,
                                     r.biasDps[0], r.biasDps[1], r.biasDps[2], r.tiltSigmaDeg, r.headingSigmaDeg, r.flags));
    }
    return out;
}

} // namespace ins10
