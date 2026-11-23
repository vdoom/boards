#include "core/health.h"

#include <QCoreApplication>

#include <algorithm>
#include <cmath>

namespace ins10 {

namespace {
struct Text {
    Q_DECLARE_TR_FUNCTIONS(ins10::FirstRun)
};

QString num(double v, int decimals) { return QString::number(v, 'f', decimals); }

// How long the filter may go without reporting 'still' before waiting for it stops
// being the right advice.
constexpr double kStillPatienceS = 15.0;

// Seconds since the filter last reported 'still' (flag 0x004), by the status history;
// the whole span of the history when it never did.
double secondsWithoutStill(const StatusHistory &history) {
    const auto &samples = history.samples();
    if (samples.empty()) return 0.0;
    double since = samples.front().t;
    for (auto it = samples.rbegin(); it != samples.rend(); ++it) {
        if (it->report.flags.valid && (it->report.flags.value & 0x004)) {
            since = it->t;
            break;
        }
    }
    return samples.back().t - since;
}

// Seen on the real board: lying untouched, the filter reported 'still' for 0.3 s out
// of 15. Waiting longer does not help then, and the user needs to know what to do.
QString neverStillAdvice(double seconds) {
    return Text::tr("The filter has not reported 'still' for %1 s. Either something disturbs the board (knocks on the "
                    "table, a pulled cable, a hand), or the firmware's stillness thresholds are too tight for this "
                    "board. To tell which, put the board on a solid surface and record 30-60 s of raw data on the "
                    "Recording page (raw 1).")
        .arg(num(seconds, 0));
}
} // namespace

void StatusHistory::add(double t, const StatusReport &report) {
    m_samples.push_back({t, report});
    while (m_samples.size() > m_capacity) m_samples.pop_front();
}

std::optional<std::pair<const StatusSample *, const StatusSample *>>
StatusHistory::window(const Pick &pick, double seconds) const {
    if (m_samples.size() < 2) return std::nullopt;
    const StatusSample *last = &m_samples.back();
    if (!pick(last->report)) return std::nullopt;
    const StatusSample *first = nullptr;
    for (auto it = m_samples.rbegin(); it != m_samples.rend(); ++it) {
        if (last->t - it->t > seconds) break;
        if (pick(it->report)) first = &*it;
    }
    if (!first || first == last || last->t - first->t < 0.5 * seconds) return std::nullopt;
    return std::make_pair(first, last);
}

std::optional<double> StatusHistory::growth(const Pick &pick, double seconds) const {
    const auto w = window(pick, seconds);
    if (!w) return std::nullopt;
    const double d = *pick(w->second->report) - *pick(w->first->report);
    if (d < 0.0) return std::nullopt;
    return d;
}

std::optional<double> StatusHistory::rate(const Pick &pick, double seconds) const {
    const auto w = window(pick, seconds);
    if (!w) return std::nullopt;
    const double d = *pick(w->second->report) - *pick(w->first->report);
    if (d < 0.0) return std::nullopt;
    return d / (w->second->t - w->first->t);
}

QList<CheckResult> evaluateFirstRun(const InfoReport &info, const StatusHistory &history) {
    QList<CheckResult> out;
    const bool haveInfo = info.isAhrs();
    const StatusSample *latest = history.latest();
    const StatusReport *status = latest ? &latest->report : nullptr;
    auto add = [&](const QString &id, const QString &title) -> CheckResult & {
        out.append(CheckResult{id, title, CheckLevel::Pending, QString(), QString()});
        return out.back();
    };
    const QString waitingInfo = Text::tr("Waiting for 'info' from the board.");
    const QString waitingStatus = Text::tr("Waiting for 'status' from the board.");

    // README item 2: imu=ok clock=external odr_hz=512.000, mag=ok, baro=ok.
    {
        CheckResult &c = add(QStringLiteral("imu"), Text::tr("IMU running at 512 Hz"));
        if (!haveInfo) {
            c.detail = waitingInfo;
        } else {
            const double odr = info.odrHz();
            c.detail = QStringLiteral("imu=%1 odr_hz=%2 measured_hz=%3")
                           .arg(info.text(QStringLiteral("imu")), info.text(QStringLiteral("odr_hz")),
                                info.text(QStringLiteral("measured_hz")));
            if (info.text(QStringLiteral("imu")) != QLatin1String("ok")) {
                c.level = CheckLevel::Fail;
                c.advice = Text::tr("The ICM-42688-P did not start. Run the bench test (ins10_test) and check the "
                                    "SPI wiring and the IMU supply.");
            } else if (!(std::abs(odr - 512.0) < 0.5)) {
                c.level = CheckLevel::Warning;
                c.advice = Text::tr("The output data rate should be 512.000 Hz; the driver configures it, so another "
                                    "value means the configuration did not take.");
            } else {
                c.level = CheckLevel::Ok;
            }
        }
    }
    {
        CheckResult &c = add(QStringLiteral("clock"), Text::tr("IMU clock from the 32.768 kHz oscillator"));
        if (!haveInfo) {
            c.detail = waitingInfo;
        } else {
            c.detail = QStringLiteral("clock=%1").arg(info.clock());
            if (info.clock() == QLatin1String("external")) {
                c.level = CheckLevel::Ok;
            } else {
                c.level = CheckLevel::Warning;
                c.advice = Text::tr("clock=internal: the 32.768 kHz clock (CLKIN) was not picked up. The filter works, "
                                    "but on the measured rate. Check the oscillator and the CLKIN pin.");
            }
        }
    }
    {
        CheckResult &c = add(QStringLiteral("sensors"), Text::tr("Magnetometer and barometer found"));
        if (!haveInfo) {
            c.detail = waitingInfo;
        } else {
            const QString mag = info.text(QStringLiteral("mag")), baro = info.text(QStringLiteral("baro"));
            c.detail = QStringLiteral("mag=%1 mag_enabled=%2 baro=%3").arg(mag, info.text(QStringLiteral("mag_enabled")), baro);
            if (mag != QLatin1String("ok") || baro != QLatin1String("ok")) {
                c.level = CheckLevel::Fail;
                c.advice = Text::tr("A sensor did not answer. The bench test (ins10_test) checks each chip on its own.");
            } else if (info.text(QStringLiteral("mag_enabled")) == QLatin1String("0")) {
                c.level = CheckLevel::Warning;
                c.advice = Text::tr("The magnetometer is switched off ('set mag 0'); heading is not referenced to north.");
            } else {
                c.level = CheckLevel::Ok;
            }
        }
    }

    // README item 3: at rest, |a| 0.98..1.02 g (the 20-bit FIFO scale), |gyro_bias| < 1 dps.
    const bool still = status && status->flags.valid && (status->flags.value & 0x004);
    const bool aligned = status && status->flags.valid && (status->flags.value & 0x001);
    const double restless = still ? 0.0 : secondsWithoutStill(history);
    const bool neverStill = aligned && !still && restless >= kStillPatienceS;
    {
        CheckResult &c = add(QStringLiteral("accel"), Text::tr("Accelerometer reads 1 g at rest"));
        if (!status || !status->accel.valid) {
            c.detail = waitingStatus;
        } else {
            const double norm = status->accel.norm;
            c.detail = Text::tr("accel %1 %2 %3 g, norm %4 g")
                           .arg(num(status->accel.g[0], 4), num(status->accel.g[1], 4), num(status->accel.g[2], 4),
                                num(norm, 4));
            if ((norm > 0.2 && norm < 0.3) || (norm > 3.5 && norm < 4.5)) {
                c.level = CheckLevel::Fail;
                c.advice = Text::tr("The norm is off by a factor of four: the scale of the 20-bit FIFO format in "
                                    "drivers/icm42688.h is wrong.");
            } else if (neverStill) {
                // Without 'still' the reading may include motion, so the norm is not
                // judged; but the user is told why the check does not complete.
                c.level = CheckLevel::Warning;
                c.advice = neverStillAdvice(restless);
            } else if (!still) {
                c.level = CheckLevel::Pending;
                c.advice = Text::tr("Leave the board still, components up, for a few seconds.");
            } else if (norm >= 0.98 && norm <= 1.02) {
                c.level = CheckLevel::Ok;
            } else {
                c.level = norm > 0.9 && norm < 1.1 ? CheckLevel::Warning : CheckLevel::Fail;
                c.advice = Text::tr("At rest the norm should be within 0.98-1.02 g. A small error goes away with the "
                                    "accelerometer calibration; a large one points at the driver's scale.");
            }
        }
    }
    {
        CheckResult &c = add(QStringLiteral("gyro"), Text::tr("Gyro bias below 1 deg/s, filter aligned and still"));
        if (!status || !status->gyroBias.valid || !status->flags.valid) {
            c.detail = waitingStatus;
        } else {
            const auto &b = status->gyroBias.dps;
            const double largest = std::max({std::abs(b[0]), std::abs(b[1]), std::abs(b[2])});
            c.detail = Text::tr("gyro_bias %1 %2 %3 dps; flags %4")
                           .arg(num(b[0], 4), num(b[1], 4), num(b[2], 4), status->flags.names.join(u' '));
            if (neverStill) {
                c.level = CheckLevel::Warning;
                c.advice = neverStillAdvice(restless);
            } else if (!aligned || !still) {
                c.level = CheckLevel::Pending;
                c.advice = Text::tr("Waiting for the flags 'aligned still': leave the board untouched.");
            } else if (largest < 1.0) {
                c.level = CheckLevel::Ok;
            } else {
                c.level = CheckLevel::Warning;
                c.advice = Text::tr("A bias above 1 deg/s is unusual for the ICM-42688-P: was the board really still, "
                                    "or is the gyro scale wrong?");
            }
        }
    }

    // README item 4: gaps=0 overflows=0 bad=0 bus_errors=0; load and largest batch.
    {
        CheckResult &c = add(QStringLiteral("imu_counters"), Text::tr("No IMU gaps, overflows, bad packets or bus errors"));
        if (!status || !status->imu.valid) {
            c.detail = waitingStatus;
        } else {
            const auto &i = status->imu;
            c.detail = QStringLiteral("gaps=%1 overflows=%2 bad=%3 bus_errors=%4 restarts=%5")
                           .arg(i.gaps).arg(i.overflows).arg(i.bad).arg(i.busErrors).arg(i.restarts);
            const auto sum = [](const StatusReport &s) -> std::optional<double> {
                if (!s.imu.valid) return std::nullopt;
                return static_cast<double>(s.imu.gaps + s.imu.overflows + s.imu.bad + s.imu.busErrors + s.imu.restarts);
            };
            const auto grew = history.growth(sum, 10.0);
            if (grew && *grew > 0) {
                c.level = CheckLevel::Fail;
                c.advice = Text::tr("The counters are growing: the FIFO is not read in time, or the SPI link is "
                                    "unreliable (wiring length, clock speed).");
            } else if (*sum(*status) > 0) {
                c.level = grew ? CheckLevel::Warning : CheckLevel::Pending;
                c.advice = Text::tr("Not zero, but not growing either: something happened earlier (at power-up?). "
                                    "Watch whether they grow.");
            } else {
                c.level = CheckLevel::Ok;
            }
        }
    }
    {
        CheckResult &c = add(QStringLiteral("load"), Text::tr("Processor keeps up (load, largest batch)"));
        if (!status || !status->load.valid) {
            c.detail = waitingStatus;
        } else {
            c.detail = Text::tr("imu+filter %1 % of CPU, largest batch %2 samples")
                           .arg(num(status->load.cpuPercent, 1))
                           .arg(status->load.largestBatch);
            int bigBatches = 0, considered = 0;
            for (auto it = history.samples().rbegin(); it != history.samples().rend() && considered < 5; ++it) {
                if (!it->report.load.valid) continue;
                ++considered;
                bigBatches += it->report.load.largestBatch > 10;
            }
            if (considered >= 5 && bigBatches == considered) {
                c.level = CheckLevel::Fail;
                c.advice = Text::tr("The largest batch stays above 10 samples: the main loop does not keep up with "
                                    "the IMU.");
            } else if (status->load.cpuPercent > 60.0) {
                c.level = CheckLevel::Fail;
                c.advice = Text::tr("The filter takes most of the processor; on the RP2040 (no FPU) 10-25 % is "
                                    "expected.");
            } else if (status->load.cpuPercent > 30.0 || status->load.largestBatch > 10) {
                c.level = CheckLevel::Warning;
                c.advice = Text::tr("Higher than the 10-25 % expected on a Pico 1; little headroom is left.");
            } else {
                c.level = CheckLevel::Ok;
            }
        }
    }

    // README item 5: mag samples grow by about 50/s, pairs by about 1/s, errors do not.
    {
        CheckResult &c = add(QStringLiteral("mag_rate"), Text::tr("Magnetometer: ~50 samples/s, ~1 SET/RESET pair/s"));
        const auto samples = history.rate(
            [](const StatusReport &s) -> std::optional<double> {
                if (!s.magCounters.valid) return std::nullopt;
                return static_cast<double>(s.magCounters.samples);
            },
            10.0);
        const auto pairs = history.rate(
            [](const StatusReport &s) -> std::optional<double> {
                if (!s.magCounters.valid) return std::nullopt;
                return static_cast<double>(s.magCounters.pairs);
            },
            10.0);
        const auto errors = history.growth(
            [](const StatusReport &s) -> std::optional<double> {
                if (!s.magCounters.valid) return std::nullopt;
                return static_cast<double>(s.magCounters.errors);
            },
            10.0);
        if (!samples || !pairs || !errors) {
            c.detail = Text::tr("Measuring the rates over several seconds...");
        } else {
            c.detail = Text::tr("%1 samples/s, %2 pairs/s, errors +%3")
                           .arg(num(*samples, 1), num(*pairs, 2))
                           .arg(*errors, 0, 'f', 0);
            if (*errors > 0) {
                c.level = CheckLevel::Fail;
                c.advice = Text::tr("Magnetometer errors are growing: check its SPI chip select and supply.");
            } else if (*samples < 1.0) {
                c.level = CheckLevel::Fail;
                c.advice = Text::tr("No magnetometer samples arrive.");
            } else if (*samples < 40.0 || *samples > 60.0 || *pairs < 0.5 || *pairs > 2.0) {
                c.level = CheckLevel::Warning;
                c.advice = Text::tr("The rates differ from the expected ~50/s and ~1/s: the measurement state "
                                    "machine is not running as designed.");
            } else {
                c.level = CheckLevel::Ok;
            }
        }
    }

    // README item 7: last_reset, stored=1 after 'cal save' and 'reboot'.
    {
        CheckResult &c = add(QStringLiteral("reset"), Text::tr("Last restart was normal (no watchdog)"));
        if (!haveInfo) {
            c.detail = waitingInfo;
        } else {
            c.detail = QStringLiteral("last_reset=%1").arg(info.text(QStringLiteral("last_reset")));
            if (info.text(QStringLiteral("last_reset")) == QLatin1String("watchdog")) {
                c.level = CheckLevel::Warning;
                c.advice = Text::tr("The watchdog restarted the board: the main loop hung. If this repeats, save a "
                                    "log of what was happening.");
            } else {
                c.level = CheckLevel::Ok;
            }
        }
    }
    {
        CheckResult &c = add(QStringLiteral("stored"), Text::tr("Calibration stored in flash"));
        if (!haveInfo) {
            c.detail = waitingInfo;
        } else {
            c.detail = QStringLiteral("stored=%1 cal_flags=%2").arg(info.text(QStringLiteral("stored")),
                                                                   info.text(QStringLiteral("cal_flags")));
            if (info.text(QStringLiteral("stored")) == QLatin1String("1")) {
                c.level = CheckLevel::Ok;
            } else {
                c.level = CheckLevel::Warning;
                c.advice = Text::tr("Nothing is stored yet. After 'Save to flash' and a reboot, 'info' should show "
                                    "stored=1; if it does not, writing the flash failed.");
            }
        }
    }
    return out;
}

} // namespace ins10
