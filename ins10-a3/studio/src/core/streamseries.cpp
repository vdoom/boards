#include "core/streamseries.h"

#include <QCoreApplication>

#include <algorithm>
#include <cmath>

namespace ins10 {

namespace {
struct Text {
    Q_DECLARE_TR_FUNCTIONS(ins10::StreamSeries)
};
} // namespace

namespace {
QStringList attNames() {
    return {Text::tr("roll"),   Text::tr("pitch"),  Text::tr("yaw"),        Text::tr("heading"),       Text::tr("bias x"),
            Text::tr("bias y"), Text::tr("bias z"), Text::tr("tilt sigma"), Text::tr("heading sigma"), Text::tr("temperature")};
}
QStringList rawNames() {
    return {Text::tr("gyro x"),  Text::tr("gyro y"),  Text::tr("gyro z"),     Text::tr("accel x"),
            Text::tr("accel y"), Text::tr("accel z"), Text::tr("temperature")};
}
QStringList magNames() { return {Text::tr("mag x"), Text::tr("mag y"), Text::tr("mag z"), Text::tr("|B|")}; }
QStringList baroNames() { return {Text::tr("altitude"), Text::tr("pressure"), Text::tr("temperature")}; }
} // namespace

StreamSeries::StreamSeries()
    : att(attNames(), kAttCapacity), raw(rawNames(), kRawCapacity), mag(magNames(), kMagCapacity),
      baro(baroNames(), kBaroCapacity) {}

void StreamSeries::retranslate() {
    att.setNames(attNames());
    raw.setNames(rawNames());
    mag.setNames(magNames());
    baro.setNames(baroNames());
}

void StreamSeries::setInfo(const InfoReport &info) {
    for (const auto &[key, value] : info.values.asKeyValueRange()) m_scales.update(key, value);
    m_clock.setOdr(m_scales.odrHz);
}

void StreamSeries::clear() {
    att.clear();
    raw.clear();
    mag.clear();
    baro.clear();
    m_clock.reset();
}

void StreamSeries::feed(const ParsedLine &line) {
    switch (line.kind) {
    case LineKind::Att: {
        const AttRecord &r = line.as<AttRecord>();
        m_clock.noteBoardTime(r.tMs);
        att.append(static_cast<double>(r.tMs) * 1e-3, {r.rollDeg, r.pitchDeg, r.yawDeg, r.headingDeg, r.biasDps[0],
                                                       r.biasDps[1], r.biasDps[2], r.tiltSigmaDeg, r.headingSigmaDeg, r.tempC});
        break;
    }
    case LineKind::Raw: {
        const RawRecord &r = line.as<RawRecord>();
        const double n = r.n;
        const Scales &s = m_scales;
        raw.append(m_clock.rawTime(r),
                   {static_cast<double>(r.gyro[0]) / n * s.gyroDpsPerLsb, static_cast<double>(r.gyro[1]) / n * s.gyroDpsPerLsb,
                    static_cast<double>(r.gyro[2]) / n * s.gyroDpsPerLsb, static_cast<double>(r.accel[0]) / n * s.accelGPerLsb,
                    static_cast<double>(r.accel[1]) / n * s.accelGPerLsb, static_cast<double>(r.accel[2]) / n * s.accelGPerLsb,
                    static_cast<double>(r.temp) / n / s.tempLsbPerC + s.tempOffsetC});
        break;
    }
    case LineKind::Mag: {
        const MagRecord &r = line.as<MagRecord>();
        m_clock.noteBoardTime(r.tMs);
        const double norm = std::sqrt(r.ut[0] * r.ut[0] + r.ut[1] * r.ut[1] + r.ut[2] * r.ut[2]);
        mag.append(static_cast<double>(r.tMs) * 1e-3, {r.ut[0], r.ut[1], r.ut[2], norm});
        break;
    }
    case LineKind::Baro: {
        const BaroRecord &r = line.as<BaroRecord>();
        m_clock.noteBoardTime(r.tMs);
        baro.append(static_cast<double>(r.tMs) * 1e-3, {r.altitudeM, r.pressureHpa, r.tempC});
        break;
    }
    default:
        break;
    }
}

double StreamSeries::latestTime() const {
    double t = 0.0;
    for (const TimeSeriesGroup *g : {&att, &raw, &mag, &baro}) {
        if (!g->isEmpty()) t = std::max(t, g->lastTime());
    }
    return t;
}

size_t StreamSeries::memoryBytes() const {
    return att.memoryBytes() + raw.memoryBytes() + mag.memoryBytes() + baro.memoryBytes();
}

} // namespace ins10
