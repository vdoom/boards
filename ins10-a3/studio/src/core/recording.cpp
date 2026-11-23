#include "core/recording.h"

#include <algorithm>
#include <cmath>

namespace ins10 {

bool Scales::update(const QString &key, const QString &value) {
    double *target = nullptr;
    if (key == QLatin1String("gyro_dps_per_lsb")) target = &gyroDpsPerLsb;
    else if (key == QLatin1String("accel_g_per_lsb")) target = &accelGPerLsb;
    else if (key == QLatin1String("temp_lsb_per_c")) target = &tempLsbPerC;
    else if (key == QLatin1String("temp_offset_c")) target = &tempOffsetC;
    else if (key == QLatin1String("odr_hz")) target = &odrHz;
    if (!target) return false;
    bool ok = false;
    const double v = value.toDouble(&ok);
    if (!ok || !std::isfinite(v)) return false;
    // A scale or rate of zero would turn every row into infinities.
    if (target != &tempOffsetC && !(v > 0.0)) return false;
    *target = v;
    return true;
}

Recording::Fed Recording::feed(QStringView line) { return feed(parseLine(line.trimmed())); }

Recording::Fed Recording::feed(const ParsedLine &line) {
    switch (line.kind) {
    case LineKind::Raw: {
        const RawRecord &r = line.as<RawRecord>();
        if (!m_seq.empty()) {
            const qint64 expected = m_seq.back() + m_n.back();
            if (r.seq > expected) m_lostSamples += r.seq - expected;
            if (r.seq != expected) ++m_breaks;
        }
        const Scales &s = m_scales;
        const double n = r.n;
        m_seq.push_back(r.seq);
        m_n.push_back(r.n);
        // Same operation order as records.py: value / n * scale.
        m_gyro.emplace_back(static_cast<double>(r.gyro[0]) / n * s.gyroDpsPerLsb,
                            static_cast<double>(r.gyro[1]) / n * s.gyroDpsPerLsb,
                            static_cast<double>(r.gyro[2]) / n * s.gyroDpsPerLsb);
        m_accel.emplace_back(static_cast<double>(r.accel[0]) / n * s.accelGPerLsb,
                             static_cast<double>(r.accel[1]) / n * s.accelGPerLsb,
                             static_cast<double>(r.accel[2]) / n * s.accelGPerLsb);
        m_temp.push_back(static_cast<double>(r.temp) / n / s.tempLsbPerC + s.tempOffsetC);
        return Fed::Raw;
    }
    case LineKind::Mag: {
        const MagRecord &r = line.as<MagRecord>();
        m_mag.emplace_back(r.ut[0], r.ut[1], r.ut[2]);
        m_magImuIndex.push_back(imuRows());
        return Fed::Mag;
    }
    case LineKind::Info:
        for (const auto &[key, value] : line.as<InfoLine>().pairs) m_scales.update(key, value);
        return Fed::Info;
    case LineKind::Banner:
        return Fed::Info;
    case LineKind::Cal: {
        const CalLine &cal = line.as<CalLine>();
        m_calibration.insert(cal.name, cal.values);
        return Fed::Cal;
    }
    default:
        return Fed::Nothing;
    }
}

void Recording::clear() { *this = Recording(); }

ImuArrays Recording::imuArrays() const { return imuArrays(0, imuRows()); }

ImuArrays Recording::imuArrays(qsizetype first, qsizetype count) const {
    first = std::clamp<qsizetype>(first, 0, imuRows());
    count = std::clamp<qsizetype>(count, 0, imuRows() - first);
    ImuArrays a;
    a.t.resize(count);
    a.dt.resize(count);
    a.gyro.resize(count, 3);
    a.accel.resize(count, 3);
    a.temp.resize(count);
    const double odr = m_scales.odrHz;
    for (qsizetype k = 0; k < count; ++k) {
        const size_t i = static_cast<size_t>(first + k);
        const double seq = static_cast<double>(m_seq[i]);
        const double n = static_cast<double>(m_n[i]);
        a.t[k] = (seq + 0.5 * n) / odr;
        a.dt[k] = n / odr;
        a.gyro.row(k) = m_gyro[i].transpose();
        a.accel.row(k) = m_accel[i].transpose();
        a.temp[k] = m_temp[i];
    }
    return a;
}

MagWithImu Recording::magWithImu() const {
    std::vector<size_t> rows;
    rows.reserve(m_mag.size());
    for (size_t k = 0; k < m_mag.size(); ++k) {
        const qsizetype index = m_magImuIndex[k];
        if (index >= 1 && index < imuRows()) rows.push_back(k);
    }
    MagWithImu out;
    const Eigen::Index n = static_cast<Eigen::Index>(rows.size());
    out.mag.resize(n, 3);
    out.accel.resize(n, 3);
    out.gyro.resize(n, 3);
    for (Eigen::Index r = 0; r < n; ++r) {
        const size_t k = rows[static_cast<size_t>(r)];
        const size_t i = static_cast<size_t>(m_magImuIndex[k]);
        out.mag.row(r) = m_mag[k].transpose();
        out.accel.row(r) = (0.5 * (m_accel[i - 1] + m_accel[i])).transpose();
        out.gyro.row(r) = (0.5 * (m_gyro[i - 1] + m_gyro[i])).transpose();
    }
    return out;
}

std::vector<bool> Recording::contiguity() const {
    std::vector<bool> out(m_seq.size(), true);
    for (size_t i = 0; i + 1 < m_seq.size(); ++i) out[i] = m_seq[i + 1] == m_seq[i] + m_n[i];
    return out;
}

} // namespace ins10
