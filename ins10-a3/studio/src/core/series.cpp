#include "core/series.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ins10 {

TimeSeriesGroup::TimeSeriesGroup(const QStringList &channels, qsizetype capacity)
    : m_names(channels), m_capacity(std::max<qsizetype>(capacity, 2)) {
    m_t.assign(static_cast<size_t>(m_capacity), 0.0);
    m_v.resize(static_cast<size_t>(channels.size()));
    for (auto &v : m_v) v.assign(static_cast<size_t>(m_capacity), 0.0f);
}

size_t TimeSeriesGroup::memoryBytes() const {
    size_t bytes = m_t.capacity() * sizeof(double);
    for (const auto &v : m_v) bytes += v.capacity() * sizeof(float);
    return bytes;
}

void TimeSeriesGroup::clear() {
    m_start = m_size = 0;
}

void TimeSeriesGroup::append(double t, const double *values, int count) {
    if (!std::isfinite(t)) return;
    if (m_size > 0) {
        const double last = lastTime();
        if (t < last - resetJump) clear();
        else if (t < last) t = last;
    }
    size_t s;
    if (m_size < m_capacity) {
        s = slot(m_size);
        ++m_size;
    } else {
        s = static_cast<size_t>(m_start);
        m_start = (m_start + 1) % m_capacity;
    }
    m_t[s] = t;
    for (int c = 0; c < channels(); ++c) {
        const double v = c < count ? values[c] : std::numeric_limits<double>::quiet_NaN();
        m_v[static_cast<size_t>(c)][s] = std::isfinite(v) ? static_cast<float>(v) : std::numeric_limits<float>::quiet_NaN();
    }
    ++m_appended;
}

qsizetype TimeSeriesGroup::lowerBound(double t) const {
    qsizetype lo = 0, hi = m_size;
    while (lo < hi) {
        const qsizetype mid = (lo + hi) / 2;
        if (time(mid) < t) lo = mid + 1;
        else hi = mid;
    }
    return lo;
}

std::optional<qsizetype> TimeSeriesGroup::nearest(double t) const {
    if (m_size == 0) return std::nullopt;
    const qsizetype i = lowerBound(t);
    if (i >= m_size) return m_size - 1;
    if (i == 0) return 0;
    return t - time(i - 1) < time(i) - t ? i - 1 : i;
}

void TimeSeriesGroup::envelope(int channel, double t0, double t1, int columns, std::vector<Bin> &out) const {
    out.assign(static_cast<size_t>(std::max(columns, 0)), Bin());
    if (columns <= 0 || !(t1 > t0) || m_size == 0 || channel < 0 || channel >= channels()) return;
    const std::vector<float> &v = m_v[static_cast<size_t>(channel)];
    const double scale = columns / (t1 - t0);
    const int lastColumn = columns - 1;
    Bin *bins = out.data();
    // The ring is at most two contiguous runs; no modulo per sample.
    qsizetype i = lowerBound(t0);
    while (i < m_size) {
        const size_t s0 = slot(i);
        const qsizetype run = std::min<qsizetype>(m_size - i, m_capacity - static_cast<qsizetype>(s0));
        const double *t = m_t.data() + s0;
        const float *x = v.data() + s0;
        for (qsizetype k = 0; k < run; ++k) {
            if (t[k] >= t1) return;
            const float value = x[k];
            if (std::isnan(value)) continue;
            const int col = std::min(lastColumn, static_cast<int>((t[k] - t0) * scale));
            Bin &b = bins[col];
            if (b.count == 0) {
                b.min = b.max = b.first = value;
            } else {
                b.min = std::min(b.min, value);
                b.max = std::max(b.max, value);
            }
            b.last = value;
            ++b.count;
        }
        i += run;
    }
}

std::optional<std::pair<float, float>> TimeSeriesGroup::range(int channel, double t0, double t1) const {
    if (channel < 0 || channel >= channels()) return std::nullopt;
    const std::vector<float> &v = m_v[static_cast<size_t>(channel)];
    float lo = std::numeric_limits<float>::infinity(), hi = -lo;
    qsizetype i = lowerBound(t0);
    bool done = false;
    while (i < m_size && !done) {
        const size_t s0 = slot(i);
        const qsizetype run = std::min<qsizetype>(m_size - i, m_capacity - static_cast<qsizetype>(s0));
        const double *t = m_t.data() + s0;
        const float *x = v.data() + s0;
        for (qsizetype k = 0; k < run; ++k) {
            if (t[k] > t1) {
                done = true;
                break;
            }
            if (std::isnan(x[k])) continue;
            lo = std::min(lo, x[k]);
            hi = std::max(hi, x[k]);
        }
        i += run;
    }
    if (!(lo <= hi)) return std::nullopt;
    return std::make_pair(lo, hi);
}

void StreamClock::reset() {
    m_lastBoardTime.reset();
    m_offset.reset();
}

double StreamClock::rawTime(const RawRecord &r) {
    const double sampleTime = (static_cast<double>(r.seq) + 0.5 * r.n) / m_odr;
    if (!m_offset) m_offset = m_lastBoardTime ? *m_lastBoardTime - static_cast<double>(r.seq) / m_odr : 0.0;
    return sampleTime + *m_offset;
}

} // namespace ins10
