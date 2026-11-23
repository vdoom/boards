// Time series for the plots: fixed-capacity ring buffers, several channels sharing one
// time axis, with a min/max envelope per screen column for drawing.
//
// Memory is allocated once; appending never allocates, and when the buffer is full
// the oldest sample is overwritten.
#pragma once

#include "core/protocol.h"

#include <QString>
#include <QStringList>

#include <optional>
#include <vector>

namespace ins10 {

class TimeSeriesGroup {
public:
    TimeSeriesGroup(const QStringList &channels, qsizetype capacity);

    int channels() const { return static_cast<int>(m_names.size()); }
    const QStringList &names() const { return m_names; }
    // New names for the same channels (another interface language).
    void setNames(const QStringList &names) {
        if (names.size() == m_names.size()) m_names = names;
    }
    qsizetype size() const { return m_size; }
    qsizetype capacity() const { return m_capacity; }
    bool isEmpty() const { return m_size == 0; }
    quint64 appended() const { return m_appended; }
    // Bytes held by the buffers; constant for the life of the group.
    size_t memoryBytes() const;

    // Times must not decrease. A time more than `resetJump` s before the last one
    // means a new time base (board restart): the group is cleared first. Smaller
    // steps back are clamped. Non-finite values are stored as NaN and skipped when
    // drawing.
    void append(double t, const double *values, int count);
    void append(double t, std::initializer_list<double> values) {
        append(t, values.begin(), static_cast<int>(values.size()));
    }
    void clear();

    // i = 0 is the oldest sample.
    double time(qsizetype i) const { return m_t[slot(i)]; }
    float value(int channel, qsizetype i) const { return m_v[static_cast<size_t>(channel)][slot(i)]; }
    double firstTime() const { return m_size ? time(0) : 0.0; }
    double lastTime() const { return m_size ? time(m_size - 1) : 0.0; }
    // First sample with time >= t.
    qsizetype lowerBound(double t) const;
    // Sample nearest to t, if any.
    std::optional<qsizetype> nearest(double t) const;

    struct Bin {
        float min = 0, max = 0, first = 0, last = 0;
        int count = 0;
    };
    // Envelope of a channel over [t0, t1) in `columns` equal bins.
    void envelope(int channel, double t0, double t1, int columns, std::vector<Bin> &out) const;
    // Min and max of a channel over [t0, t1]; nothing if no finite sample.
    std::optional<std::pair<float, float>> range(int channel, double t0, double t1) const;

    double resetJump = 1.0;

private:
    size_t slot(qsizetype i) const { return static_cast<size_t>((m_start + i) % m_capacity); }

    QStringList m_names;
    qsizetype m_capacity;
    std::vector<double> m_t;
    std::vector<std::vector<float>> m_v;
    qsizetype m_start = 0, m_size = 0;
    quint64 m_appended = 0;
};

// One time axis for all streams. $ATT, $MAG and $BARO carry the Pico's t_ms; $RAW
// carries only its sample number. The offset between the two is estimated once, at
// the first $RAW (latest t_ms minus seq / odr), and kept until reset().
class StreamClock {
public:
    void reset();
    void setOdr(double hz) { if (hz > 0) m_odr = hz; }
    double odr() const { return m_odr; }
    void noteBoardTime(qint64 tMs) { m_lastBoardTime = static_cast<double>(tMs) * 1e-3; }
    // Board time (s) of the middle of a $RAW block.
    double rawTime(const RawRecord &r);
    bool hasOffset() const { return m_offset.has_value(); }
    double offset() const { return m_offset.value_or(0.0); }

private:
    double m_odr = 512.0;
    std::optional<double> m_lastBoardTime;
    std::optional<double> m_offset;
};

} // namespace ins10
