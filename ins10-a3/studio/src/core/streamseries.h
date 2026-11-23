// The stream records turned into plot series on one time axis (board time, s).
//
// Buffers are sized for at least ten minutes at 200 Hz attitude output and at the raw
// rates the plots ask for; memory does not grow after construction.
#pragma once

#include "core/recording.h"
#include "core/series.h"
#include "core/status.h"

namespace ins10 {

class StreamSeries {
public:
    enum AttChannel { Roll, Pitch, Yaw, Heading, BiasX, BiasY, BiasZ, TiltSigma, HeadingSigma, AttTemp };
    enum RawChannel { GyroX, GyroY, GyroZ, AccelX, AccelY, AccelZ, RawTemp };
    enum MagChannel { MagX, MagY, MagZ, MagNorm };
    enum BaroChannel { Altitude, Pressure, BaroTemp };

    static constexpr qsizetype kAttCapacity = 1 << 17;  // 10.9 min at 200 Hz
    static constexpr qsizetype kRawCapacity = 1 << 17;  // 17 min at 128 Hz (raw 4)
    static constexpr qsizetype kMagCapacity = 1 << 15;  // 10.7 min at 51 Hz
    static constexpr qsizetype kBaroCapacity = 1 << 14; // 10.7 min at 25.6 Hz

    StreamSeries();

    TimeSeriesGroup att;
    TimeSeriesGroup raw;
    TimeSeriesGroup mag;
    TimeSeriesGroup baro;

    // Channel names in the current interface language.
    void retranslate();
    // Scale factors and the IMU rate from 'info'.
    void setInfo(const InfoReport &info);
    void feed(const ParsedLine &line);
    void clear();
    // Newest time in any group.
    double latestTime() const;
    size_t memoryBytes() const;

private:
    Scales m_scales;
    StreamClock m_clock;
};

} // namespace ins10
