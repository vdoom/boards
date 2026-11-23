// A recorded log run through the firmware's own fusion library (lib/ins10, compiled
// for the PC from the firmware's sources): what the board's filter would have output
// on that data. The same steps as the firmware's tests/replay.c (ins10_replay), so the
// two give the same result on the same log.
#pragma once

#include <QString>
#include <QStringList>

#include <array>
#include <atomic>
#include <vector>

namespace ins10 {

struct ReplayRow {
    double t = 0.0;                     // s of IMU data processed
    std::array<double, 4> q{};          // w x y z
    double rollDeg = 0, pitchDeg = 0, yawDeg = 0, headingDeg = 0;
    std::array<double, 3> biasDps{};
    double tiltSigmaDeg = 0, headingSigmaDeg = 0;
    unsigned flags = 0;
};

struct ReplayResult {
    std::vector<ReplayRow> rows;        // one per $RAW line once the filter is aligned
    long imuRecords = 0;
    long lostSamples = 0;
    double seconds = 0.0;
    QStringList ignoredCalibration;     // 'cal set' lines the library refused
    bool cancelled = false;
};

struct ReplayOptions {
    double rateHz = 512.0;              // IMU sample rate (ins10_replay --rate)
    QStringList calibration;            // 'cal set' lines applied before the log
    const std::atomic<bool> *cancel = nullptr;
};

ReplayResult replayLog(const QStringList &lines, const ReplayOptions &options = {});

// The CSV that ins10_replay prints, header included.
QStringList replayCsv(const ReplayResult &result);

} // namespace ins10
