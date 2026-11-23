// The three calibration procedures as the command-line tool runs them (cmd_imucal,
// cmd_magcal, cmd_thermalcal in tools/ins10tool/cli.py): what is subtracted before
// fitting, which fields and flag bits are written, when a result is refused. Plus the
// live progress figures the tool prints while recording.
#pragma once

#include "core/calib/imu.h"
#include "core/calib/mag.h"
#include "core/calib/thermal.h"
#include "core/calibration_fields.h"
#include "core/recording.h"

#include <atomic>
#include <optional>

namespace ins10::calib {

enum class Procedure { Imu, Mag, Thermal };

struct WorkflowResult {
    Procedure procedure = Procedure::Imu;
    bool ok = false;                 // a calibration was computed
    QString error;                   // why not, when !ok
    std::optional<CalibrationError::Code> errorCode;
    bool writable = false;           // the result may be written to the board
    QString refusal;                 // why it may not, when computed but not writable
    CalibrationSet fields;           // fields to write ("cal set")
    quint32 setFlags = 0;            // bits OR-ed into the flag word
    Warnings warnings;
    std::optional<ImuCalibration> imu;
    std::optional<MagCalibration> mag;
    std::optional<ThermalCalibration> thermal;
    // Inputs as fitted, for the result views.
    Eigen::MatrixX3d magInput;       // uT
    Eigen::VectorXd thermalTemp;     // C
    Eigen::MatrixX3d thermalGyro, thermalAccel;
};

// The stored temperature polynomial of "gyro" or "accel" evaluated at temp (cli.tempco).
Eigen::MatrixX3d tempco(const CalibrationSet &cal, const QString &prefix, const Eigen::VectorXd &temp);

WorkflowResult runImuCal(const Recording &rec, const CalibrationSet &current, const std::atomic<bool> *cancel = nullptr);

struct MagOptions {
    // With a board attached, an uncalibrated accelerometer is worth a warning (the tool
    // does not know the board's calibration offline).
    bool deviceConnected = true;
    bool southern = false;           // the field points up: picks the sign of the alignment (mag.h)
};
WorkflowResult runMagCal(const Recording &rec, const CalibrationSet &current, const MagOptions &options = {},
                         const std::atomic<bool> *cancel = nullptr);

WorkflowResult runThermalCal(const Recording &rec, const CalibrationSet &current,
                             const std::atomic<bool> *cancel = nullptr);

// ---- Live progress -----------------------------------------------------------------

struct ImuProgress {
    std::vector<StaticInterval> intervals;
    int positions = 0;
    int orientations = 0;
};
ImuProgress imuProgress(const Recording &rec);

struct MagProgress {
    int samples = 0;
    double coverage = 0.0;               // of directions about the centre of the box
    std::vector<bool> cells;             // 6 bands x 12 sectors
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    double radius = 0.0;                 // mean distance from the centre
};
MagProgress magProgress(const Recording &rec);

struct ThermalProgress {
    int samples = 0;
    double now = 0.0, min = 0.0, max = 0.0;
    double span() const { return max - min; }
    double gyroMotionDps = 0.0;          // largest deviation from the median gyro reading
};
ThermalProgress thermalProgress(const Recording &rec);

} // namespace ins10::calib
