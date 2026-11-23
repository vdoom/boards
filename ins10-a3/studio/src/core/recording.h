// Sensor data collected from the stream, in physical units and uncalibrated: the C++
// counterpart of class Recording in tools/ins10tool/records.py.
//
// IMU rows are block means: the firmware summed `n` raw samples starting at sample
// index `seq`. Time is seq / odr, so it is exact and shows lost samples. Magnetometer
// rows remember how many IMU rows had arrived before them, which pairs each with the
// IMU reading at about the same moment.
#pragma once

#include "core/calibration_fields.h"
#include "core/protocol.h"

#include <Eigen/Core>

#include <QString>
#include <QStringView>

#include <vector>

namespace ins10 {

// Scale factors of the ICM-42688-P 20-bit FIFO format; the "# key=value" lines of
// 'info' in the stream or log override them.
struct Scales {
    double gyroDpsPerLsb = 2000.0 / 524288.0;
    double accelGPerLsb = 16.0 / 524288.0;
    double tempLsbPerC = 132.48;
    double tempOffsetC = 25.0;
    double odrHz = 512.0;

    // Updates the known keys from an 'info' line; returns true if any was set.
    bool update(const QString &key, const QString &value);
};

struct ImuArrays {
    Eigen::VectorXd t;      // s, middle of each block
    Eigen::VectorXd dt;     // s, duration of each block
    Eigen::MatrixX3d gyro;  // dps
    Eigen::MatrixX3d accel; // g
    Eigen::VectorXd temp;   // deg C
    Eigen::Index rows() const { return t.size(); }
};

struct MagWithImu {
    Eigen::MatrixX3d mag;   // uT
    Eigen::MatrixX3d accel; // g, mean of the IMU rows around each magnetometer row
    Eigen::MatrixX3d gyro;  // dps
};

class Recording {
public:
    enum class Fed { Nothing, Raw, Mag, Info, Cal };

    Fed feed(QStringView line);
    Fed feed(const ParsedLine &line);
    void clear();

    const Scales &scales() const { return m_scales; }
    void setScales(const Scales &scales) { m_scales = scales; }
    const CalibrationSet &calibration() const { return m_calibration; }

    qsizetype imuRows() const { return static_cast<qsizetype>(m_seq.size()); }
    qsizetype magRows() const { return static_cast<qsizetype>(m_mag.size()); }
    // Samples missing between consecutive IMU rows (only forward gaps, as records.py).
    qint64 lostSamples() const { return m_lostSamples; }
    // Number of places where an IMU row does not follow the previous one.
    qint64 breaks() const { return m_breaks; }
    bool isContiguous() const { return m_breaks == 0; }

    const std::vector<qint64> &seq() const { return m_seq; }
    const std::vector<qint32> &n() const { return m_n; }
    const std::vector<Eigen::Vector3d> &gyro() const { return m_gyro; }
    const std::vector<Eigen::Vector3d> &accel() const { return m_accel; }
    const std::vector<double> &temp() const { return m_temp; }
    const std::vector<Eigen::Vector3d> &mag() const { return m_mag; }
    const std::vector<qsizetype> &magImuIndex() const { return m_magImuIndex; }

    // All IMU rows, or `count` rows from `first`.
    ImuArrays imuArrays() const;
    ImuArrays imuArrays(qsizetype first, qsizetype count) const;
    // Magnetometer rows with the IMU reading at the same moment: the mean of the IMU
    // rows just before and just after each one; rows at either end are dropped.
    MagWithImu magWithImu() const;
    // contiguity[i] is false where IMU rows i and i + 1 are not consecutive.
    std::vector<bool> contiguity() const;

private:
    Scales m_scales;
    std::vector<qint64> m_seq;
    std::vector<qint32> m_n;
    std::vector<Eigen::Vector3d> m_gyro, m_accel;
    std::vector<double> m_temp;
    std::vector<Eigen::Vector3d> m_mag;
    std::vector<qsizetype> m_magImuIndex;
    qint64 m_lostSamples = 0;
    qint64 m_breaks = 0;
    CalibrationSet m_calibration;
};

} // namespace ins10
