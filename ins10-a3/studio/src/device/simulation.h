// Motion and sensor models behind the simulated board.
//
// A motion is a body rate as a function of time; the attitude is its integral, so the
// simulated gyro is exactly consistent with the attitude the accelerometer and
// magnetometer see. Sensor errors follow the firmware's calibration model
//     corrected = matrix * (raw - bias - tempco(T - t_ref))
// inverted: raw = matrix^-1 * true + bias + tempco + noise.
#pragma once

#include <Eigen/Core>

#include <QtGlobal>

#include <random>
#include <vector>

namespace ins10::sim {

inline constexpr double kDipDeg = 66.0; // Ukraine
// Earth's field, ENU, uT: 50 uT, dipping 66 deg below the horizon towards north.
Eigen::Vector3d defaultFieldNav();

struct SensorModel {
    Eigen::Vector3d gyroBias = Eigen::Vector3d::Zero();   // dps at tcRef
    Eigen::Matrix3d gyroMatrix = Eigen::Matrix3d::Identity();
    Eigen::Vector3d accelBias = Eigen::Vector3d::Zero();  // g
    Eigen::Matrix3d accelMatrix = Eigen::Matrix3d::Identity();
    Eigen::Vector3d magHard = Eigen::Vector3d::Zero();    // uT
    Eigen::Matrix3d magSoft = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d magMount = Eigen::Matrix3d::Identity(); // IMU axes -> magnetometer axes
    // Bias drift per axis (rows) for (T - tcRef)^1..3 (columns).
    Eigen::Matrix3d gyroTc = Eigen::Matrix3d::Zero();     // dps / C^k
    Eigen::Matrix3d accelTc = Eigen::Matrix3d::Zero();    // g / C^k
    double tcRefC = 25.0;
    double gyroNoiseDps = 0.045;   // per sample at 512 Hz
    double accelNoiseG = 0.0011;
    double magNoiseUt = 0.05;
    Eigen::Vector3d fieldNav = defaultFieldNav();

    static SensorModel ideal();     // no errors, no noise
    static SensorModel typical();   // errors of an uncalibrated board, as in the Python tests

    Eigen::Vector3d gyroDrift(double tempC) const;
    Eigen::Vector3d accelDrift(double tempC) const;
};

// Raw readings in physical units for a body rate and attitude.
struct SensorReading {
    Eigen::Vector3d gyroDps;
    Eigen::Vector3d accelG;
    Eigen::Vector3d magUt;
    double tempC = 25.0;
};

class Motion {
public:
    virtual ~Motion() = default;
    virtual Eigen::Vector3d rate(double t) const = 0; // body rate, rad/s
    virtual double temperature(double t) const { (void)t; return 25.0; }
};

// Lying still; temperature follows T0 + (T1 - T0)(1 - exp(-t / tau)).
class StillMotion : public Motion {
public:
    StillMotion(double t0C = 25.0, double t1C = 25.0, double tauS = 600.0) : m_t0(t0C), m_t1(t1C), m_tau(tauS) {}
    Eigen::Vector3d rate(double) const override { return Eigen::Vector3d::Zero(); }
    double temperature(double t) const override;

private:
    double m_t0, m_t1, m_tau;
};

// Rest 3.5 s, turn to a new orientation in 1.5 s about a random axis by 70-150 deg,
// and so on (multi_position_motion in test_calib.py). Ends with a rest.
class MultiPositionMotion : public Motion {
public:
    MultiPositionMotion(int positions, quint64 seed, double tempC = 31.0);
    Eigen::Vector3d rate(double t) const override;
    double temperature(double) const override { return m_temp; }
    double duration() const;

private:
    struct Turn {
        Eigen::Vector3d axis;
        double angle;
    };
    std::vector<Turn> m_turns;
    double m_temp;
};

// Slow, smooth tumble through all orientations (smooth_tumble in test_calib.py).
class TumbleMotion : public Motion {
public:
    TumbleMotion(double maxRateDps, quint64 seed, double tempC = 30.0);
    Eigen::Vector3d rate(double t) const override;
    double temperature(double) const override { return m_temp; }

private:
    struct Term {
        double amplitude, frequency, phase;
    };
    Term m_terms[3][4];
    double m_scale;
    double m_temp;
};

// The --demo choreography: rest, a quarter turn in yaw, roll and pitch swings, a slow
// tumble, repeating every 40 s. Warms up slowly.
class DemoMotion : public Motion {
public:
    Eigen::Vector3d rate(double t) const override;
    double temperature(double t) const override;
};

// Integrates a motion at the IMU rate and produces sensor readings.
class Simulator {
public:
    Simulator(const Motion *motion, const SensorModel &sensors, double odrHz, quint64 seed);

    // Advances by one IMU sample and returns its readings (gyro, accel, temp; the
    // magnetometer reading is computed only on request).
    SensorReading step();
    Eigen::Vector3d magReading();          // at the current attitude, with noise
    Eigen::Vector4d attitude() const { return m_q; } // body -> ENU {w, x, y, z}
    Eigen::Vector3d trueRateDps() const { return m_rateDps; }
    double time() const { return m_t; }    // s, time of the last sample
    double temperature() const { return m_tempC; }
    qint64 sampleIndex() const { return m_index; }
    const SensorModel &sensors() const { return m_sensors; }
    void setSensors(const SensorModel &s) { m_sensors = s; }
    void setMotion(const Motion *motion) { m_motion = motion; }
    void reset();
    void setAttitude(const Eigen::Vector4d &q) { m_q = q; }

private:
    const Motion *m_motion;
    SensorModel m_sensors;
    double m_dt;
    std::mt19937_64 m_rng;
    std::normal_distribution<double> m_normal{0.0, 1.0};
    Eigen::Vector4d m_q{1.0, 0.0, 0.0, 0.0};
    Eigen::Vector3d m_rateDps = Eigen::Vector3d::Zero();
    double m_t = 0.0;
    double m_tempC = 25.0;
    qint64 m_index = 0;
    quint64 m_seed;

    Eigen::Vector3d noise(double sigma);
};

} // namespace ins10::sim
