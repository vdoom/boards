#include "device/simulation.h"

#include "core/attitude.h"

#include <Eigen/Geometry>
#include <Eigen/LU>

#include <cmath>
#include <numbers>

namespace ins10::sim {

namespace {
constexpr double kPi = std::numbers::pi;
constexpr double kDeg = kPi / 180.0;

// Rotation by `angle` about `axis`, smoothly started and stopped, over `duration`:
// rate = axis * angle * (1 - cos(2 pi tau / T)) / T, which integrates to angle.
Eigen::Vector3d pulse(const Eigen::Vector3d &axis, double angle, double tau, double duration) {
    if (tau < 0.0 || tau >= duration) return Eigen::Vector3d::Zero();
    return axis * (angle * (1.0 - std::cos(2.0 * kPi * tau / duration)) / duration);
}

// Out to `angle` and back about `axis` over `duration`.
Eigen::Vector3d swing(const Eigen::Vector3d &axis, double angle, double tau, double duration) {
    if (tau < 0.0 || tau >= duration) return Eigen::Vector3d::Zero();
    return axis * (angle * kPi / duration * std::sin(2.0 * kPi * tau / duration));
}
} // namespace

Eigen::Vector3d defaultFieldNav() {
    return 50.0 * Eigen::Vector3d(0.0, std::cos(kDipDeg * kDeg), -std::sin(kDipDeg * kDeg));
}

SensorModel SensorModel::ideal() {
    SensorModel s;
    s.gyroNoiseDps = s.accelNoiseG = s.magNoiseUt = 0.0;
    return s;
}

SensorModel SensorModel::typical() {
    SensorModel s;
    s.gyroBias = {0.31, -0.22, 0.14};
    s.gyroMatrix << 1.004, 0.003, -0.002, -0.004, 0.995, 0.001, 0.002, -0.003, 1.006;
    s.accelBias = {0.012, -0.018, 0.025};
    s.accelMatrix << 1.004, 0.002, -0.001, 0.002, 0.997, 0.003, -0.001, 0.003, 1.006;
    s.magHard = {23.0, -41.0, 12.0};
    s.magSoft << 1.08, 0.03, -0.02, 0.03, 0.94, 0.04, -0.02, 0.04, 1.01;
    s.magMount = (Eigen::AngleAxisd(178.0 * kDeg, Eigen::Vector3d::UnitZ()) *
                  Eigen::AngleAxisd(2.0 * kDeg, Eigen::Vector3d::UnitY()) *
                  Eigen::AngleAxisd(-1.5 * kDeg, Eigen::Vector3d::UnitX()))
                     .toRotationMatrix();
    s.gyroTc << 0.012, 2e-4, 0.0, -0.008, -1e-4, 0.0, 0.015, 3e-4, 0.0;
    s.accelTc << 1.5e-4, 0.0, 0.0, -1.0e-4, 0.0, 0.0, 2.5e-4, 0.0, 0.0;
    s.tcRefC = 25.0;
    return s;
}

Eigen::Vector3d SensorModel::gyroDrift(double tempC) const {
    const double d = tempC - tcRefC;
    return gyroTc.col(0) * d + gyroTc.col(1) * (d * d) + gyroTc.col(2) * (d * d * d);
}

Eigen::Vector3d SensorModel::accelDrift(double tempC) const {
    const double d = tempC - tcRefC;
    return accelTc.col(0) * d + accelTc.col(1) * (d * d) + accelTc.col(2) * (d * d * d);
}

double StillMotion::temperature(double t) const {
    return m_t0 + (m_t1 - m_t0) * (1.0 - std::exp(-t / m_tau));
}

MultiPositionMotion::MultiPositionMotion(int positions, quint64 seed, double tempC) : m_temp(tempC) {
    std::mt19937_64 rng(seed);
    std::normal_distribution<double> normal;
    std::uniform_real_distribution<double> angle(70.0 * kDeg, 150.0 * kDeg);
    for (int i = 0; i < positions; ++i) {
        Eigen::Vector3d axis(normal(rng), normal(rng), normal(rng));
        axis.normalize();
        m_turns.push_back({axis, angle(rng)});
    }
}

double MultiPositionMotion::duration() const { return 5.0 * static_cast<double>(m_turns.size()) + 3.5; }

Eigen::Vector3d MultiPositionMotion::rate(double t) const {
    const double k = std::floor(t / 5.0);
    if (k < 0.0 || k >= static_cast<double>(m_turns.size())) return Eigen::Vector3d::Zero();
    const Turn &turn = m_turns[static_cast<size_t>(k)];
    return pulse(turn.axis, turn.angle, t - 5.0 * k - 3.5, 1.5);
}

TumbleMotion::TumbleMotion(double maxRateDps, quint64 seed, double tempC) : m_temp(tempC) {
    std::mt19937_64 rng(seed);
    std::uniform_real_distribution<double> freq(0.03, 0.25), amp(0.3, 1.0), phase(0.0, 6.28);
    double largest = 0.0;
    for (auto &axis : m_terms) {
        double sum = 0.0;
        for (Term &term : axis) {
            term = {amp(rng), freq(rng), phase(rng)};
            sum += term.amplitude;
        }
        largest = std::max(largest, sum);
    }
    // The sum of the amplitudes bounds each component; the terms rarely line up, so
    // scale for a peak of about the requested rate.
    m_scale = maxRateDps * kDeg / (0.7 * largest);
}

Eigen::Vector3d TumbleMotion::rate(double t) const {
    Eigen::Vector3d w = Eigen::Vector3d::Zero();
    for (int a = 0; a < 3; ++a) {
        for (const Term &term : m_terms[a]) w[a] += term.amplitude * std::sin(2.0 * kPi * term.frequency * t + term.phase);
    }
    return w * m_scale;
}

Eigen::Vector3d DemoMotion::rate(double t) const {
    const double tau = std::fmod(t, 40.0);
    const Eigen::Vector3d x = Eigen::Vector3d::UnitX(), y = Eigen::Vector3d::UnitY(), z = Eigen::Vector3d::UnitZ();
    Eigen::Vector3d w = Eigen::Vector3d::Zero();
    w += pulse(z, 90.0 * kDeg, tau - 4.0, 4.0);      // quarter turn to the left
    w += swing(x, 30.0 * kDeg, tau - 10.0, 6.0);     // roll out and back
    w += swing(y, -20.0 * kDeg, tau - 18.0, 6.0);    // pitch up and back
    w += pulse(z, -180.0 * kDeg, tau - 26.0, 10.0);  // slow half turn to the right...
    w += swing(x, 15.0 * kDeg, tau - 26.0, 5.0);     // ...rocking on the way
    w += swing(x, -15.0 * kDeg, tau - 31.0, 5.0);
    w += pulse(z, 90.0 * kDeg, tau - 36.0, 3.5);     // and back to the start
    return w;
}

double DemoMotion::temperature(double t) const { return 24.0 + 9.0 * (1.0 - std::exp(-t / 400.0)); }

Simulator::Simulator(const Motion *motion, const SensorModel &sensors, double odrHz, quint64 seed)
    : m_motion(motion), m_sensors(sensors), m_dt(1.0 / odrHz), m_rng(seed), m_seed(seed) {
    if (m_motion) m_tempC = m_motion->temperature(0.0);
}

void Simulator::reset() {
    m_rng.seed(m_seed);
    m_q = {1.0, 0.0, 0.0, 0.0};
    m_rateDps.setZero();
    m_t = 0.0;
    m_index = 0;
    if (m_motion) m_tempC = m_motion->temperature(0.0);
}

Eigen::Vector3d Simulator::noise(double sigma) {
    if (sigma <= 0.0) return Eigen::Vector3d::Zero();
    return {sigma * m_normal(m_rng), sigma * m_normal(m_rng), sigma * m_normal(m_rng)};
}

SensorReading Simulator::step() {
    m_t = static_cast<double>(m_index) * m_dt;
    const Eigen::Vector3d w = m_motion ? m_motion->rate(m_t) : Eigen::Vector3d::Zero();
    m_q = quatMultiply(m_q, quatFromRotvec(w * m_dt));
    if ((m_index & 1023) == 0) m_q = quatNormalized(m_q);
    m_tempC = m_motion ? m_motion->temperature(m_t) : 25.0;
    ++m_index;

    const Eigen::Matrix3d r = quatToDcm(m_q);
    const Eigen::Vector3d up = r.transpose() * Eigen::Vector3d::UnitZ();
    m_rateDps = w / kDeg;
    SensorReading s;
    s.tempC = m_tempC;
    s.gyroDps = m_sensors.gyroMatrix.inverse() * m_rateDps + m_sensors.gyroBias + m_sensors.gyroDrift(m_tempC) +
                noise(m_sensors.gyroNoiseDps);
    s.accelG = m_sensors.accelMatrix.inverse() * up + m_sensors.accelBias + m_sensors.accelDrift(m_tempC) +
               noise(m_sensors.accelNoiseG);
    return s;
}

Eigen::Vector3d Simulator::magReading() {
    const Eigen::Vector3d fieldBody = quatToDcm(m_q).transpose() * m_sensors.fieldNav;
    return m_sensors.magSoft * (m_sensors.magMount * fieldBody) + m_sensors.magHard + noise(m_sensors.magNoiseUt);
}

} // namespace ins10::sim
