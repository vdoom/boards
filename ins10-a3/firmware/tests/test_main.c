/* Host test suite for the INS10 fusion library.
 *
 *   ins10_tests            run everything
 *   ins10_tests <name>     run one test (see the table at the bottom)
 *
 * Scenario tests drive the complete pipeline with simulated sensors and assert on the
 * attitude error against the simulator's ground truth. Each prints its metrics, so the
 * output doubles as a performance report.
 */
#include "ins10/ins10.h"
#include "sim.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define PI 3.14159265358979323846
#define D2R (PI / 180.0)

static int failures;

#define CHECK(cond) \
    do { \
        if (!(cond)) { \
            printf("    FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            ++failures; \
        } \
    } while (0)

/* ---- Motion profiles ------------------------------------------------------------- */

static double smooth_gate(double t, double start, double stop, double ramp) {
    if (t <= start || t >= stop) return 0.0;
    double a = (t - start) / ramp, b = (stop - t) / ramp;
    double x = a < b ? a : b;
    return x >= 1.0 ? 1.0 : 0.5 - 0.5 * cos(PI * x);
}

static void rate_zero(double t, double w[3]) {
    (void)t;
    w[0] = w[1] = w[2] = 0.0;
}

/* Continuous tumbling through every orientation, peaks near 150 dps. */
static void rate_tumble(double t, double w[3]) {
    w[0] = 1.2 * sin(0.31 * t) + 0.8 * sin(1.3 * t + 1.0);
    w[1] = 1.0 * sin(0.23 * t + 0.5) + 0.6 * cos(0.9 * t);
    w[2] = 0.9 * cos(0.17 * t) + 0.7 * sin(1.1 * t + 2.0);
}

/* Gentle handling after 3 s at rest: slow turns about all axes, peaks near 30 dps. */
static void rate_gentle(double t, double w[3]) {
    double g = smooth_gate(t, 3.0, 1e12, 1.0);
    w[0] = g * 0.25 * sin(0.41 * t);
    w[1] = g * 0.20 * sin(0.29 * t + 1.0);
    w[2] = g * 0.30 * sin(0.19 * t + 2.0);
}

/* At rest for 10 s, tumbling for 20 s, repeating. */
static void rate_stop_go(double t, double w[3]) {
    double phase = fmod(t, 30.0);
    double g = smooth_gate(phase, 10.0, 30.0, 1.0);
    rate_tumble(t, w);
    for (int i = 0; i < 3; ++i) w[i] *= g;
}

/* 5 Hz coning with a 60 dps cone rate. */
static void rate_coning(double t, double w[3]) {
    double omega = 2.0 * PI * 5.0;
    w[0] = 1.0 * cos(omega * t);
    w[1] = 1.0 * sin(omega * t);
    w[2] = 0.3;
}

/* One violent spin beyond the gyro range, then gentle motion. */
static void rate_spike(double t, double w[3]) {
    rate_gentle(t, w);
    w[2] += 45.0 * smooth_gate(t, 20.0, 20.6, 0.2); /* up to about 2600 dps */
}

/* Horizontal acceleration bursts (vehicle-like) and a 2 Hz shake. */
static void acc_bursts(double t, double a[3]) {
    double phase = fmod(t, 20.0);
    double burst = 0.3 * smooth_gate(phase, 5.0, 10.0, 0.5);
    double shake = 0.5 * smooth_gate(phase, 13.0, 18.0, 0.5) * sin(2.0 * PI * 2.0 * t);
    a[0] = burst + 0.6 * shake;
    a[1] = -0.5 * burst + shake;
    a[2] = 0.3 * shake;
}

/* A 25 uT disturbance, fixed in the world, switched on for 15 s out of every 40 s
   with a different direction each time. */
static void mag_bursts(double t, double d[3]) {
    int n = (int)(t / 40.0);
    double g = smooth_gate(fmod(t, 40.0), 15.0, 30.0, 0.3);
    double az = 1.3 + 2.1 * n, el = 0.4 * ((n % 3) - 1);
    d[0] = 25.0 * g * cos(el) * cos(az);
    d[1] = 25.0 * g * cos(el) * sin(az);
    d[2] = 25.0 * g * sin(el);
}

/* Warm-up from 25 to 45 C in 400 s, then steady. */
static double temp_ramp(double t) {
    return 25.0 + 20.0 * (1.0 - exp(-t / 130.0));
}

/* ---- Scenario runner ------------------------------------------------------------- */

typedef struct {
    double tilt_sq, head_sq, tilt_max, head_max, bias_max;
    long n, beyond_3sigma;
} stats;

static void accumulate(stats *st, const sim *s, const ins10_fusion *f) {
    ins10_output o;
    double tilt, head, bias[3];
    ins10_fusion_get(f, &o);
    sim_errors(s, o.q, &tilt, &head, NULL);
    sim_true_bias(s, bias);
    st->tilt_sq += tilt * tilt;
    st->head_sq += head * head;
    if (tilt > st->tilt_max) st->tilt_max = tilt;
    if (fabs(head) > st->head_max) st->head_max = fabs(head);
    for (int i = 0; i < 3; ++i) {
        double e = fabs((double)o.gyro_bias_dps[i] - bias[i] / D2R);
        if (e > st->bias_max) st->bias_max = e;
    }
    if (tilt > 3.0 * 1.414 * (double)o.tilt_sigma_deg + 0.05 ||
        fabs(head) > 3.0 * (double)o.heading_sigma_deg + 0.1) ++st->beyond_3sigma;
    ++st->n;
}

static void report(const char *label, const stats *st) {
    double n = st->n > 0 ? (double)st->n : 1.0;
    printf("    %-26s tilt rms %.3f max %.3f deg | heading rms %.3f max %.3f deg | "
           "bias max %.4f dps | >3 sigma %.1f %%\n",
           label, sqrt(st->tilt_sq / n), st->tilt_max, sqrt(st->head_sq / n), st->head_max,
           st->bias_max, 100.0 * (double)st->beyond_3sigma / n);
}

static void feed(sim *s, ins10_fusion *f) {
    sim_sample x;
    sim_step(s, &x);
    ins10_fusion_imu(f, x.gyro_dps, x.accel_g, x.temp_c, (float)(1.0 / s->fs), x.saturated);
    if (x.mag_valid) ins10_fusion_mag(f, x.mag_ut, x.mag_age_s);
}

/* Runs until t_end, accumulating statistics from t_eval on. */
static void run(sim *s, ins10_fusion *f, double t_end, double t_eval, stats *st) {
    while (s->t < t_end) {
        feed(s, f);
        if (st && s->t >= t_eval) accumulate(st, s, f);
    }
}

static void make_fusion(ins10_fusion *f, const ins10_fusion_config *cfg, bool with_mag) {
    ins10_fusion_config c;
    ins10_calibration cal;
    if (cfg) c = *cfg;
    else ins10_fusion_default_config(&c);
    ins10_cal_defaults(&cal);
    if (with_mag) cal.flags |= INS10_CAL_MAG;
    ins10_fusion_init(f, &c, &cal, NULL);
}

static void typical_errors(sim *s) {
    s->gyro_bias[0] = 0.30 * D2R;
    s->gyro_bias[1] = -0.20 * D2R;
    s->gyro_bias[2] = 0.25 * D2R;
    s->gyro_scale = 0.002;
}

/* ---- Unit tests ------------------------------------------------------------------ */

static void test_math(void) {
    float v[3] = {0.3f, -1.1f, 0.7f}, q[4], r[9], r2[9], q2[4];
    ins10_quat_from_rotvec(v, q);
    ins10_quat_to_dcm(q, r);
    ins10_rotvec_to_dcm(v, r2);
    for (int i = 0; i < 9; ++i) CHECK(fabsf(r[i] - r2[i]) < 1e-5f);
    ins10_dcm_to_quat(r, q2);
    float sign = q[0] * q2[0] + q[1] * q2[1] + q[2] * q2[2] + q[3] * q2[3] > 0 ? 1.0f : -1.0f;
    for (int i = 0; i < 4; ++i) CHECK(fabsf(q[i] - sign * q2[i]) < 1e-5f);

    /* Rotating there and back is the identity. */
    float a[3] = {1.0f, 2.0f, 3.0f}, b[3], c[3];
    ins10_quat_rotate(q, a, b);
    ins10_quat_rotate_inv(q, b, c);
    for (int i = 0; i < 3; ++i) CHECK(fabsf(a[i] - c[i]) < 1e-5f);

    /* Euler angles of a known yaw-pitch-roll sequence. */
    float yaw = 0.7f, pitch = -0.4f, roll = 1.9f;
    float qz[4] = {cosf(yaw / 2), 0, 0, sinf(yaw / 2)};
    float qy[4] = {cosf(pitch / 2), 0, sinf(pitch / 2), 0};
    float qx[4] = {cosf(roll / 2), sinf(roll / 2), 0, 0};
    float t[4], e[3];
    ins10_quat_mul(qz, qy, t);
    ins10_quat_mul(t, qx, q);
    ins10_quat_to_euler(q, &e[0], &e[1], &e[2]);
    CHECK(fabsf(e[0] - roll) < 1e-5f && fabsf(e[1] - pitch) < 1e-5f && fabsf(e[2] - yaw) < 1e-5f);

    /* All four branches of the matrix-to-quaternion conversion. */
    const float cases[4][3] = {{0.1f, 0.2f, 0.1f}, {3.0f, 0.1f, 0.1f}, {0.1f, 3.0f, 0.1f}, {0.1f, 0.1f, 3.0f}};
    for (int k = 0; k < 4; ++k) {
        ins10_quat_from_rotvec(cases[k], q);
        ins10_quat_to_dcm(q, r);
        ins10_dcm_to_quat(r, q2);
        sign = q[0] * q2[0] + q[1] * q2[1] + q[2] * q2[2] + q[3] * q2[3] > 0 ? 1.0f : -1.0f;
        for (int i = 0; i < 4; ++i) CHECK(fabsf(q[i] - sign * q2[i]) < 1e-5f);
    }
}

/* The attitude integrator alone, on noise-free coning motion. */
static void test_integrator(void) {
    sim s;
    sim_defaults(&s, 1);
    s.rate = rate_coning;
    s.gyro_noise = 0.0;
    s.mag_div = 0;
    ins10_ekf e;
    const float q0[4] = {1, 0, 0, 0};
    ins10_ekf_init(&e, q0, NULL, 1e-4f, 1e-4f, 1e-8f);
    double worst = 0.0;
    while (s.t < 60.0) {
        sim_sample x;
        sim_step(&s, &x);
        float w[3];
        for (int i = 0; i < 3; ++i) w[i] = x.gyro_dps[i] * INS10_DEG2RAD;
        ins10_ekf_integrate(&e, w, (float)(1.0 / s.fs));
        double total;
        sim_errors(&s, e.q, NULL, NULL, &total);
        if (total > worst) worst = total;
    }
    printf("    coning 5 Hz, 60 s: worst integration error %.4f deg\n", worst);
    CHECK(worst < 0.005);
}

static void test_calibration(void) {
    ins10_calibration cal;
    ins10_cal_defaults(&cal);
    float raw[3] = {1.0f, 2.0f, 3.0f}, out[3];
    ins10_cal_apply_accel(&cal, raw, 40.0f, out);
    for (int i = 0; i < 3; ++i) CHECK(out[i] == raw[i]);

    float bias[3] = {0.1f, -0.2f, 0.3f};
    float matrix[9] = {1.1f, 0.01f, 0, 0, 0.9f, 0.02f, -0.03f, 0, 1.05f};
    float tc[3] = {0.001f, 0.0f, 0.00001f}, tref = 20.0f;
    CHECK(ins10_cal_set_field(&cal, "accel_bias", bias, 3));
    CHECK(ins10_cal_set_field(&cal, "accel_matrix", matrix, 9));
    CHECK(ins10_cal_set_field(&cal, "accel_tc_x", tc, 3));
    CHECK(ins10_cal_set_field(&cal, "accel_tc_tref", &tref, 1));
    CHECK(!ins10_cal_set_field(&cal, "accel_bias", bias, 2));
    CHECK(!ins10_cal_set_field(&cal, "no_such_field", bias, 3));
    ins10_cal_apply_accel(&cal, raw, 30.0f, out);
    float dx = raw[0] - bias[0] - (0.001f * 10 + 0.00001f * 1000);
    float dy = raw[1] - bias[1], dz = raw[2] - bias[2];
    CHECK(fabsf(out[0] - (1.1f * dx + 0.01f * dy)) < 1e-5f);
    CHECK(fabsf(out[1] - (0.9f * dy + 0.02f * dz)) < 1e-5f);
    CHECK(fabsf(out[2] - (-0.03f * dx + 1.05f * dz)) < 1e-5f);

    float gyro[3] = {180.0f, 0, 0};
    ins10_cal_apply_gyro(&cal, gyro, 25.0f, out);
    CHECK(fabsf(out[0] - INS10_PI) < 1e-5f);
}

static void test_persist(void) {
    ins10_persist a, b;
    uint8_t buffer[1024];
    uint32_t sequence = 0;
    ins10_persist_defaults(&a);
    a.cal.flags = INS10_CAL_MAG | INS10_CAL_ACCEL;
    a.cal.mag.bias[1] = -12.5f;
    a.thermal.count = 42;
    CHECK(ins10_crc32("123456789", 9) == 0xcbf43926u);
    size_t n = ins10_persist_encode(&a, 7, buffer, sizeof(buffer));
    CHECK(n == INS10_PERSIST_RECORD_SIZE && n <= sizeof(buffer));
    CHECK(ins10_persist_decode(buffer, n, &b, &sequence));
    CHECK(sequence == 7 && memcmp(&a, &b, sizeof(a)) == 0);
    buffer[40] ^= 0x01;
    CHECK(!ins10_persist_decode(buffer, n, &b, &sequence));
    buffer[40] ^= 0x01;
    CHECK(!ins10_persist_decode(buffer, n - 1, &b, &sequence));
    CHECK(ins10_persist_encode(&a, 7, buffer, 16) == 0);
    memset(buffer, 0xff, sizeof(buffer)); /* erased flash */
    CHECK(!ins10_persist_decode(buffer, sizeof(buffer), &b, &sequence));
}

static void test_thermal_model(void) {
    /* Recover a known quadratic from noisy observations across a temperature sweep. */
    sim s;
    sim_defaults(&s, 3);
    ins10_thermal t;
    ins10_thermal_init(&t, NULL, 0.02f * INS10_DEG2RAD, 2.0f, 8.0f);
    const double c0[3] = {0.004, -0.003, 0.001}, c1[3] = {2e-4, -1e-4, 3e-4}, c2[3] = {2e-5, 1e-5, -3e-5};
    for (int k = 0; k < 600; ++k) {
        double temp = 15.0 + 40.0 * k / 600.0, u = (temp - 25.0) / 10.0;
        float y[3];
        for (int a = 0; a < 3; ++a) {
            y[a] = (float)(c0[a] + c1[a] * u + c2[a] * u * u + 5e-5 * sim_gauss(&s));
        }
        ins10_thermal_observe(&t, y, (float)temp, 2.5e-9f, 0.0f);
    }
    float slope[3], sigma, bias[3];
    ins10_thermal_slope(&t, 45.0f, slope, &sigma);
    ins10_thermal_predict(&t, 45.0f, bias);
    for (int a = 0; a < 3; ++a) {
        double true_slope = (c1[a] + 2.0 * c2[a] * 2.0) / 10.0;
        double true_bias = c0[a] + c1[a] * 2.0 + c2[a] * 4.0;
        CHECK(fabs((double)slope[a] - true_slope) < 4e-6);
        CHECK(fabs((double)bias[a] - true_bias) < 3e-5);
    }
    CHECK(sigma < 3e-6f && t.model.count == 600);

    /* Reloading a stored model re-opens only the turn-on bias. */
    ins10_thermal reloaded;
    ins10_thermal_init(&reloaded, &t.model, 0.02f * INS10_DEG2RAD, 2.0f, 8.0f);
    CHECK(reloaded.model.cov[0] > t.model.cov[0] && reloaded.model.cov[3] == t.model.cov[3]);

    /* Temperature filter: a steady ramp is tracked as its slope. */
    ins10_thermal f;
    ins10_thermal_init(&f, NULL, 0.02f * INS10_DEG2RAD, 2.0f, 8.0f);
    for (int k = 0; k < 512 * 60; ++k) {
        ins10_thermal_temp(&f, 25.0f + 0.05f * (float)k / 512.0f, 1.0f / 512.0f);
    }
    CHECK(fabsf(f.temp_rate - 0.05f) < 0.003f);
}

static void test_motion_detector(void) {
    sim s;
    sim_defaults(&s, 4);
    s.rate = rate_stop_go;
    typical_errors(&s);
    ins10_fusion f;
    make_fusion(&f, NULL, true);
    long still_at_rest = 0, at_rest = 0, still_moving = 0, moving = 0;
    while (s.t < 120.0) {
        feed(&s, &f);
        double phase = fmod(s.t, 30.0);
        if (phase > 2.5 && phase < 9.5) {
            ++at_rest;
            still_at_rest += f.motion.still;
        } else if (phase > 12.0 && phase < 28.0) {
            ++moving;
            still_moving += f.motion.still;
        }
    }
    printf("    still while at rest %.1f %%, still while moving %.2f %%\n",
           100.0 * still_at_rest / at_rest, 100.0 * still_moving / moving);
    CHECK(still_at_rest > at_rest * 0.9);
    CHECK(still_moving == 0);
}

/* ---- Scenario tests -------------------------------------------------------------- */

static void test_static(void) {
    sim s;
    sim_defaults(&s, 10);
    s.rate = rate_zero;
    typical_errors(&s);
    /* Start tilted and pointing somewhere arbitrary. */
    double v[3] = {0.5, -0.8, 2.0}, a = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    s.q[0] = cos(a / 2);
    for (int i = 0; i < 3; ++i) s.q[i + 1] = sin(a / 2) * v[i] / a;

    ins10_fusion f;
    make_fusion(&f, NULL, true);
    run(&s, &f, 2.0, 0, NULL);
    ins10_output o;
    ins10_fusion_get(&f, &o);
    CHECK(o.flags & INS10_F_ALIGNED);
    CHECK(!(o.flags & INS10_F_COARSE));
    CHECK(o.flags & INS10_F_HEADING);
    stats st = {0};
    run(&s, &f, 60.0, 5.0, &st);
    report("static", &st);
    ins10_fusion_get(&f, &o);
    CHECK(o.flags & INS10_F_STILL);
    CHECK(st.tilt_max < 0.15 && st.head_max < 0.5 && st.bias_max < 0.01);
}

static void test_tumble(void) {
    sim s;
    sim_defaults(&s, 11);
    s.rate = rate_tumble;
    typical_errors(&s);
    ins10_fusion f;
    make_fusion(&f, NULL, true);
    /* Starts while already moving and never rests: coarse alignment, 0.3 dps of
       unknown bias that can only be found from the accelerometer and magnetometer. */
    stats warm = {0}, mid = {0}, st = {0};
    run(&s, &f, 30.0, 4.0, &warm);
    report("tumble, 4-30 s", &warm);
    run(&s, &f, 120.0, 30.0, &mid);
    report("tumble, 30-120 s", &mid);
    run(&s, &f, 400.0, 120.0, &st);
    report("tumble, 120-400 s", &st);
    double n = (double)st.n;
    CHECK(warm.tilt_max < 0.5 && warm.head_max < 3.0);
    CHECK(sqrt(st.tilt_sq / n) < 0.1 && st.tilt_max < 0.3);
    CHECK(sqrt(st.head_sq / n) < 0.3 && st.head_max < 1.0);
    CHECK(st.bias_max < 0.04);
    CHECK((double)st.beyond_3sigma / n < 0.05);
}

static void test_no_mag(void) {
    sim s;
    sim_defaults(&s, 12);
    s.rate = rate_stop_go;
    typical_errors(&s);
    ins10_fusion f;
    make_fusion(&f, NULL, false); /* uncalibrated magnetometer is ignored */
    /* Without a compass the heading is relative to where the device pointed at
       power-up; what matters is how far it drifts from there. */
    double tilt_max = 0.0, drift_max = 0.0, tilt, head, head0 = 0.0;
    bool have_ref = false;
    while (s.t < 300.0) {
        feed(&s, &f);
        if (s.t < 5.0) continue;
        sim_errors(&s, f.ekf.q, &tilt, &head, NULL);
        if (!have_ref) {
            head0 = head;
            have_ref = true;
        }
        double drift = fmod(head - head0 + 540.0, 360.0) - 180.0;
        if (tilt > tilt_max) tilt_max = tilt;
        if (fabs(drift) > drift_max) drift_max = fabs(drift);
    }
    printf("    no magnetometer, 5 min: tilt max %.3f deg, heading drift max %.3f deg\n",
           tilt_max, drift_max);
    ins10_output o;
    ins10_fusion_get(&f, &o);
    CHECK(o.flags & INS10_F_MAG_UNCAL);
    CHECK(!(o.flags & INS10_F_HEADING));
    CHECK(tilt_max < 0.3);
    CHECK(drift_max < 3.0);
}

static void accel_scenario(const ins10_fusion_config *cfg, stats *st, uint64_t seed) {
    sim s;
    sim_defaults(&s, seed);
    s.rate = rate_gentle;
    s.lin_acc = acc_bursts;
    typical_errors(&s);
    ins10_fusion f;
    make_fusion(&f, cfg, true);
    run(&s, &f, 240.0, 10.0, st);
}

static void test_accel_disturbance(void) {
    ins10_fusion_config plain;
    ins10_fusion_default_config(&plain);
    plain.accel_adaptive = false; /* a conventional EKF with fixed noise */
    stats adaptive = {0}, fixed = {0};
    accel_scenario(NULL, &adaptive, 13);
    accel_scenario(&plain, &fixed, 13);
    report("acceleration, adaptive", &adaptive);
    report("acceleration, fixed noise", &fixed);
    CHECK(adaptive.tilt_max < 0.6);
    CHECK(adaptive.tilt_max * 10.0 < fixed.tilt_max);
    CHECK(adaptive.head_max < 1.0);
}

static void mag_scenario(const ins10_fusion_config *cfg, stats *st, uint64_t seed) {
    sim s;
    sim_defaults(&s, seed);
    s.rate = rate_gentle;
    s.mag_dist = mag_bursts;
    typical_errors(&s);
    ins10_fusion f;
    make_fusion(&f, cfg, true);
    run(&s, &f, 320.0, 10.0, st);
}

static void test_mag_disturbance(void) {
    ins10_fusion_config plain;
    ins10_fusion_default_config(&plain);
    plain.mag_adaptive = false;
    plain.mag_yaw_only = false;
    stats adaptive = {0}, fixed = {0};
    mag_scenario(NULL, &adaptive, 14);
    mag_scenario(&plain, &fixed, 14);
    report("magnetic, adaptive", &adaptive);
    report("magnetic, fixed noise", &fixed);
    CHECK(adaptive.head_max < 1.5);
    CHECK(adaptive.head_max * 10.0 < fixed.head_max);
    /* A magnetic disturbance must not reach roll and pitch at all. */
    CHECK(adaptive.tilt_max < 0.2);
}

static void test_thermal_drift(void) {
    /* Pass 1: warm-up with rest periods. The bias moves 0.3 dps; the filter must
       follow it and learn the slope. */
    sim s;
    sim_defaults(&s, 15);
    s.rate = rate_stop_go;
    s.temp = temp_ramp;
    typical_errors(&s);
    const double tempco[3] = {0.015, -0.010, 0.012}; /* dps per deg C */
    for (int i = 0; i < 3; ++i) s.gyro_tempco[i] = tempco[i] * D2R;
    ins10_fusion f;
    make_fusion(&f, NULL, true);
    stats st = {0};
    bool saw_transient = false;
    while (s.t < 500.0) {
        feed(&s, &f);
        if (s.t >= 5.0) accumulate(&st, &s, &f);
        if (s.t > 30.0 && s.t < 60.0) {
            ins10_output o;
            ins10_fusion_get(&f, &o);
            saw_transient |= (o.flags & INS10_F_THERMAL) != 0;
        }
    }
    report("warm-up, learning", &st);
    CHECK(saw_transient);
    /* The vertical component of the bias is held only as tightly as the magnetometer
       can vouch for the absence of rotation during the 10 s rests (see block_update). */
    CHECK(st.bias_max < 0.05 && st.tilt_max < 0.3 && st.head_max < 1.0);
    float slope[3], sigma;
    ins10_thermal_slope(&f.therm, 35.0f, slope, &sigma);
    printf("    learned tempco at 35 C: %.4f %.4f %.4f dps/C (true %.4f %.4f %.4f)\n",
           (double)slope[0] / D2R, (double)slope[1] / D2R, (double)slope[2] / D2R,
           tempco[0], tempco[1], tempco[2]);
    for (int i = 0; i < 3; ++i) CHECK(fabs((double)slope[i] / D2R - tempco[i]) < 0.004);

    /* Pass 2: the same warm-up with no rest at all, with and without the model
       learned in pass 1 (as if reloaded from flash on the next power-up). */
    ins10_thermal_model learned = *ins10_fusion_thermal_model(&f);
    stats with_model = {0}, without = {0};
    for (int pass = 0; pass < 2; ++pass) {
        sim s2;
        sim_defaults(&s2, 16);
        s2.rate = rate_tumble;
        s2.temp = temp_ramp;
        typical_errors(&s2);
        for (int i = 0; i < 3; ++i) s2.gyro_tempco[i] = tempco[i] * D2R;
        ins10_fusion_config c;
        ins10_calibration cal;
        ins10_fusion_default_config(&c);
        ins10_cal_defaults(&cal);
        cal.flags |= INS10_CAL_MAG;
        ins10_fusion f2;
        ins10_fusion_init(&f2, &c, &cal, pass == 0 ? &learned : NULL);
        run(&s2, &f2, 400.0, 40.0, pass == 0 ? &with_model : &without);
    }
    report("warm-up, model reused", &with_model);
    report("warm-up, no model", &without);
    CHECK(with_model.bias_max < without.bias_max);
    CHECK(with_model.bias_max < 0.04);
}

static void test_recovery(void) {
    sim s;
    sim_defaults(&s, 17);
    s.rate = rate_gentle;
    typical_errors(&s);
    ins10_fusion f;
    make_fusion(&f, NULL, true);
    run(&s, &f, 30.0, 0, NULL);
    /* Corrupt the attitude by 40 degrees behind the filter's back, in motion. */
    float kick[3] = {0.5f, 0.45f, 0.2f}, dq[4], q[4];
    ins10_quat_from_rotvec(kick, dq);
    ins10_quat_mul(f.ekf.q, dq, q);
    memcpy(f.ekf.q, q, sizeof(q));
    double tilt, head, t_fixed = -1.0;
    while (s.t < 150.0) {
        feed(&s, &f);
        ins10_output o;
        ins10_fusion_get(&f, &o);
        sim_errors(&s, o.q, &tilt, &head, NULL);
        if (t_fixed < 0 && tilt < 1.0 && fabs(head) < 3.0) t_fixed = s.t - 30.0;
    }
    printf("    40 deg attitude fault while moving: recovered in %.1f s, final tilt %.2f heading %.2f deg, "
           "%u watchdog action(s)\n", t_fixed, tilt, head, (unsigned)f.recoveries);
    CHECK(t_fixed > 0 && t_fixed < 75.0);
    CHECK(tilt < 0.5 && fabs(head) < 1.5);
    CHECK(f.recoveries >= 1);

    /* The same fault at rest. Rest alone proves nothing about linear acceleration, so
       this also waits for the watchdog. */
    sim r;
    sim_defaults(&r, 18);
    r.rate = rate_zero;
    typical_errors(&r);
    make_fusion(&f, NULL, true);
    run(&r, &f, 10.0, 0, NULL);
    ins10_quat_mul(f.ekf.q, dq, q);
    memcpy(f.ekf.q, q, sizeof(q));
    run(&r, &f, 50.0, 0, NULL);
    sim_errors(&r, f.ekf.q, &tilt, NULL, NULL);
    printf("    same fault at rest: tilt %.2f deg after 40 s\n", tilt);
    CHECK(tilt < 0.5);
}

static void test_saturation(void) {
    sim s;
    sim_defaults(&s, 19);
    s.rate = rate_spike;
    typical_errors(&s);
    ins10_fusion f;
    make_fusion(&f, NULL, true);
    run(&s, &f, 20.0, 0, NULL);
    bool flagged = false;
    double worst = 0.0, tilt, head;
    while (s.t < 21.0) {
        feed(&s, &f);
        ins10_output o;
        ins10_fusion_get(&f, &o);
        flagged |= (o.flags & INS10_F_GYRO_SAT) != 0;
        sim_errors(&s, o.q, NULL, NULL, &tilt);
        if (tilt > worst) worst = tilt;
    }
    stats st = {0};
    run(&s, &f, 60.0, 30.0, &st);
    sim_errors(&s, f.ekf.q, &tilt, &head, NULL);
    printf("    spin beyond 2000 dps: error peaked at %.1f deg\n", worst);
    report("after saturation", &st);
    CHECK(flagged);
    CHECK(st.tilt_max < 0.3 && st.head_max < 1.0);
}

/* Everything at once, with the sensor imperfections that survive a real calibration:
   accelerometer and magnetometer residuals, gyro scale error, warm-up drift, tumbling
   with rest periods, bursts of acceleration and of magnetic disturbance. The numbers
   printed here are the honest expectation for a board; the ideal-sensor tests above
   are not. Two cases: gyro scale calibrated by the host tool (0.1 % left) and the
   factory tolerance (0.5 %). The second is dominated by scale error accumulating
   while tumbling through an acceleration burst, when nothing can correct it. */
static void realistic_case(double gyro_scale, stats *st) {
    sim s;
    sim_defaults(&s, 20);
    s.rate = rate_stop_go;
    s.lin_acc = acc_bursts;
    s.mag_dist = mag_bursts;
    s.temp = temp_ramp;
    typical_errors(&s);
    s.gyro_scale = gyro_scale;
    for (int i = 0; i < 3; ++i) s.gyro_tempco[i] = (i == 1 ? -0.010 : 0.015) * D2R;
    s.accel_bias[0] = 0.003;
    s.accel_bias[1] = -0.002;
    s.accel_bias[2] = 0.004;
    s.accel_scale = 0.002;
    s.mag_bias[0] = 0.4;
    s.mag_bias[1] = -0.3;
    s.mag_bias[2] = 0.5;
    s.mag_scale[0] = 0.01;
    s.mag_scale[1] = -0.008;
    s.mag_scale[2] = 0.012;
    ins10_fusion_config c;
    ins10_fusion_default_config(&c);
    c.gyro_scale_err = (float)gyro_scale;
    ins10_fusion f;
    make_fusion(&f, &c, true);
    run(&s, &f, 600.0, 5.0, st);
}

static void test_realistic(void) {
    stats cal = {0}, factory = {0};
    realistic_case(0.001, &cal);
    realistic_case(0.005, &factory);
    report("realistic, gyro calibrated", &cal);
    report("realistic, gyro factory", &factory);
    double n = (double)cal.n;
    CHECK(sqrt(cal.tilt_sq / n) < 0.5 && cal.tilt_max < 1.2);
    CHECK(sqrt(cal.head_sq / n) < 1.5 && cal.head_max < 3.5);
    CHECK(cal.bias_max < 0.1);
    CHECK(factory.tilt_max < 4.0 && factory.head_max < 6.0);
}

/* ---- Audit scenarios: cases the tests above do not reach -------------------------- */

static double slow_rate_dps;
static int slow_axis;

/* At rest for 20 s, then a perfectly smooth, very slow rotation about one body axis:
   to the gyro alone this is indistinguishable from a bias. */
static void rate_slow(double t, double w[3]) {
    w[0] = w[1] = w[2] = 0.0;
    w[slow_axis] = slow_rate_dps * D2R * smooth_gate(t, 20.0, 1e12, 2.0);
}

static void test_slow_rotation(void) {
    static const struct { int axis; double rate; const char *label; } cases[] = {
        {2, 0.30, "yaw 0.30 dps"}, {2, 0.10, "yaw 0.10 dps"},
        {0, 0.12, "tilt 0.12 dps"}, {1, 0.04, "tilt 0.04 dps"},
    };
    for (unsigned k = 0; k < sizeof(cases) / sizeof(cases[0]); ++k) {
        sim s;
        sim_defaults(&s, 30 + k);
        slow_axis = cases[k].axis;
        slow_rate_dps = cases[k].rate;
        s.rate = rate_slow;
        typical_errors(&s);
        ins10_fusion f;
        make_fusion(&f, NULL, true);
        stats st = {0};
        run(&s, &f, 260.0, 5.0, &st);
        report(cases[k].label, &st);
        CHECK(st.tilt_max < 0.5);
        CHECK(st.head_max < 1.5);
        CHECK(st.bias_max < 0.08);
    }
}

static void test_declination_and_hemisphere(void) {
    /* True north differs from magnetic north by the declination; the truth of the
       simulator is in true ENU. */
    const double decl = 12.0;
    stats with = {0}, without = {0}, south = {0};
    for (int pass = 0; pass < 3; ++pass) {
        sim s;
        sim_defaults(&s, 40);
        s.rate = rate_stop_go;
        typical_errors(&s);
        double horiz = 50.0 * cos(60.0 * D2R), az = pass == 2 ? 0.0 : decl * D2R;
        s.mag_field[0] = horiz * sin(az);
        s.mag_field[1] = horiz * cos(az);
        s.mag_field[2] = (pass == 2 ? 1.0 : -1.0) * 50.0 * sin(60.0 * D2R); /* south: field points up */
        ins10_fusion_config c;
        ins10_calibration cal;
        ins10_fusion_default_config(&c);
        ins10_cal_defaults(&cal);
        cal.flags |= INS10_CAL_MAG;
        if (pass == 0) cal.declination_deg = (float)decl;
        ins10_fusion f;
        ins10_fusion_init(&f, &c, &cal, NULL);
        run(&s, &f, 120.0, 5.0, pass == 0 ? &with : pass == 1 ? &without : &south);
        if (pass == 2) {
            ins10_output o;
            ins10_fusion_get(&f, &o);
            CHECK(fabs((double)o.mag_dip_deg + 60.0) < 1.0);
        }
    }
    report("declination 12 deg, set", &with);
    report("declination 12 deg, unset", &without);
    report("southern hemisphere", &south);
    CHECK(with.head_max < 1.0);
    CHECK(without.head_max > 11.0 && without.head_max < 13.5); /* exactly the declination */
    CHECK(south.head_max < 1.0 && south.tilt_max < 0.3);
}

/* Power-up in awkward attitudes: an axis straight up or down, upside down. */
static void test_start_attitudes(void) {
    static const double starts[][3] = {
        {0.0, PI / 2, 0.0}, {0.0, -PI / 2, 0.0}, {PI, 0.0, 0.0}, {PI / 2, 0.0, 0.0},
        {0.0, PI / 2 - 0.02, 1.0}, {2.5, 0.7, -2.0},
    };
    double worst_tilt = 0.0, worst_head = 0.0;
    for (unsigned k = 0; k < sizeof(starts) / sizeof(starts[0]); ++k) {
        sim s;
        sim_defaults(&s, 50 + k);
        s.rate = rate_gentle;
        typical_errors(&s);
        /* roll, pitch, yaw */
        double cr = cos(starts[k][0] / 2), sr = sin(starts[k][0] / 2);
        double cp = cos(starts[k][1] / 2), sp = sin(starts[k][1] / 2);
        double cy = cos(starts[k][2] / 2), sy = sin(starts[k][2] / 2);
        s.q[0] = cy * cp * cr + sy * sp * sr;
        s.q[1] = cy * cp * sr - sy * sp * cr;
        s.q[2] = cy * sp * cr + sy * cp * sr;
        s.q[3] = sy * cp * cr - cy * sp * sr;
        ins10_fusion f;
        make_fusion(&f, NULL, true);
        stats st = {0};
        run(&s, &f, 30.0, 2.5, &st);
        if (st.tilt_max > worst_tilt) worst_tilt = st.tilt_max;
        if (st.head_max > worst_head) worst_head = st.head_max;
    }
    printf("    six start attitudes: worst tilt %.3f deg, worst heading %.3f deg\n", worst_tilt, worst_head);
    CHECK(worst_tilt < 0.3 && worst_head < 1.0);
}

/* Calibration uploaded while the filter is running, as the host tool does. */
static void test_live_calibration(void) {
    sim s;
    sim_defaults(&s, 60);
    s.rate = rate_gentle;
    typical_errors(&s);
    s.mag_bias[0] = 18.0;
    s.mag_bias[1] = -12.0;
    s.mag_bias[2] = 9.0;
    s.accel_bias[0] = 0.02;
    s.accel_bias[1] = -0.015;
    ins10_fusion f;
    make_fusion(&f, NULL, false);
    run(&s, &f, 40.0, 0, NULL);
    ins10_output o;
    ins10_fusion_get(&f, &o);
    CHECK(o.flags & INS10_F_MAG_UNCAL);

    /* imucal result arrives. */
    ins10_calibration cal = f.cal;
    cal.accel.bias[0] = 0.02f;
    cal.accel.bias[1] = -0.015f;
    cal.flags |= INS10_CAL_ACCEL;
    ins10_fusion_set_calibration(&f, &cal);
    stats accel = {0};
    run(&s, &f, 46.0, 43.0, &accel);
    report("after accel calibration", &accel);
    CHECK(accel.tilt_max < 0.3);

    /* magcal result arrives: first use of the magnetometer. */
    cal.mag.bias[0] = 18.0f;
    cal.mag.bias[1] = -12.0f;
    cal.mag.bias[2] = 9.0f;
    cal.flags |= INS10_CAL_MAG;
    ins10_fusion_set_calibration(&f, &cal);
    stats first = {0};
    run(&s, &f, 52.0, 47.0, &first);
    report("after first mag calibration", &first);
    CHECK(first.head_max < 1.5);

    /* The surroundings change (new hard iron); the filter lives with the stale
       calibration for minutes, long enough to adopt the distorted field as its
       reference. Then the user calibrates again. */
    s.mag_bias[0] = -6.0;
    s.mag_bias[2] = 20.0;
    run(&s, &f, 240.0, 0, NULL);
    cal.mag.bias[0] = -6.0f;
    cal.mag.bias[2] = 20.0f;
    ins10_fusion_set_calibration(&f, &cal);
    stats again = {0};
    run(&s, &f, 246.0, 241.0, &again);
    report("after mag re-calibration", &again);
    CHECK(again.head_max < 1.5 && again.tilt_max < 0.3);
}

/* Broadband vibration: white accelerometer noise of 0.2 g per sample. Gravity is still
   in there on average and must keep being used. */
static void test_vibration(void) {
    sim s;
    sim_defaults(&s, 70);
    s.rate = rate_gentle;
    s.accel_noise = 0.2;
    typical_errors(&s);
    ins10_fusion f;
    make_fusion(&f, NULL, true);
    /* Nothing ever looks still, so the 0.3 dps of bias has to be found from gravity
       and the compass through the vibration; allow two minutes for that. */
    stats early = {0}, st = {0};
    run(&s, &f, 120.0, 10.0, &early);
    run(&s, &f, 300.0, 120.0, &st);
    report("vibration 0.2 g, 10-120 s", &early);
    report("vibration 0.2 g, 2-5 min", &st);
    CHECK(early.tilt_max < 1.5 && early.head_max < 3.0);
    CHECK(st.tilt_max < 1.0 && st.head_max < 2.0 && st.bias_max < 0.05);

    /* And with the gyro as the only good sensor it must not be worse than gyro-only:
       tilt would drift without bound if gravity were simply ignored. */
    sim s2;
    sim_defaults(&s2, 71);
    s2.rate = rate_gentle;
    s2.accel_noise = 0.2;
    typical_errors(&s2);
    s2.gyro_bias[0] = 0.0;
    make_fusion(&f, NULL, true);
    stats late = {0};
    run(&s2, &f, 900.0, 600.0, &late);
    report("vibration, minutes 10-15", &late);
    CHECK(late.tilt_max < 1.0);
}

/* Lost samples, free fall, a hard shock, and an hour of operation. */
static void test_robustness(void) {
    sim s;
    sim_defaults(&s, 80);
    s.rate = rate_tumble;
    typical_errors(&s);
    ins10_fusion f;
    make_fusion(&f, NULL, true);
    run(&s, &f, 60.0, 0, NULL);
    /* 0.3 s of IMU data lost while tumbling: the truth moves on, the filter is told. */
    sim_sample x;
    while (s.t < 60.3) sim_step(&s, &x);
    ins10_fusion_gap(&f, 0.3f);
    stats gap = {0};
    run(&s, &f, 75.0, 63.0, &gap);
    report("after a 0.3 s gap", &gap);
    CHECK(gap.tilt_max < 0.5 && gap.head_max < 1.5);

    /* Garbage in: non-finite and absurd samples must not poison the state. */
    float nan_v[3] = {NAN, 0.0f, 0.0f}, zero[3] = {0.0f, 0.0f, 0.0f}, huge[3] = {1e30f, -1e30f, 1e30f};
    ins10_fusion_imu(&f, nan_v, zero, 25.0f, 1.0f / 512.0f, false);
    ins10_fusion_imu(&f, zero, nan_v, 25.0f, 1.0f / 512.0f, false);
    ins10_fusion_mag(&f, nan_v, 0.0f);
    for (int i = 0; i < 20; ++i) ins10_fusion_imu(&f, huge, huge, 25.0f, 1.0f / 512.0f, true);
    ins10_fusion_mag(&f, huge, 0.0f);
    ins10_output o;
    ins10_fusion_get(&f, &o);
    bool finite = true;
    for (int i = 0; i < 4; ++i) finite = finite && isfinite(o.q[i]);
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) finite = finite && isfinite(f.ekf.P[i][j]);
    }
    CHECK(finite);
    CHECK(isfinite(o.tilt_sigma_deg) && isfinite(o.heading_sigma_deg));
    stats after = {0};
    run(&s, &f, 140.0, 110.0, &after);
    report("after garbage input", &after);
    CHECK(after.tilt_max < 0.5 && after.head_max < 1.5);

    /* One hour: no numerical drift, covariance stays symmetric and positive. */
    sim h;
    sim_defaults(&h, 81);
    h.rate = rate_stop_go;
    h.temp = temp_ramp;
    typical_errors(&h);
    make_fusion(&f, NULL, true);
    stats hour = {0};
    run(&h, &f, 3600.0, 3000.0, &hour);
    report("minutes 50-60 of an hour", &hour);
    double asym = 0.0, min_diag = 1.0;
    for (int i = 0; i < 6; ++i) {
        if ((double)f.ekf.P[i][i] < min_diag) min_diag = (double)f.ekf.P[i][i];
        for (int j = 0; j < 6; ++j) {
            double d = fabs((double)f.ekf.P[i][j] - (double)f.ekf.P[j][i]);
            if (d > asym) asym = d;
        }
    }
    CHECK(asym == 0.0 && min_diag > 0.0);
    CHECK(hour.tilt_max < 0.3 && hour.head_max < 1.0 && hour.bias_max < 0.03);
    double qn = 0.0;
    for (int i = 0; i < 4; ++i) qn += (double)f.ekf.q[i] * (double)f.ekf.q[i];
    CHECK(fabs(qn - 1.0) < 1e-5);
}

/* ---- Registry -------------------------------------------------------------------- */

typedef struct {
    const char *name;
    void (*run)(void);
} test_case;

static const test_case tests[] = {
    {"math", test_math},
    {"integrator", test_integrator},
    {"calibration", test_calibration},
    {"persist", test_persist},
    {"thermal_model", test_thermal_model},
    {"motion_detector", test_motion_detector},
    {"static", test_static},
    {"tumble", test_tumble},
    {"no_mag", test_no_mag},
    {"accel_disturbance", test_accel_disturbance},
    {"mag_disturbance", test_mag_disturbance},
    {"thermal_drift", test_thermal_drift},
    {"recovery", test_recovery},
    {"saturation", test_saturation},
    {"realistic", test_realistic},
    {"slow_rotation", test_slow_rotation},
    {"declination_hemisphere", test_declination_and_hemisphere},
    {"start_attitudes", test_start_attitudes},
    {"live_calibration", test_live_calibration},
    {"vibration", test_vibration},
    {"robustness", test_robustness},
};

int main(int argc, char **argv) {
    int ran = 0;
    for (size_t i = 0; i < sizeof(tests) / sizeof(tests[0]); ++i) {
        if (argc > 1 && strcmp(argv[1], tests[i].name) != 0) continue;
        int before = failures;
        printf("[ RUN  ] %s\n", tests[i].name);
        tests[i].run();
        printf("[ %s ] %s\n", failures == before ? " OK " : "FAIL", tests[i].name);
        ++ran;
    }
    if (ran == 0) {
        printf("no test named '%s'\n", argc > 1 ? argv[1] : "");
        return 2;
    }
    printf("%d test(s), %d failed check(s)\n", ran, failures);
    return failures ? 1 : 0;
}
