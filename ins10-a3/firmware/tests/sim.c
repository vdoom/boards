#include "sim.h"
#include <math.h>
#include <string.h>

#define PI 3.14159265358979323846
#define SUBSTEPS 8

static void qmul(const double a[4], const double b[4], double o[4]) {
    double w = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
    double x = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
    double y = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
    double z = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
    o[0] = w; o[1] = x; o[2] = y; o[3] = z;
}

static void qexp(const double v[3], double q[4]) {
    double a = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    double s = a > 1e-12 ? sin(0.5 * a) / a : 0.5;
    q[0] = cos(0.5 * a); q[1] = s * v[0]; q[2] = s * v[1]; q[3] = s * v[2];
}

static void qdcm(const double q[4], double r[9]) {
    double w = q[0], x = q[1], y = q[2], z = q[3];
    r[0] = 1 - 2 * (y * y + z * z); r[1] = 2 * (x * y - w * z); r[2] = 2 * (x * z + w * y);
    r[3] = 2 * (x * y + w * z); r[4] = 1 - 2 * (x * x + z * z); r[5] = 2 * (y * z - w * x);
    r[6] = 2 * (x * z - w * y); r[7] = 2 * (y * z + w * x); r[8] = 1 - 2 * (x * x + y * y);
}

/* body = R^T nav */
static void to_body(const double r[9], const double n[3], double b[3]) {
    for (int i = 0; i < 3; ++i) b[i] = r[i] * n[0] + r[3 + i] * n[1] + r[6 + i] * n[2];
}

static double uniform(sim *s) {
    s->rng ^= s->rng >> 12;
    s->rng ^= s->rng << 25;
    s->rng ^= s->rng >> 27;
    uint64_t v = s->rng * 2685821657736338717ull;
    return ((double)(v >> 11) + 0.5) / 9007199254740992.0;
}

double sim_gauss(sim *s) {
    double u1 = uniform(s), u2 = uniform(s);
    return sqrt(-2.0 * log(u1)) * cos(2.0 * PI * u2);
}

void sim_defaults(sim *s, uint64_t seed) {
    memset(s, 0, sizeof(*s));
    s->fs = 512.0;
    s->mag_div = 10;
    s->mag_delay = 2;
    /* Noise density times sqrt(fs / 2): 0.0028 dps/rtHz and 70 ug/rtHz. */
    s->gyro_noise = 0.045 * PI / 180.0;
    s->accel_noise = 0.0011;
    s->mag_noise = 0.05;
    s->gyro_limit = 2000.0 * PI / 180.0;
    /* 50 uT with 66 degrees of dip, pointing north and down. */
    s->mag_field[0] = 0.0;
    s->mag_field[1] = 50.0 * cos(66.0 * PI / 180.0);
    s->mag_field[2] = -50.0 * sin(66.0 * PI / 180.0);
    s->q[0] = 1.0;
    s->rng = seed * 0x9e3779b97f4a7c15ull + 0x1234567ull;
    for (int i = 0; i < SIM_HISTORY; ++i) s->q_hist[i][0] = 1.0;
}

void sim_true_bias(const sim *s, double bias[3]) {
    double temp = s->temp ? s->temp(s->t) : 25.0;
    for (int i = 0; i < 3; ++i) bias[i] = s->gyro_bias[i] + s->gyro_tempco[i] * (temp - 25.0);
}

void sim_step(sim *s, sim_sample *out) {
    double dt = 1.0 / s->fs, h = dt / SUBSTEPS;
    for (int i = 0; i < SUBSTEPS; ++i) {
        double w[3], dq[4], q[4];
        s->rate(s->t + (i + 0.5) * h, w);
        double v[3] = {w[0] * h, w[1] * h, w[2] * h};
        qexp(v, dq);
        qmul(s->q, dq, q);
        double n = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        for (int j = 0; j < 4; ++j) s->q[j] = q[j] / n;
    }
    ++s->k;
    s->t = (double)s->k * dt;
    memcpy(s->q_hist[s->k % SIM_HISTORY], s->q, sizeof(s->q));

    double temp = s->temp ? s->temp(s->t) : 25.0;
    double w[3], bias[3], r[9], f_nav[3] = {0.0, 0.0, 1.0}, f_body[3];
    s->rate(s->t, w);
    sim_true_bias(s, bias);
    out->saturated = false;
    for (int i = 0; i < 3; ++i) {
        double g = w[i] * (1.0 + s->gyro_scale) + bias[i] + s->gyro_noise * sim_gauss(s);
        if (fabs(g) > s->gyro_limit) {
            g = g > 0 ? s->gyro_limit : -s->gyro_limit;
            out->saturated = true;
        }
        out->gyro_dps[i] = (float)(g * 180.0 / PI);
    }
    if (s->lin_acc) {
        double a[3];
        s->lin_acc(s->t, a);
        for (int i = 0; i < 3; ++i) f_nav[i] += a[i];
    }
    qdcm(s->q, r);
    to_body(r, f_nav, f_body);
    for (int i = 0; i < 3; ++i) {
        out->accel_g[i] = (float)(f_body[i] * (1.0 + s->accel_scale) + s->accel_bias[i] +
                                  s->accel_noise * sim_gauss(s));
    }
    out->temp_c = (float)(temp + 0.02 * sim_gauss(s));

    out->mag_valid = s->mag_div > 0 && s->k % s->mag_div == 0;
    if (out->mag_valid) {
        int delay = s->mag_delay < SIM_HISTORY ? s->mag_delay : SIM_HISTORY - 1;
        long past = s->k - delay;
        double t_past = (double)past * dt;
        double m_nav[3] = {s->mag_field[0], s->mag_field[1], s->mag_field[2]}, m_body[3];
        if (s->mag_dist) {
            double d[3];
            s->mag_dist(t_past, d);
            for (int i = 0; i < 3; ++i) m_nav[i] += d[i];
        }
        qdcm(s->q_hist[((past % SIM_HISTORY) + SIM_HISTORY) % SIM_HISTORY], r);
        to_body(r, m_nav, m_body);
        for (int i = 0; i < 3; ++i) {
            out->mag_ut[i] = (float)(m_body[i] * (1.0 + s->mag_scale[i]) + s->mag_bias[i] +
                                     s->mag_noise * sim_gauss(s));
        }
        out->mag_age_s = (float)(delay * dt);
    }
}

void sim_errors(const sim *s, const float q_est[4], double *tilt, double *heading, double *total) {
    double qe[4] = {q_est[0], q_est[1], q_est[2], q_est[3]}, rt[9], re[9];
    qdcm(s->q, rt);
    qdcm(qe, re);
    /* Tilt: angle between the true and estimated up vectors (third rows). */
    double dot = rt[6] * re[6] + rt[7] * re[7] + rt[8] * re[8];
    if (dot > 1.0) dot = 1.0;
    if (dot < -1.0) dot = -1.0;
    if (tilt) *tilt = acos(dot) * 180.0 / PI;
    /* Heading: the rotation about the vertical contained in E = R_est R_true^T. */
    double e00 = 0, e01 = 0, e10 = 0, e11 = 0;
    for (int k = 0; k < 3; ++k) {
        e00 += re[k] * rt[k];
        e01 += re[k] * rt[3 + k];
        e10 += re[3 + k] * rt[k];
        e11 += re[3 + k] * rt[3 + k];
    }
    if (heading) *heading = atan2(e10 - e01, e00 + e11) * 180.0 / PI;
    double qc[4] = {s->q[0], -s->q[1], -s->q[2], -s->q[3]}, qd[4];
    qmul(qc, qe, qd);
    /* atan2 of the vector part: acos(w) loses all precision near zero error. */
    double vec = sqrt(qd[1] * qd[1] + qd[2] * qd[2] + qd[3] * qd[3]);
    if (total) *total = 2.0 * atan2(vec, fabs(qd[0])) * 180.0 / PI;
}
