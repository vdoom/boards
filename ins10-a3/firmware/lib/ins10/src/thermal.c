#include "ins10/thermal.h"
#include "ins10/math3.h"
#include <string.h>

/* Turn-on bias repeatability assumed when a stored model is reused, and the prior for
   a part that was never observed. ICM-42688-P: +/-0.5 dps initial zero-rate output. */
#define TURN_ON_SIGMA (0.1f * INS10_DEG2RAD)
#define FIRST_SIGMA   (0.6f * INS10_DEG2RAD)

static float regressor(float temp_c) {
    return ins10_clampf((temp_c - INS10_THERMAL_T0) / INS10_THERMAL_SCALE, -8.0f, 8.0f);
}

void ins10_thermal_model_defaults(ins10_thermal_model *m, float slope_prior) {
    memset(m, 0, sizeof(*m));
    float s1 = slope_prior * INS10_THERMAL_SCALE; /* per unit of u */
    float s2 = 0.25f * s1;                        /* curvature prior, per u^2 */
    m->cov[0] = FIRST_SIGMA * FIRST_SIGMA;
    m->cov[3] = s1 * s1;
    m->cov[5] = s2 * s2;
}

static bool model_usable(const ins10_thermal_model *m) {
    if (m->count == 0) return false;
    for (int i = 0; i < 6; ++i) {
        if (!isfinite(m->cov[i])) return false;
    }
    for (int a = 0; a < 3; ++a) {
        for (int k = 0; k < 3; ++k) {
            if (!isfinite(m->coef[a][k]) || fabsf(m->coef[a][k]) > 1.0f) return false;
        }
    }
    return m->cov[0] > 0.0f && m->cov[3] > 0.0f && m->cov[5] > 0.0f;
}

void ins10_thermal_init(ins10_thermal *t, const ins10_thermal_model *stored,
                        float slope_prior, float tau_temp, float tau_rate) {
    memset(t, 0, sizeof(*t));
    t->tau_temp = tau_temp;
    t->tau_rate = tau_rate;
    if (stored && model_usable(stored)) {
        t->model = *stored;
        t->model.cov[0] += TURN_ON_SIGMA * TURN_ON_SIGMA;
    } else {
        ins10_thermal_model_defaults(&t->model, slope_prior);
    }
}

void ins10_thermal_rebase(ins10_thermal *t, bool slopes_changed, float slope_prior) {
    if (slopes_changed) {
        ins10_thermal_model_defaults(&t->model, slope_prior);
    } else {
        t->model.cov[0] += FIRST_SIGMA * FIRST_SIGMA;
    }
}

void ins10_thermal_temp(ins10_thermal *t, float temp_c, float dt) {
    if (!isfinite(temp_c) || !(dt > 0.0f)) return;
    if (!t->temp_valid) {
        t->temp = temp_c;
        t->temp_rate = 0.0f;
        t->temp_valid = true;
        return;
    }
    float a = dt / (t->tau_temp + dt), b = dt / (t->tau_rate + dt);
    float step = a * (temp_c - t->temp);
    t->temp += step;
    t->temp_rate += b * (step / dt - t->temp_rate);
}

void ins10_thermal_observe(ins10_thermal *t, const float bias[3], float temp_c,
                           float var, float bias_rw_var) {
    ins10_thermal_model *m = &t->model;
    float u = regressor(temp_c);
    float h[3] = {1.0f, u, u * u};
    float *c = m->cov;
    c[0] += bias_rw_var;
    /* P h^T for the packed symmetric matrix. */
    float ph[3] = {
        c[0] * h[0] + c[1] * h[1] + c[2] * h[2],
        c[1] * h[0] + c[3] * h[1] + c[4] * h[2],
        c[2] * h[0] + c[4] * h[1] + c[5] * h[2],
    };
    float s = h[0] * ph[0] + h[1] * ph[1] + h[2] * ph[2] + var;
    if (!(s > 0.0f)) return;
    float k[3] = {ph[0] / s, ph[1] / s, ph[2] / s};
    for (int a = 0; a < 3; ++a) {
        float innov = bias[a] - (m->coef[a][0] + m->coef[a][1] * u + m->coef[a][2] * u * u);
        for (int i = 0; i < 3; ++i) m->coef[a][i] += k[i] * innov;
    }
    c[0] -= k[0] * ph[0];
    c[1] -= k[0] * ph[1];
    c[2] -= k[0] * ph[2];
    c[3] -= k[1] * ph[1];
    c[4] -= k[1] * ph[2];
    c[5] -= k[2] * ph[2];
    static const int diagonal[3] = {0, 3, 5};
    for (int i = 0; i < 3; ++i) {
        if (!(c[diagonal[i]] > 1e-16f)) c[diagonal[i]] = 1e-16f;
    }
    if (m->count == 0) {
        m->t_min = m->t_max = temp_c;
    } else {
        if (temp_c < m->t_min) m->t_min = temp_c;
        if (temp_c > m->t_max) m->t_max = temp_c;
    }
    if (m->count < UINT32_MAX) ++m->count;
}

void ins10_thermal_predict(const ins10_thermal *t, float temp_c, float bias[3]) {
    float u = regressor(temp_c);
    for (int a = 0; a < 3; ++a) {
        bias[a] = t->model.coef[a][0] + t->model.coef[a][1] * u + t->model.coef[a][2] * u * u;
    }
}

void ins10_thermal_slope(const ins10_thermal *t, float temp_c, float slope[3], float *sigma) {
    const ins10_thermal_model *m = &t->model;
    float u = regressor(temp_c);
    float inv = 1.0f / INS10_THERMAL_SCALE;
    for (int a = 0; a < 3; ++a) slope[a] = (m->coef[a][1] + 2.0f * m->coef[a][2] * u) * inv;
    float var = m->cov[3] + 4.0f * u * m->cov[4] + 4.0f * u * u * m->cov[5];
    *sigma = sqrtf(var > 0.0f ? var : 0.0f) * inv;
}
