#include "ins10/ekf.h"
#include "ins10/math3.h"
#include <string.h>

#define N INS10_EKF_N
#define BIAS_LIMIT 0.35f /* rad/s, about 20 dps: far beyond any real ICM-42688 bias */
#define VAR_FLOOR 1e-14f
#define ATT_VAR_MAX 40.0f /* rad^2: "completely unknown" */
#define BIAS_VAR_MAX 0.1f

void ins10_ekf_up_body(const ins10_ekf *e, float up[3]) {
    float w = e->q[0], x = e->q[1], y = e->q[2], z = e->q[3];
    up[0] = 2.0f * (x * z - w * y);
    up[1] = 2.0f * (y * z + w * x);
    up[2] = 1.0f - 2.0f * (x * x + y * y);
}

/* Adds R^T diag(tilt, tilt, yaw) R to the attitude block. */
void ins10_ekf_add_attitude_var(ins10_ekf *e, float var_tilt, float var_yaw) {
    float n[3];
    ins10_ekf_up_body(e, n);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            e->P[i][j] += (var_yaw - var_tilt) * n[i] * n[j];
        }
        e->P[i][i] += var_tilt;
    }
}

void ins10_ekf_add_bias_var(ins10_ekf *e, float var) {
    for (int i = 3; i < N; ++i) e->P[i][i] += var;
}

static void condition(ins10_ekf *e) {
    for (int i = 0; i < N; ++i) {
        float limit = i < 3 ? ATT_VAR_MAX : BIAS_VAR_MAX;
        if (!(e->P[i][i] > VAR_FLOOR)) e->P[i][i] = VAR_FLOOR;
        if (e->P[i][i] > limit) {
            /* Scale the row and column so the matrix stays positive semi-definite. */
            float s = sqrtf(limit / e->P[i][i]);
            for (int j = 0; j < N; ++j) {
                e->P[i][j] *= s;
                e->P[j][i] *= s;
            }
        }
    }
}

void ins10_ekf_init(ins10_ekf *e, const float q[4], const float bias[3],
                    float var_tilt, float var_yaw, float var_bias) {
    memset(e, 0, sizeof(*e));
    memcpy(e->q, q, sizeof(e->q));
    ins10_quat_normalize(e->q);
    if (bias) memcpy(e->bias, bias, sizeof(e->bias));
    ins10_ekf_add_attitude_var(e, var_tilt, var_yaw);
    ins10_ekf_add_bias_var(e, var_bias);
}

void ins10_ekf_integrate(ins10_ekf *e, const float rate[3], float dt) {
    /* Rotation vector over the last sample interval, from samples of the
       instantaneous rate at its ends (w0, w1) and one sample earlier (wm):
         integral of w  ~  dt (5 w1 + 8 w0 - wm) / 12     (third-order Adams-Moulton)
         commutation    ~  dt^2 (w0 x w1) / 12            (second Magnus term)
       Together they leave an O(dt^4) error per step. On 5 Hz coning sampled at
       512 Hz this is about 300 times more accurate than the trapezoid rule alone. */
    float w1[3], phi[3], dq[4], q[4];
    for (int i = 0; i < 3; ++i) w1[i] = rate[i] - e->bias[i];
    if (e->prev_count >= 1) {
        float w0[3], c[3];
        for (int i = 0; i < 3; ++i) w0[i] = e->prev_rate[0][i] - e->bias[i];
        ins10_cross(w0, w1, c);
        for (int i = 0; i < 3; ++i) {
            float sum;
            if (e->prev_count >= 2) {
                float wm = e->prev_rate[1][i] - e->bias[i];
                sum = (5.0f * w1[i] + 8.0f * w0[i] - wm) * (1.0f / 12.0f);
            } else {
                sum = 0.5f * (w0[i] + w1[i]);
            }
            phi[i] = dt * sum + (dt * dt * (1.0f / 12.0f)) * c[i];
        }
    } else {
        for (int i = 0; i < 3; ++i) phi[i] = w1[i] * dt;
    }
    ins10_quat_from_rotvec(phi, dq);
    ins10_quat_mul(e->q, dq, q);
    ins10_quat_normalize(q);
    memcpy(e->q, q, sizeof(q));
    for (int i = 0; i < 3; ++i) {
        e->prev_rate[1][i] = e->prev_rate[0][i];
        e->prev_rate[0][i] = rate[i];
        e->dtheta_acc[i] += phi[i];
    }
    if (e->prev_count < 2) ++e->prev_count;
    e->dt_acc += dt;
}

void ins10_ekf_propagate(ins10_ekf *e, float q_att, float q_bias) {
    /* x' = F x,  F = [Phi, -T I; 0, I],  Phi = exp(dtheta)^T. */
    float d[9], phi[9], a[9], b[9], c[9], pb[9], pa[9];
    float t = e->dt_acc;
    ins10_rotvec_to_dcm(e->dtheta_acc, d);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            phi[3 * i + j] = d[3 * j + i];
            a[3 * i + j] = e->P[i][j];
            b[3 * i + j] = e->P[i][j + 3];
            c[3 * i + j] = e->P[i + 3][j + 3];
        }
    }
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            float sb = 0.0f, sa = 0.0f;
            for (int k = 0; k < 3; ++k) {
                sb += phi[3 * i + k] * b[3 * k + j];
                sa += phi[3 * i + k] * a[3 * k + j];
            }
            pb[3 * i + j] = sb;
            pa[3 * i + j] = sa;
        }
    }
    for (int i = 0; i < 3; ++i) {
        for (int j = i; j < 3; ++j) {
            float s = 0.0f;
            for (int k = 0; k < 3; ++k) s += pa[3 * i + k] * phi[3 * j + k];
            s += -t * (pb[3 * i + j] + pb[3 * j + i]) + t * t * c[3 * i + j];
            if (i == j) s += q_att;
            e->P[i][j] = e->P[j][i] = s;
        }
        for (int j = 0; j < 3; ++j) {
            float s = pb[3 * i + j] - t * c[3 * i + j];
            e->P[i][j + 3] = e->P[j + 3][i] = s;
        }
        e->P[i + 3][i + 3] += q_bias;
    }
    condition(e);
    e->dtheta_acc[0] = e->dtheta_acc[1] = e->dtheta_acc[2] = 0.0f;
    e->dt_acc = 0.0f;
}

void ins10_ekf_shift_bias(ins10_ekf *e, const float delta[3]) {
    for (int i = 0; i < 3; ++i) {
        e->bias[i] = ins10_clampf(e->bias[i] + delta[i], -BIAS_LIMIT, BIAS_LIMIT);
    }
}

/* One scalar measurement z = h x + v, var(v) = r_var, with the error state accumulated
   in dx instead of being injected immediately. axis, when given, restricts the gain of
   both state blocks to that body direction. The covariance update is the Joseph form
   written out for a scalar, which stays valid for a modified (sub-optimal) gain:
     P <- P - K v^T - v K^T + s K K^T,   v = P h^T,  s = h v + r. */
static void scalar_update(ins10_ekf *e, const float h[N], float resid, float r_var,
                          const float *axis, float dx[N]) {
    float v[N], k[N];
    float s = r_var;
    for (int i = 0; i < N; ++i) {
        float sum = 0.0f;
        for (int j = 0; j < N; ++j) sum += e->P[i][j] * h[j];
        v[i] = sum;
        s += h[i] * sum;
    }
    if (!(s > 1e-20f)) return;
    float inv = 1.0f / s;
    for (int i = 0; i < N; ++i) k[i] = v[i] * inv;
    if (axis) {
        float ka = ins10_dot(k, axis), kb = ins10_dot(k + 3, axis);
        for (int i = 0; i < 3; ++i) {
            k[i] = axis[i] * ka;
            k[i + 3] = axis[i] * kb;
        }
    }
    float innov = resid;
    for (int i = 0; i < N; ++i) innov -= h[i] * dx[i];
    for (int i = 0; i < N; ++i) dx[i] += k[i] * innov;
    for (int i = 0; i < N; ++i) {
        for (int j = i; j < N; ++j) {
            float p = e->P[i][j] - k[i] * v[j] - v[i] * k[j] + s * k[i] * k[j];
            e->P[i][j] = e->P[j][i] = p;
        }
    }
}

static void inject(ins10_ekf *e, const float dx[N]) {
    float dq[4], q[4];
    ins10_quat_from_rotvec(dx, dq);
    ins10_quat_mul(e->q, dq, q);
    ins10_quat_normalize(q);
    memcpy(e->q, q, sizeof(q));
    ins10_ekf_shift_bias(e, dx + 3);
    condition(e);
}

void ins10_ekf_gravity_innovation(const ins10_ekf *e, const float accel[3],
                                  float resid[3], float *hph_trace) {
    float g[3];
    ins10_ekf_up_body(e, g);
    for (int i = 0; i < 3; ++i) resid[i] = accel[i] - g[i];
    /* H = [g]x, so trace(H P H^T) = trace(P_att) - g^T P_att g. */
    float tr = e->P[0][0] + e->P[1][1] + e->P[2][2];
    float gpg = 0.0f;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) gpg += g[i] * e->P[i][j] * g[j];
    }
    *hph_trace = tr - gpg > 0.0f ? tr - gpg : 0.0f;
}

void ins10_ekf_update_gravity(ins10_ekf *e, const float accel[3], float var) {
    /* accel = R^T z = g_b + g_b x dtheta, so the measurement matrix is [g_b]x. */
    float g[3], dx[N] = {0};
    ins10_ekf_up_body(e, g);
    const float rows[3][N] = {
        {0.0f, -g[2], g[1], 0.0f, 0.0f, 0.0f},
        {g[2], 0.0f, -g[0], 0.0f, 0.0f, 0.0f},
        {-g[1], g[0], 0.0f, 0.0f, 0.0f, 0.0f},
    };
    for (int i = 0; i < 3; ++i) scalar_update(e, rows[i], accel[i] - g[i], var, NULL, dx);
    inject(e, dx);
}

bool ins10_ekf_heading_innovation(const ins10_ekf *e, const float mag[3],
                                  const float north[2], float tan_dip,
                                  float *phi, float h[N], float *hph) {
    float r[9], m[3];
    ins10_quat_to_dcm(e->q, r);
    ins10_m3_mul_v(r, mag, m);
    float horiz2 = m[0] * m[0] + m[1] * m[1];
    if (!(horiz2 > 1e-12f)) return false;
    /* Angle, counter-clockwise about +z, from the reference direction to the
       measurement rotated into the navigation frame. With the body-frame error
       convention this angle equals -h_n . (R dtheta), where
       h_n = z + tan(dip) * north couples tilt error into azimuth through the dip. */
    *phi = atan2f(north[0] * m[1] - north[1] * m[0], north[0] * m[0] + north[1] * m[1]);
    float hn[3] = {tan_dip * north[0], tan_dip * north[1], 1.0f};
    for (int j = 0; j < 3; ++j) {
        h[j] = -(r[j] * hn[0] + r[3 + j] * hn[1] + r[6 + j] * hn[2]);
        h[j + 3] = 0.0f;
    }
    float s = 0.0f;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) s += h[i] * e->P[i][j] * h[j];
    }
    *hph = s > 0.0f ? s : 0.0f;
    return true;
}

void ins10_ekf_update_heading(ins10_ekf *e, const float h[N], float phi, float var,
                              bool yaw_only) {
    float up[3], dx[N] = {0};
    ins10_ekf_up_body(e, up);
    scalar_update(e, h, phi, var, yaw_only ? up : NULL, dx);
    inject(e, dx);
}

void ins10_ekf_update_zero_rate(ins10_ekf *e, const float rate[3], float var_horizontal,
                                float var_vertical) {
    /* The two noise levels belong to directions in the navigation frame, so the three
       scalar measurements are taken along an orthonormal body basis {b1, b2, up}
       instead of along the sensor axes; their noises are then uncorrelated. */
    float up[3], b1[3], b2[3], seed[3] = {0.0f, 0.0f, 0.0f}, dx[N] = {0};
    ins10_ekf_up_body(e, up);
    int least = fabsf(up[0]) < fabsf(up[1]) ? 0 : 1;
    if (fabsf(up[2]) < fabsf(up[least])) least = 2;
    seed[least] = 1.0f;
    ins10_cross(up, seed, b1);
    float norm = ins10_norm(b1);
    for (int i = 0; i < 3; ++i) b1[i] /= norm;
    ins10_cross(up, b1, b2);
    const float *basis[3] = {b1, b2, up};
    for (int k = 0; k < 3; ++k) {
        const float *b = basis[k];
        float h[N] = {0.0f, 0.0f, 0.0f, b[0], b[1], b[2]};
        float resid = 0.0f;
        for (int i = 0; i < 3; ++i) resid += b[i] * (rate[i] - e->bias[i]);
        scalar_update(e, h, resid, k < 2 ? var_horizontal : var_vertical, NULL, dx);
    }
    inject(e, dx);
}

void ins10_ekf_reset_yaw(ins10_ekf *e, float angle, float var_yaw) {
    float qz[4] = {cosf(0.5f * angle), 0.0f, 0.0f, sinf(0.5f * angle)}, q[4];
    ins10_quat_mul(qz, e->q, q);
    ins10_quat_normalize(q);
    memcpy(e->q, q, sizeof(q));
    /* Project yaw out of the covariance (rotating about the vertical leaves the
       body-frame up vector unchanged), then give it the requested variance. */
    float n[3];
    ins10_ekf_up_body(e, n);
    float pn[N] = {0};
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < 3; ++j) pn[i] += e->P[i][j] * n[j];
    }
    float npn = ins10_dot(pn, n);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            e->P[i][j] += -pn[i] * n[j] - n[i] * pn[j] + (npn + var_yaw) * n[i] * n[j];
        }
        for (int j = 3; j < N; ++j) {
            e->P[i][j] -= n[i] * pn[j];
            e->P[j][i] = e->P[i][j];
        }
    }
    condition(e);
}

void ins10_ekf_sigmas(const ins10_ekf *e, float *tilt, float *yaw, float *bias) {
    float n[3], npn = 0.0f;
    ins10_ekf_up_body(e, n);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) npn += n[i] * e->P[i][j] * n[j];
    }
    float tr = e->P[0][0] + e->P[1][1] + e->P[2][2];
    float tilt_var = 0.5f * (tr - npn);
    if (tilt) *tilt = sqrtf(tilt_var > 0.0f ? tilt_var : 0.0f);
    if (yaw) *yaw = sqrtf(npn > 0.0f ? npn : 0.0f);
    if (bias) *bias = sqrtf((e->P[3][3] + e->P[4][4] + e->P[5][5]) / 3.0f);
}
