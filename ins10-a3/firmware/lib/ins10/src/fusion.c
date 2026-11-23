#include "ins10/fusion.h"
#include "ins10/math3.h"
#include <string.h>

#define D2R INS10_DEG2RAD
#define R2D INS10_RAD2DEG

static float sq(float x) { return x * x; }

void ins10_fusion_default_config(ins10_fusion_config *c) {
    memset(c, 0, sizeof(*c));
    c->sample_rate_hz = 512.0f;
    c->update_decim = 5;
    c->still_block_s = 0.5f;

    /* ICM-42688-P: 0.0028 dps/rtHz typical. The filter value is larger to cover
       vibration rectification and quantisation. */
    c->gyro_noise_dps_rthz = 0.005f;
    c->gyro_bias_rw_dps = 2e-4f;
    c->gyro_scale_err = 0.003f;
    c->gyro_corr_time_s = 1.0f;
    c->gyro_full_scale_dps = 2000.0f;

    c->accel_adaptive = true;
    c->accel_sigma_g = 0.01f;
    c->accel_norm_gain = 4.0f;
    c->accel_release_s = 0.5f;
    c->accel_ema_s = 0.1f;
    c->accel_corr_time_s = 0.4f;
    c->accel_diverge_s = 30.0f;

    c->mag_adaptive = true;
    c->mag_sigma_ut = 0.5f;
    c->mag_dist_gain = 4.0f;
    c->mag_release_s = 2.0f;
    c->mag_ema_s = 0.1f;
    c->mag_corr_time_s = 1.0f;
    c->mag_gate = 9.0f; /* 3 sigma */
    c->mag_ref_tau_s = 60.0f;
    c->mag_diverge_s = 20.0f;
    c->mag_min_horiz = 0.1f;
    c->mag_latency_sigma_s = 0.002f;
    c->mag_yaw_only = true;
    c->mag_require_cal = true;

    c->sustain_gate = 4.0f; /* 2 sigma */

    c->still_gyro_std_dps = 0.12f;
    c->still_accel_std_g = 0.004f;
    c->still_gyro_step_dps = 0.05f;
    c->still_accel_step_g = 0.0015f;
    c->still_accel_drift_g = 0.001f;
    c->still_max_rate_dps = 3.0f;
    c->still_mag_step_ut = 0.3f;
    c->zaru_sigma_dps = 0.02f;

    c->temp_tau_s = 2.0f;
    c->temp_rate_tau_s = 8.0f;
    c->thermal_transient_cps = 0.02f;
    c->tempco_prior_dps = 0.02f;
    c->thermal_corr_time_s = 20.0f;

    c->align_timeout_s = 3.0f;
}

static void start(ins10_fusion *f) {
    const ins10_fusion_config *c = &f->cfg;
    ins10_motion_config mc;
    float len = c->sample_rate_hz * c->still_block_s;
    mc.block_len = (uint16_t)ins10_clampf(len, 16.0f, 4096.0f);
    mc.gyro_var_max = sq(c->still_gyro_std_dps * D2R);
    mc.accel_var_max = sq(c->still_accel_std_g);
    mc.gyro_step_max = c->still_gyro_step_dps * D2R;
    mc.accel_step_max = c->still_accel_step_g;
    mc.accel_drift_max = c->still_accel_drift_g;
    mc.rate_max = c->still_max_rate_dps * D2R;
    mc.mag_step_max = c->still_mag_step_ut;
    ins10_motion_init(&f->motion, &mc);
    const float identity[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    ins10_ekf_init(&f->ekf, identity, NULL, 1.0f, 1.0f, sq(0.5f * D2R));
    if (f->cfg.update_decim == 0) f->cfg.update_decim = 1;
}

void ins10_fusion_init(ins10_fusion *f, const ins10_fusion_config *cfg,
                       const ins10_calibration *cal, const ins10_thermal_model *thermal) {
    memset(f, 0, sizeof(*f));
    f->cfg = *cfg;
    if (cal) f->cal = *cal;
    else ins10_cal_defaults(&f->cal);
    ins10_thermal_init(&f->therm, thermal, cfg->tempco_prior_dps * D2R,
                       cfg->temp_tau_s, cfg->temp_rate_tau_s);
    start(f);
}

static float hold(ins10_hold *h, float x, float dt, float window) {
    h->age += dt;
    if (h->age >= 0.5f * window) {
        h->previous = h->current;
        h->current = 0.0f;
        h->age = 0.0f;
    }
    if (x > h->current) h->current = x;
    return h->current > h->previous ? h->current : h->previous;
}

static void reset_mag_state(ins10_fusion *f) {
    f->heading_valid = f->mag_ref_valid = f->mag_disturbed = false;
    f->mag_level = f->mag_white = f->mag_phibar = 0.0f;
    f->mag_dist_time = f->mag_incons_s = f->mag_disturbance = 0.0f;
    memset(&f->mag_level_hold, 0, sizeof(f->mag_level_hold));
    memset(&f->mag_white_hold, 0, sizeof(f->mag_white_hold));
}

void ins10_fusion_set_calibration(ins10_fusion *f, const ins10_calibration *cal) {
    const ins10_calibration *old = &f->cal;
    bool gyro_tc = memcmp(&old->gyro_tc, &cal->gyro_tc, sizeof(cal->gyro_tc)) != 0;
    bool gyro = gyro_tc || memcmp(&old->gyro, &cal->gyro, sizeof(cal->gyro)) != 0;
    bool accel = memcmp(&old->accel, &cal->accel, sizeof(cal->accel)) != 0 ||
                 memcmp(&old->accel_tc, &cal->accel_tc, sizeof(cal->accel_tc)) != 0;
    bool mag = memcmp(&old->mag, &cal->mag, sizeof(cal->mag)) != 0 ||
               old->declination_deg != cal->declination_deg ||
               ((old->flags ^ cal->flags) & INS10_CAL_MAG) != 0;
    f->cal = *cal;
    if (gyro) ins10_thermal_rebase(&f->therm, gyro_tc, f->cfg.tempco_prior_dps * D2R);
    if (!f->aligned) return;
    /* The estimate was made with the old calibration. Say so in the covariance, or
       the correction would look like a disturbance and be resisted for a long time. */
    if (accel) ins10_ekf_add_attitude_var(&f->ekf, sq(2.0f * D2R), 0.0f);
    if (gyro) ins10_ekf_add_bias_var(&f->ekf, sq(0.5f * D2R));
    if (mag) reset_mag_state(f);
}

void ins10_fusion_restart(ins10_fusion *f) {
    ins10_fusion_config cfg = f->cfg;
    ins10_calibration cal = f->cal;
    ins10_thermal therm = f->therm;
    uint32_t recoveries = f->recoveries;
    memset(f, 0, sizeof(*f));
    f->cfg = cfg;
    f->cal = cal;
    f->therm = therm;
    f->recoveries = recoveries;
    start(f);
}

const ins10_thermal_model *ins10_fusion_thermal_model(const ins10_fusion *f) {
    return &f->therm.model;
}

/* ---- Initial alignment --------------------------------------------------------- */

/* Attitude with zero yaw whose body-frame up vector is `up`. */
static void tilt_to_quat(const float up[3], float q[4]) {
    float east[3], north[3];
    /* East is the horizontal direction closest to the body X axis; when X is nearly
       vertical, use Y for north instead. */
    if (fabsf(up[0]) < 0.9f) {
        float axis[3] = {1.0f, 0.0f, 0.0f};
        for (int i = 0; i < 3; ++i) east[i] = axis[i] - up[0] * up[i];
        float n = ins10_norm(east);
        for (int i = 0; i < 3; ++i) east[i] /= n;
        ins10_cross(up, east, north);
    } else {
        float axis[3] = {0.0f, 1.0f, 0.0f};
        for (int i = 0; i < 3; ++i) north[i] = axis[i] - up[1] * up[i];
        float n = ins10_norm(north);
        for (int i = 0; i < 3; ++i) north[i] /= n;
        ins10_cross(north, up, east);
    }
    /* Rows of the body -> nav rotation are the nav axes expressed in the body frame. */
    float r[9] = {east[0], east[1], east[2], north[0], north[1], north[2], up[0], up[1], up[2]};
    ins10_dcm_to_quat(r, q);
}

static void reset_window(ins10_fusion *f) {
    f->win_n = 0;
    f->win_dt = f->win_rate_sq = 0.0f;
    for (int i = 0; i < 3; ++i) f->win_accel[i] = f->win_rate[i] = 0.0f;
    f->win_saturated = false;
}

/* Called when a stillness block completes before alignment. */
static void try_align(ins10_fusion *f) {
    const ins10_fusion_config *c = &f->cfg;
    const ins10_motion *m = &f->motion;
    bool fine = m->still;
    if (!fine && f->time < (double)c->align_timeout_s) return;
    float up[3], norm = ins10_norm(m->accel_mean);
    if (norm < 0.3f || norm > 3.0f) return; /* free fall or a shock: wait */
    for (int i = 0; i < 3; ++i) up[i] = m->accel_mean[i] / norm;

    float q[4], bias[3] = {0.0f, 0.0f, 0.0f};
    float var_tilt, var_bias;
    tilt_to_quat(up, q);
    if (fine) {
        /* About a second of stillness bounds the rate only loosely; the zero-rate
           updates tighten it as the still period goes on. */
        memcpy(bias, m->gyro_mean, sizeof(bias));
        var_tilt = sq(1.0f * D2R);
        var_bias = sq(0.15f * D2R);
    } else {
        /* Moving: the mean acceleration is only roughly gravity and the gyro mean is
           not a bias. Start from the thermal model if it has ever been learned. */
        if (f->therm.model.count > 0) {
            ins10_thermal_predict(&f->therm, m->temp_mean, bias);
            var_bias = sq(0.15f * D2R);
        } else {
            var_bias = sq(0.6f * D2R);
        }
        var_tilt = sq(15.0f * D2R);
    }
    ins10_ekf_init(&f->ekf, q, bias, var_tilt, sq(1.0f * D2R), var_bias);
    f->aligned = true;
    f->coarse = !fine;
    f->temp_ff = f->therm.temp;
    f->thermal_obs_time = f->time;
    f->mag_time = f->time;
    reset_window(f);
    f->block_pending = false;
}

/* ---- Gravity update with adaptive weighting ------------------------------------ */

static void recover(ins10_fusion *f, float var_tilt, float var_yaw) {
    ins10_ekf_add_attitude_var(&f->ekf, var_tilt, var_yaw);
    ++f->recoveries;
    f->recovery_time = f->time;
}

static void accel_update(ins10_fusion *f, const float accel[3], float t) {
    const ins10_fusion_config *c = &f->cfg;
    float alpha = t / (c->accel_ema_s + t);
    float sigma2 = sq(c->accel_sigma_g);
    float norm = ins10_norm(accel);
    float dn = norm - 1.0f;

    /* Test 1, specific-force magnitude. An external acceleration shows in |a| only
       through its component along gravity; holding the peak for accel_release_s
       keeps the weight down between the peaks of an oscillation. */
    f->acc_level = hold(&f->acc_level_hold, c->accel_norm_gain * dn * dn, t, c->accel_release_s);
    f->acc_dn_bar += alpha * (dn - f->acc_dn_bar);
    if (norm < 0.3f || norm > 3.0f) {
        /* Free fall or a shock: there is no gravity direction to use. */
        f->acc_disturbed = true;
        f->acc_disturbance = fabsf(dn);
        return;
    }

    /* Test 2, the innovation itself, which also sees horizontal accelerations. It is
       split in the navigation frame into a sustained part (its running mean) and a
       fluctuating part. Vibration is white: it averages out and costs only its own
       variance. A sustained acceleration does not average out; it stays correlated
       for accel_corr_time_s, so every sample of it carries 1/n_corr of the information
       a white sample would, and its power is weighted by n_corr. Only what the
       filter's own uncertainty (hph) cannot explain counts as acceleration, so a real
       attitude error is still corrected. */
    float resid[3], rn[3], hph;
    ins10_ekf_gravity_innovation(&f->ekf, accel, resid, &hph);
    ins10_quat_rotate(f->ekf.q, resid, rn);
    float dev2 = 0.0f, rbar2 = 0.0f;
    for (int i = 0; i < 3; ++i) {
        f->acc_rbar[i] += alpha * (rn[i] - f->acc_rbar[i]);
        dev2 += sq(rn[i] - f->acc_rbar[i]);
        rbar2 += sq(f->acc_rbar[i]);
    }
    f->acc_white = hold(&f->acc_white_hold, dev2 * (1.0f / 3.0f), t, c->accel_release_s);
    /* The mean of white noise is not exactly zero; remove its expected power. */
    float corr = (rbar2 - c->sustain_gate * hph) * (1.0f / 3.0f) -
                 f->acc_white * alpha / (2.0f - alpha);
    if (corr < 0.0f) corr = 0.0f;
    float n_corr = 2.0f * c->accel_corr_time_s / t;
    if (n_corr < 1.0f) n_corr = 1.0f;

    /* Note that being at rest (no rotation, steady accelerometer) earns no extra
       trust here: a vehicle accelerating smoothly in a straight line looks exactly
       like that, and only the innovation tells the two apart. */
    float var = sigma2;
    if (c->accel_adaptive) var += f->acc_level + f->acc_white + n_corr * corr;
    ins10_ekf_update_gravity(&f->ekf, accel, var);
    f->acc_disturbed = var > 10.0f * sigma2;
    f->acc_disturbance = sqrtf(3.0f * (f->acc_white + corr));
    if (!c->accel_adaptive) return;

    /* Divergence watchdog. A sustained innovation with |a| = 1 g that never goes away
       is an attitude error, not an acceleration: a real horizontal acceleration a also
       raises |a| by about a^2/2. Re-open the tilt covariance; the innovation is then
       explained by it and gravity pulls the estimate back. */
    if (3.0f * corr > sq(0.05f) && fabsf(f->acc_dn_bar) < 0.01f + 0.25f * rbar2) {
        f->acc_incons_s += t;
        if (f->acc_incons_s > c->accel_diverge_s) {
            recover(f, rbar2 > sq(0.1f) ? rbar2 : sq(0.1f), 0.0f);
            f->acc_incons_s = 0.0f;
        }
    } else {
        f->acc_incons_s -= 2.0f * t;
        if (f->acc_incons_s < 0.0f) f->acc_incons_s = 0.0f;
    }
}

/* ---- Zero-rate update and thermal learning, once per still block ---------------- */

/* A reference vector that moved by less than `angle` over the first k blocks of a
   still period bounds the mean rate by angle / (k block_s). Successive blocks refine
   one and the same bound, so block k contributes only the information it adds:
   variance angle^2 / (T_k^2 - T_{k-1}^2). */
static float rate_bound_var(float angle, int k, float block_s) {
    return sq(angle / block_s) / (float)(2 * k - 1);
}

static void block_update(ins10_fusion *f) {
    const ins10_fusion_config *c = &f->cfg;
    const ins10_motion *m = &f->motion;
    if (!m->still) return;
    /* A quiet gyro is consistent with rest and equally with a slow, smooth rotation;
       assuming rest would teach the filter that rotation as a bias. What bounds the
       rate is how long gravity (rotation about horizontal axes) and the magnetic
       field (rotation about the vertical) have stayed put in the body frame. */
    float block_s = (float)m->cfg.block_len / c->sample_rate_hz;
    float floor_var = sq(c->zaru_sigma_dps * D2R);
    int blocks = (int)m->quiet_blocks - 1;
    float var_h = rate_bound_var(c->still_accel_drift_g, blocks, block_s);
    if (var_h < floor_var) var_h = floor_var;
    /* Without a magnetometer in use nothing can bound rotation about the vertical;
       rest is then assumed, which is the documented limitation. */
    float var_v = floor_var;
    if (f->heading_valid && !f->mag_blocked && m->mag_anchor_valid && f->mag_ref_valid) {
        int mag_blocks = blocks - (int)m->mag_anchor_block;
        float mh = f->mag_ref_h > 5.0f ? f->mag_ref_h : 5.0f;
        var_v = mag_blocks > 0 ? rate_bound_var(c->still_mag_step_ut / mh, mag_blocks, block_s)
                               : sq(1.0f);
        if (var_v < floor_var) var_v = floor_var;
    }
    ins10_ekf_update_zero_rate(&f->ekf, m->gyro_mean, var_h, var_v);
    /* The thermal model learns from the block mean itself. It keeps one variance for
       all three axes, so it takes the horizontal one, and only from still periods
       that have already lasted a while. */
    if (blocks < 3) return;
    float elapsed = (float)(f->time - f->thermal_obs_time);
    f->thermal_obs_time = f->time;
    ins10_thermal_observe(&f->therm, m->gyro_mean, m->temp_mean, var_h,
                          sq(c->gyro_bias_rw_dps * D2R) * ins10_clampf(elapsed, 0.0f, 3600.0f));
}

/* ---- Covariance step ------------------------------------------------------------ */

static void window_update(ins10_fusion *f) {
    const ins10_fusion_config *c = &f->cfg;
    float t = f->win_dt, inv = 1.0f / (float)f->win_n;
    float accel[3], rate[3];
    for (int i = 0; i < 3; ++i) {
        accel[i] = f->win_accel[i] * inv;
        rate[i] = f->win_rate[i] * inv;
    }
    /* Attitude process noise: angle random walk, plus a term proportional to the rate
       for scale-factor and misalignment errors, plus a large term on saturation. */
    float q_att = sq(c->gyro_noise_dps_rthz * D2R) * t +
                  sq(c->gyro_scale_err) * (f->win_rate_sq * inv) * c->gyro_corr_time_s * t;
    if (f->win_saturated) {
        q_att += sq(0.5f * c->gyro_full_scale_dps * D2R * t);
        f->sat_time = f->time;
    }
    float q_bias = sq(c->gyro_bias_rw_dps * D2R) * t;
    if (f->therm.temp_valid) {
        /* Thermal feed-forward: move the bias along the learned slope and loosen it
           in proportion to how uncertain that slope still is. */
        float slope[3], sigma, delta[3];
        ins10_thermal_slope(&f->therm, f->therm.temp, slope, &sigma);
        float dtemp = f->therm.temp - f->temp_ff;
        f->temp_ff = f->therm.temp;
        for (int i = 0; i < 3; ++i) delta[i] = slope[i] * dtemp;
        ins10_ekf_shift_bias(&f->ekf, delta);
        q_bias += sq(sigma * f->therm.temp_rate) * c->thermal_corr_time_s * t;
    }
    f->rate_rms2 += t / (1.0f + t) * (f->win_rate_sq * inv - f->rate_rms2);
    ins10_ekf_propagate(&f->ekf, q_att, q_bias);
    /* The window mean describes the middle of the window, while the attitude is at
       its end. Seen from the body a fixed vector turns with -w, so carry the mean
       forward by its age; at 150 dps the uncorrected lag is a quarter of a degree. */
    float age = 0.5f * (float)(f->win_n - 1) * t * inv, w[3], turn[3], moved[3];
    for (int i = 0; i < 3; ++i) w[i] = rate[i] - f->ekf.bias[i];
    ins10_cross(w, accel, turn);
    for (int i = 0; i < 3; ++i) moved[i] = accel[i] - turn[i] * age;
    accel_update(f, moved, t);
    if (f->block_pending) {
        f->block_pending = false;
        block_update(f);
    }
    reset_window(f);
}

/* Last line of defence: if anything non-finite ever reaches the state, start again
   from alignment instead of reporting garbage for ever. */
static void check_state(ins10_fusion *f) {
    float sum = f->ekf.q[0] + f->ekf.q[1] + f->ekf.q[2] + f->ekf.q[3];
    for (int i = 0; i < 3; ++i) sum += f->ekf.bias[i];
    for (int i = 0; i < INS10_EKF_N; ++i) {
        for (int j = 0; j < INS10_EKF_N; ++j) sum += f->ekf.P[i][j];
    }
    if (isfinite(sum)) return;
    uint32_t faults = f->faults + 1, rejected = f->rejected;
    ins10_fusion_restart(f);
    f->faults = faults;
    f->rejected = rejected;
}

void ins10_fusion_imu(ins10_fusion *f, const float gyro_dps[3], const float accel_g[3],
                      float temp_c, float dt, bool saturated) {
    for (int i = 0; i < 3; ++i) {
        /* Not finite, or far outside what the sensor can output: refuse, so that one
           corrupted sample cannot poison the state. */
        if (!(fabsf(gyro_dps[i]) < 1.0e4f) || !(fabsf(accel_g[i]) < 100.0f)) {
            ++f->rejected;
            return;
        }
    }
    if (!(fabsf(temp_c) < 200.0f)) temp_c = f->therm.temp_valid ? f->therm.temp : INS10_THERMAL_T0;
    if (!(dt > 0.0f) || dt > 0.1f) dt = 1.0f / f->cfg.sample_rate_hz;

    float w[3], a[3];
    ins10_cal_apply_gyro(&f->cal, gyro_dps, temp_c, w);
    ins10_cal_apply_accel(&f->cal, accel_g, temp_c, a);
    memcpy(f->last_rate, w, sizeof(w));
    memcpy(f->last_accel, a, sizeof(a));
    f->time += (double)dt;
    ins10_thermal_temp(&f->therm, temp_c, dt);
    bool block = ins10_motion_imu(&f->motion, w, a, temp_c);

    if (!f->aligned) {
        if (block) try_align(f);
        return;
    }
    ins10_ekf_integrate(&f->ekf, w, dt);
    float rate_sq = 0.0f;
    for (int i = 0; i < 3; ++i) {
        f->win_accel[i] += a[i];
        f->win_rate[i] += w[i];
        rate_sq += sq(w[i] - f->ekf.bias[i]);
    }
    f->win_rate_sq += rate_sq;
    f->win_dt += dt;
    f->win_saturated |= saturated;
    f->block_pending |= block;
    if (++f->win_n >= f->cfg.update_decim) {
        window_update(f);
        check_state(f);
    }
}

void ins10_fusion_gap(ins10_fusion *f, float seconds) {
    if (!f->aligned || !(seconds > 0.0f)) return;
    /* The rate during the gap is unknown. Take the largest of the last sample, the
       recent RMS (a single sample of a varying motion can be near zero) and 30 dps: a
       gap usually has a mechanical cause. */
    float rate2 = ins10_dot(f->last_rate, f->last_rate);
    if (rate2 < f->rate_rms2) rate2 = f->rate_rms2;
    if (rate2 < sq(0.5f)) rate2 = sq(0.5f);
    float var = rate2 * sq(ins10_clampf(seconds, 0.0f, 10.0f));
    ins10_ekf_add_attitude_var(&f->ekf, var, var);
    f->ekf.prev_count = 0; /* the integrator history is stale */
    f->time += (double)seconds;
    f->sat_time = f->time;
}

/* ---- Heading update with magnetic disturbance rejection ------------------------- */

void ins10_fusion_mag(ins10_fusion *f, const float mag_ut[3], float age_s) {
    const ins10_fusion_config *c = &f->cfg;
    float m[3];
    for (int i = 0; i < 3; ++i) {
        if (!(fabsf(mag_ut[i]) < 1.0e4f)) {
            ++f->rejected;
            return;
        }
    }
    ins10_cal_apply_mag(&f->cal, mag_ut, m);
    memcpy(f->last_mag, m, sizeof(m));
    ins10_motion_mag(&f->motion, m);
    f->mag_blocked = c->mag_require_cal && !(f->cal.flags & INS10_CAL_MAG);
    if (!f->aligned || f->mag_blocked) return;

    /* Bring the sample to the present: seen from the body, a fixed field rotates
       with -w. */
    float w[3], turn[3];
    for (int i = 0; i < 3; ++i) w[i] = f->last_rate[i] - f->ekf.bias[i];
    age_s = ins10_clampf(age_s, 0.0f, 0.1f);
    ins10_cross(w, m, turn);
    for (int i = 0; i < 3; ++i) m[i] -= turn[i] * age_s;

    float up[3];
    ins10_ekf_up_body(&f->ekf, up);
    float m2 = ins10_dot(m, m);
    if (m2 < 1.0f) return; /* below 1 uT: not a field measurement */
    float mv = ins10_dot(up, m);
    float mh = sqrtf(m2 - mv * mv > 0.0f ? m2 - mv * mv : 0.0f);
    float min_h = c->mag_min_horiz * sqrtf(m2);

    float dt = ins10_clampf((float)(f->time - f->mag_time), 0.0f, 1.0f);
    f->mag_time = f->time;

    /* The invariants are only as good as the tilt they are computed with. Until the
       tilt is known to about a degree the reference simply follows the measurement
       (no test), and afterwards the tilt uncertainty is discounted from the deviation. */
    float tilt_sigma;
    ins10_ekf_sigmas(&f->ekf, &tilt_sigma, NULL, NULL);
    if (!f->mag_ref_valid) {
        if (mh < min_h) return;
        f->mag_ref_h = mh;
        f->mag_ref_v = mv;
        f->mag_ref_valid = tilt_sigma < 1.5f * D2R;
    }
    /* Disturbance monitor. With the tilt known from gravity, the field has two
       observable invariants: its horizontal magnitude and its vertical component. A
       disturbance vector shows two of its three components there; the third, the
       east component, is the one that turns the compass and cannot be seen directly.
       Assume it is as large as the visible ones. */
    float dh = mh - f->mag_ref_h, dv = mv - f->mag_ref_v;
    float e2 = dh * dh + dv * dv - c->sustain_gate * m2 * sq(tilt_sigma);
    if (e2 < 0.0f) e2 = 0.0f;
    float ref2 = sq(f->mag_ref_h) + sq(f->mag_ref_v);
    float sigma2 = sq(c->mag_sigma_ut);
    f->mag_level = hold(&f->mag_level_hold, c->mag_dist_gain * 0.5f * e2, dt, c->mag_release_s);
    f->mag_disturbance = sqrtf(e2);
    bool field_clean = f->mag_level < 9.0f * sigma2;

    /* Reference field. While the invariants hold, follow slow genuine changes. While
       they fail, hold the reference; a field that stays different for longer than
       mag_ref_tau_s is a new environment, not a passing disturbance, and is adopted. */
    if (e2 < 9.0f * sigma2) {
        float a_ref = dt / (c->mag_ref_tau_s + dt);
        f->mag_ref_h += a_ref * dh;
        f->mag_ref_v += a_ref * dv;
        f->mag_dist_time -= dt;
        if (f->mag_dist_time < 0.0f) f->mag_dist_time = 0.0f;
    } else {
        f->mag_dist_time += dt;
        if (f->mag_dist_time > c->mag_ref_tau_s) {
            float a_ref = dt / (0.25f * c->mag_ref_tau_s + dt);
            f->mag_ref_h += a_ref * dh;
            f->mag_ref_v += a_ref * dv;
        }
    }
    if (mh < min_h || f->mag_ref_h < c->mag_min_horiz * sqrtf(ref2)) {
        f->mag_disturbed = true; /* field too close to vertical to carry heading */
        return;
    }

    float decl = f->cal.declination_deg * D2R;
    float north[2] = {sinf(decl), cosf(decl)};
    float tan_dip = ins10_clampf(-f->mag_ref_v / f->mag_ref_h, -20.0f, 20.0f);
    float phi, h[INS10_EKF_N], hph;
    if (!ins10_ekf_heading_innovation(&f->ekf, m, north, tan_dip, &phi, h, &hph)) return;

    if (!f->heading_valid) {
        ins10_ekf_reset_yaw(&f->ekf, -phi, sq(3.0f * D2R) + f->mag_level / (mh * mh));
        f->heading_valid = true;
        f->mag_phibar = 0.0f;
        f->mag_white = 0.0f;
        return;
    }
    /* Same split as for the accelerometer: the sustained part of the heading
       innovation that the yaw uncertainty cannot explain is a disturbance and is
       weighted by its correlation length; so is the level from the invariants. */
    float alpha = dt / (c->mag_ema_s + dt);
    f->mag_phibar += alpha * (phi - f->mag_phibar);
    f->mag_white = hold(&f->mag_white_hold, sq(phi - f->mag_phibar), dt, c->mag_release_s);
    float corr = sq(f->mag_phibar) - c->sustain_gate * hph -
                 f->mag_white * alpha / (2.0f - alpha);
    if (corr < 0.0f) corr = 0.0f;
    float n_corr = 2.0f * c->mag_corr_time_s / (dt > 1e-3f ? dt : 1e-3f);
    if (n_corr < 1.0f) n_corr = 1.0f;
    float r0 = sigma2 / (mh * mh) + sq(ins10_norm(w) * c->mag_latency_sigma_s);
    float var = r0;
    if (c->mag_adaptive) var += f->mag_white + n_corr * (corr + f->mag_level / (mh * mh));
    ins10_ekf_update_heading(&f->ekf, h, phi, var, c->mag_yaw_only);
    check_state(f);
    f->mag_disturbed = var > 10.0f * r0;
    if (!c->mag_adaptive) return;

    /* Divergence watchdog: heading persistently disagrees with a field that passes
       both invariants. Re-open the yaw covariance and follow the compass. */
    if (corr > c->mag_gate * r0 && field_clean) {
        f->mag_incons_s += dt;
        if (f->mag_incons_s > c->mag_diverge_s) {
            recover(f, 0.0f, sq(f->mag_phibar) > sq(0.1f) ? sq(f->mag_phibar) : sq(0.1f));
            f->mag_incons_s = 0.0f;
        }
    } else {
        f->mag_incons_s -= 2.0f * dt;
        if (f->mag_incons_s < 0.0f) f->mag_incons_s = 0.0f;
    }
}

/* ---- Output --------------------------------------------------------------------- */

void ins10_fusion_get(const ins10_fusion *f, ins10_output *o) {
    const ins10_fusion_config *c = &f->cfg;
    float r[9], roll, pitch, yaw, tilt, hdg, bias;
    memset(o, 0, sizeof(*o));
    memcpy(o->q, f->ekf.q, sizeof(o->q));
    ins10_quat_to_dcm(f->ekf.q, r);
    ins10_quat_to_euler(f->ekf.q, &roll, &pitch, &yaw);
    o->roll_deg = roll * R2D;
    o->pitch_deg = pitch * R2D;
    o->yaw_deg = yaw * R2D;
    /* Body X in ENU is the first column of R; compass heading runs clockwise from north. */
    float heading = atan2f(r[0], r[3]) * R2D;
    o->heading_deg = heading < 0.0f ? heading + 360.0f : heading;
    ins10_m3_mul_v(r, f->last_accel, o->lin_accel_g);
    o->lin_accel_g[2] -= 1.0f;
    for (int i = 0; i < 3; ++i) {
        o->rate_dps[i] = (f->last_rate[i] - f->ekf.bias[i]) * R2D;
        o->accel_g[i] = f->last_accel[i];
        o->gyro_bias_dps[i] = f->ekf.bias[i] * R2D;
        o->mag_ut[i] = f->last_mag[i];
    }
    ins10_ekf_sigmas(&f->ekf, &tilt, &hdg, &bias);
    o->tilt_sigma_deg = tilt * R2D;
    o->heading_sigma_deg = hdg * R2D;
    o->bias_sigma_dps = bias * R2D;
    o->temp_c = f->therm.temp;
    o->temp_rate_cps = f->therm.temp_rate;
    o->accel_disturbance_g = f->acc_disturbance;
    o->mag_disturbance_ut = f->mag_disturbance;
    if (f->mag_ref_valid) {
        o->mag_field_ut = sqrtf(sq(f->mag_ref_h) + sq(f->mag_ref_v));
        o->mag_dip_deg = atan2f(-f->mag_ref_v, f->mag_ref_h) * R2D;
    }

    uint32_t flags = 0;
    if (f->aligned) {
        flags |= INS10_F_ALIGNED;
        if (f->coarse && o->tilt_sigma_deg > 1.5f) flags |= INS10_F_COARSE;
        if (f->motion.still) flags |= INS10_F_STILL;
        if (f->heading_valid) flags |= INS10_F_HEADING;
        if (f->acc_disturbed) flags |= INS10_F_ACC_DISTURBED;
        if (f->heading_valid && f->mag_disturbed) flags |= INS10_F_MAG_DISTURBED;
        if (f->heading_valid && f->time - f->mag_time > 1.0) flags |= INS10_F_MAG_STALE;
        if (f->sat_time > 0.0 && f->time - f->sat_time < 1.0) flags |= INS10_F_GYRO_SAT;
        if (f->recoveries > 0 && f->time - f->recovery_time < 5.0) flags |= INS10_F_RECOVERY;
    }
    if (f->mag_blocked) flags |= INS10_F_MAG_UNCAL;
    if (fabsf(f->therm.temp_rate) > c->thermal_transient_cps) flags |= INS10_F_THERMAL;
    o->flags = flags;
}
