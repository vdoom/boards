#include "ins10/motion.h"
#include <math.h>
#include <string.h>

void ins10_motion_init(ins10_motion *m, const ins10_motion_config *cfg) {
    memset(m, 0, sizeof(*m));
    m->cfg = *cfg;
    if (m->cfg.block_len < 8) m->cfg.block_len = 8;
}

void ins10_motion_reset(ins10_motion *m) {
    ins10_motion_config cfg = m->cfg;
    ins10_motion_init(m, &cfg);
}

static float max_step(const float a[3], const float b[3]) {
    float worst = 0.0f;
    for (int i = 0; i < 3; ++i) {
        float d = fabsf(a[i] - b[i]);
        if (d > worst) worst = d;
    }
    return worst;
}

static void finish_block(ins10_motion *m) {
    float inv = 1.0f / (float)m->n;
    float gyro_mean[3], accel_mean[3];
    float gyro_var = 0.0f, accel_var = 0.0f, rate = 0.0f;
    for (int i = 0; i < 3; ++i) {
        float gm = m->g_sum[i] * inv, am = m->a_sum[i] * inv;
        float gv = m->g_sq[i] * inv - gm * gm, av = m->a_sq[i] * inv - am * am;
        if (gv > gyro_var) gyro_var = gv;
        if (av > accel_var) accel_var = av;
        gyro_mean[i] = m->g0[i] + gm;
        accel_mean[i] = m->a0[i] + am;
        if (fabsf(gyro_mean[i]) > rate) rate = fabsf(gyro_mean[i]);
    }
    bool quiet = gyro_var < m->cfg.gyro_var_max && accel_var < m->cfg.accel_var_max &&
                 rate < m->cfg.rate_max;
    if (quiet && m->have_block) {
        quiet = max_step(gyro_mean, m->gyro_mean) < m->cfg.gyro_step_max &&
                max_step(accel_mean, m->accel_mean) < m->cfg.accel_step_max;
    }
    memcpy(m->gyro_mean, gyro_mean, sizeof(gyro_mean));
    memcpy(m->accel_mean, accel_mean, sizeof(accel_mean));
    m->temp_mean = m->t0 + m->t_sum * inv;
    m->gyro_var = gyro_var;
    m->accel_var = accel_var;
    m->have_block = true;

    m->mag_valid = m->mag_n > 0;
    if (m->mag_valid) {
        for (int i = 0; i < 3; ++i) m->mag_mean[i] = m->m_sum[i] / (float)m->mag_n;
    }
    if (!quiet) {
        m->quiet_blocks = 0;
        m->mag_anchor_valid = false;
    } else {
        /* A rotation too slow and smooth for the tests above still moves gravity and
           the magnetic field in the body frame. Compare both with what they were when
           this still period began; if either has drifted, the period was not really
           still, and a new one starts from this block. */
        bool drifted = false;
        if (m->quiet_blocks > 0 &&
            max_step(accel_mean, m->accel_anchor) > m->cfg.accel_drift_max) drifted = true;
        if (m->mag_valid && m->mag_anchor_valid &&
            max_step(m->mag_mean, m->mag_anchor) > m->cfg.mag_step_max) drifted = true;
        if (drifted) m->quiet_blocks = 0;
        if (m->quiet_blocks == 0) {
            memcpy(m->accel_anchor, accel_mean, sizeof(m->accel_anchor));
            m->mag_anchor_valid = false;
        }
        if (m->mag_valid && !m->mag_anchor_valid) {
            /* Normally the first block of the period; later if the magnetometer
               started delivering in the middle of it. */
            memcpy(m->mag_anchor, m->mag_mean, sizeof(m->mag_anchor));
            m->mag_anchor_valid = true;
            m->mag_anchor_block = m->quiet_blocks;
        }
        if (m->quiet_blocks < UINT16_MAX) ++m->quiet_blocks;
    }
    m->still = m->quiet_blocks >= 2;
    m->n = 0;
    m->mag_n = 0;
}

bool ins10_motion_imu(ins10_motion *m, const float gyro[3], const float accel[3], float temp_c) {
    if (m->n == 0) {
        for (int i = 0; i < 3; ++i) {
            m->g0[i] = gyro[i];
            m->a0[i] = accel[i];
            m->g_sum[i] = m->g_sq[i] = m->a_sum[i] = m->a_sq[i] = m->m_sum[i] = 0.0f;
        }
        m->t0 = temp_c;
        m->t_sum = 0.0f;
    }
    for (int i = 0; i < 3; ++i) {
        float dg = gyro[i] - m->g0[i], da = accel[i] - m->a0[i];
        m->g_sum[i] += dg;
        m->g_sq[i] += dg * dg;
        m->a_sum[i] += da;
        m->a_sq[i] += da * da;
    }
    m->t_sum += temp_c - m->t0;
    if (++m->n < m->cfg.block_len) return false;
    finish_block(m);
    return true;
}

void ins10_motion_mag(ins10_motion *m, const float mag[3]) {
    /* The sums are cleared by the first IMU sample of a block, so a magnetometer
       sample arriving before it is dropped. At most one per block is lost. */
    if (m->n == 0) return;
    for (int i = 0; i < 3; ++i) m->m_sum[i] += mag[i];
    ++m->mag_n;
}
