/* Stillness detector.
 *
 * Works on fixed-length blocks of IMU samples. A block is "quiet" when the gyro and
 * accelerometer variances are at sensor-noise level, the mean rate is small and neither
 * mean moved since the previous block. The device is declared still after two
 * consecutive quiet blocks.
 *
 * A quiet gyro does not prove the absence of rotation: a slow, smooth turn (a pan head,
 * a turntable) reads exactly like a bias. What does prove it is that gravity and the
 * magnetic field stay put in the body frame. Both are therefore compared with their
 * value at the start of the still period, and the period ends when either drifts.
 * quiet_blocks counts the blocks since that start, so the caller knows for how long
 * the reference vectors have been steady and can bound the rotation rate accordingly.
 *
 * The block means are what the filter uses for the zero-rate update and what the
 * thermal model learns from.
 */
#ifndef INS10_MOTION_H
#define INS10_MOTION_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t block_len;
    float gyro_var_max;   /* (rad/s)^2, per axis */
    float accel_var_max;  /* g^2, per axis */
    float gyro_step_max;  /* rad/s, block-to-block change of the mean */
    float accel_step_max; /* g */
    float accel_drift_max; /* g, drift of the mean from the start of the still period */
    float rate_max;       /* rad/s, largest mean rate still accepted as a bias */
    float mag_step_max;   /* uT, drift from the start of the still period */
} ins10_motion_config;

typedef struct {
    ins10_motion_config cfg;
    /* Running block. Sums are of deviations from the first sample of the block, which
       keeps single precision adequate for variances near 1e-6. */
    uint16_t n, mag_n;
    float g0[3], a0[3], t0;
    float g_sum[3], g_sq[3], a_sum[3], a_sq[3], t_sum, m_sum[3];
    /* Last completed block. */
    float gyro_mean[3], accel_mean[3], mag_mean[3], temp_mean;
    float gyro_var, accel_var; /* largest per-axis variance */
    bool mag_valid, have_block;
    float accel_anchor[3], mag_anchor[3]; /* means at the start of the still period */
    bool mag_anchor_valid;
    uint16_t mag_anchor_block; /* quiet_blocks value at which the mag anchor was taken */
    uint16_t quiet_blocks;     /* blocks since the start of the still period */
    bool still;
} ins10_motion;

void ins10_motion_init(ins10_motion *m, const ins10_motion_config *cfg);
/* Returns true when this sample completed a block. */
bool ins10_motion_imu(ins10_motion *m, const float gyro[3], const float accel[3], float temp_c);
void ins10_motion_mag(ins10_motion *m, const float mag[3]);
/* Forget the still state, for example after a sensor restart. */
void ins10_motion_reset(ins10_motion *m);

#ifdef __cplusplus
}
#endif

#endif
