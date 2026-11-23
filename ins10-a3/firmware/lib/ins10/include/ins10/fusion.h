/* INS10 attitude and heading reference: the complete sensor-fusion pipeline.
 *
 *   raw gyro/accel/temp --calibration--> stillness detector, thermal tracker
 *                                    \-> error-state EKF (ekf.h) <-- adaptive noise
 *   raw mag --------------calibration--> heading-only update   <-- disturbance monitor
 *
 * Frames: navigation is East-North-Up; the body frame is the IMU chip frame (a board
 * lying flat, components up, reads +1 g on Z and has zero roll and pitch).
 *
 * Usage:
 *   ins10_fusion_default_config(&cfg);
 *   ins10_fusion_init(&f, &cfg, &calibration, stored_thermal_model_or_NULL);
 *   for every IMU sample:  ins10_fusion_imu(&f, gyro_dps, accel_g, temp_c, dt, saturated);
 *   for every mag sample:  ins10_fusion_mag(&f, mag_ut, age_s);
 *   ins10_fusion_get(&f, &out);
 *
 * The library is portable C11 with no dynamic memory and no platform dependencies.
 */
#ifndef INS10_FUSION_H
#define INS10_FUSION_H

#include "ins10/calib.h"
#include "ins10/ekf.h"
#include "ins10/motion.h"
#include "ins10/thermal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Status flags in ins10_output.flags. */
#define INS10_F_ALIGNED       (1u << 0)  /* attitude initialised */
#define INS10_F_COARSE        (1u << 1)  /* aligned while moving; converging */
#define INS10_F_STILL         (1u << 2)  /* at rest; zero-rate updates active */
#define INS10_F_HEADING       (1u << 3)  /* heading referenced to the magnetic field */
#define INS10_F_ACC_DISTURBED (1u << 4)  /* linear acceleration detected; accel de-weighted */
#define INS10_F_MAG_DISTURBED (1u << 5)  /* magnetic disturbance detected; mag de-weighted */
#define INS10_F_MAG_UNCAL     (1u << 6)  /* mag ignored: no calibration stored */
#define INS10_F_GYRO_SAT      (1u << 7)  /* gyro or accel saturated recently */
#define INS10_F_THERMAL       (1u << 8)  /* temperature transient in progress */
#define INS10_F_RECOVERY      (1u << 9)  /* divergence watchdog re-opened the covariance recently */
#define INS10_F_MAG_STALE     (1u << 10) /* no usable mag sample for over a second */

typedef struct {
    float sample_rate_hz;      /* nominal IMU rate; sizes the stillness block */
    uint16_t update_decim;     /* IMU samples per covariance step / accel update */
    float still_block_s;       /* stillness block length */

    /* Gyro model. */
    float gyro_noise_dps_rthz; /* angle random walk density */
    float gyro_bias_rw_dps;    /* bias random walk, dps / sqrt(s) */
    float gyro_scale_err;      /* residual scale-factor and misalignment error, fraction */
    float gyro_corr_time_s;    /* correlation time used for the rate-proportional noise */
    float gyro_full_scale_dps;

    /* Accelerometer (gravity) update. */
    bool accel_adaptive;       /* false: fixed noise, a conventional EKF (for comparison) */
    float accel_sigma_g;       /* trusted level per update when nothing is detected */
    float accel_norm_gain;     /* weight of the |a| - 1 g test */
    float accel_release_s;     /* how long a detected acceleration keeps the weight low */
    float accel_ema_s;         /* averaging time that separates sustained from fluctuating */
    float accel_corr_time_s;   /* assumed correlation time of a sustained acceleration */
    float accel_diverge_s;     /* persistent inconsistency before the watchdog acts */

    /* Magnetometer (heading) update. */
    bool mag_adaptive;
    float mag_sigma_ut;
    float mag_dist_gain;       /* weight of the field-magnitude and dip tests */
    float mag_release_s;
    float mag_ema_s;
    float mag_corr_time_s;     /* assumed correlation time of a magnetic disturbance */
    float mag_gate;            /* sustained heading innovation, in sigmas squared, counted as inconsistent */
    float mag_ref_tau_s;       /* adaptation time of the reference field */
    float mag_diverge_s;
    float mag_min_horiz;       /* minimum horizontal fraction of the field carrying heading */
    float mag_latency_sigma_s; /* timing uncertainty of a mag sample */
    bool mag_yaw_only;         /* mag never corrects roll and pitch */
    bool mag_require_cal;      /* ignore the magnetometer until it is calibrated */

    /* A sustained innovation counts as a disturbance only beyond this many variances
       of what the filter's own uncertainty explains. */
    float sustain_gate;

    /* Stillness detection and the zero-rate update. */
    float still_gyro_std_dps;
    float still_accel_std_g;
    float still_gyro_step_dps;
    float still_accel_step_g;
    float still_accel_drift_g; /* drift of gravity in the body frame that ends a still period */
    float still_max_rate_dps;
    float still_mag_step_ut;
    float zaru_sigma_dps;

    /* Temperature. */
    float temp_tau_s;
    float temp_rate_tau_s;
    float thermal_transient_cps; /* deg C / s that raises INS10_F_THERMAL */
    float tempco_prior_dps;      /* 1-sigma of the unlearned bias tempco, dps per deg C */
    float thermal_corr_time_s;

    /* Initial alignment. */
    float align_timeout_s;       /* give up waiting for stillness and align coarsely */
} ins10_fusion_config;

/* Peak hold over a sliding window: reports the largest input of the last half to
   full window, however large the peak was. */
typedef struct {
    float current, previous, age;
} ins10_hold;

typedef struct {
    float q[4];              /* body -> ENU, {w, x, y, z} */
    float roll_deg, pitch_deg, yaw_deg; /* ZYX Euler angles of q */
    float heading_deg;       /* compass heading of the body X axis, 0..360, clockwise from north */
    float rate_dps[3];       /* bias-corrected body rate */
    float accel_g[3];        /* calibrated specific force, body frame */
    float lin_accel_g[3];    /* acceleration with gravity removed, navigation frame */
    float gyro_bias_dps[3];
    float mag_ut[3];         /* calibrated field, body frame */
    float tilt_sigma_deg, heading_sigma_deg, bias_sigma_dps; /* filter 1-sigma */
    float temp_c, temp_rate_cps;
    float accel_disturbance_g;  /* estimated external acceleration level */
    float mag_disturbance_ut;   /* deviation of the field from its reference */
    float mag_field_ut, mag_dip_deg; /* current reference field */
    uint32_t flags;
} ins10_output;

typedef struct {
    ins10_fusion_config cfg;
    ins10_calibration cal;
    ins10_ekf ekf;
    ins10_motion motion;
    ins10_thermal therm;

    bool aligned, coarse;
    double time;             /* seconds of IMU data processed */
    float last_rate[3];      /* calibrated body rate, bias not removed */
    float last_accel[3];
    float last_mag[3];

    /* Update window between covariance steps. */
    uint16_t win_n;
    float win_dt, win_accel[3], win_rate[3], win_rate_sq;
    bool win_saturated, block_pending;

    float rate_rms2;         /* recent mean square body rate, (rad/s)^2 */

    /* Adaptive accelerometer state. */
    ins10_hold acc_level_hold, acc_white_hold, mag_level_hold, mag_white_hold;
    float acc_level;         /* variance from the magnitude test, g^2 */
    float acc_rbar[3];       /* sustained part of the innovation, navigation frame, g */
    float acc_dn_bar;        /* sustained part of |a| - 1 */
    float acc_white;         /* variance of the fluctuating part, g^2 per axis */
    float acc_incons_s;
    float acc_disturbance;   /* reported level, g */
    bool acc_disturbed;

    /* Adaptive magnetometer state. */
    bool heading_valid, mag_ref_valid, mag_disturbed, mag_blocked;
    float mag_ref_h, mag_ref_v; /* horizontal magnitude and upward component, uT */
    float mag_level;         /* variance from the field tests, uT^2 */
    float mag_phibar;        /* sustained part of the heading innovation, rad */
    float mag_white;         /* variance of its fluctuating part, rad^2 */
    float mag_dist_time;     /* how long the field has failed its tests, s */
    float mag_incons_s;
    float mag_disturbance;
    double mag_time;

    /* Thermal feed-forward and observation bookkeeping. */
    float temp_ff;
    double thermal_obs_time;

    double sat_time, recovery_time;
    uint32_t recoveries;     /* number of watchdog interventions */
    uint32_t rejected;       /* input samples refused as physically impossible */
    uint32_t faults;         /* restarts after the state stopped being finite */
} ins10_fusion;

void ins10_fusion_default_config(ins10_fusion_config *cfg);
void ins10_fusion_init(ins10_fusion *f, const ins10_fusion_config *cfg,
                       const ins10_calibration *cal, const ins10_thermal_model *thermal);
/* Replace the calibration without restarting (after a "cal set" from the host). The
   filter is told what changed: a new IMU calibration re-opens tilt and bias, a new
   magnetometer calibration re-initialises heading and the reference field. */
void ins10_fusion_set_calibration(ins10_fusion *f, const ins10_calibration *cal);
/* Start again from initial alignment, keeping calibration and the thermal model. */
void ins10_fusion_restart(ins10_fusion *f);

/* One IMU sample in sensor units (dps, g, deg C); dt is the sample period in seconds. */
void ins10_fusion_imu(ins10_fusion *f, const float gyro_dps[3], const float accel_g[3],
                      float temp_c, float dt, bool saturated);
/* Tell the filter that IMU samples were lost for about `seconds` (FIFO overflow,
   sensor restart): the rotation during the gap is unknown, so the attitude covariance
   is opened accordingly. */
void ins10_fusion_gap(ins10_fusion *f, float seconds);
/* One magnetometer sample in uT; age_s is how long ago it was taken. */
void ins10_fusion_mag(ins10_fusion *f, const float mag_ut[3], float age_s);

void ins10_fusion_get(const ins10_fusion *f, ins10_output *out);
const ins10_thermal_model *ins10_fusion_thermal_model(const ins10_fusion *f);

#ifdef __cplusplus
}
#endif

#endif
