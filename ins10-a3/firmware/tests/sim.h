/* Sensor simulator for the host tests.
 *
 * The true attitude is integrated in double precision at eight times the IMU rate
 * from an analytic body-rate profile, so it is independent of the integrator under
 * test. Sensor outputs are derived from it with configurable noise, bias, temperature
 * drift, scale error, saturation, external acceleration and magnetic disturbance.
 */
#ifndef INS10_SIM_H
#define INS10_SIM_H

#include <stdbool.h>
#include <stdint.h>

typedef void (*sim_vec_fn)(double t, double out[3]);
typedef double (*sim_scalar_fn)(double t);

#define SIM_HISTORY 8

typedef struct {
    /* Configuration. */
    double fs;              /* IMU sample rate, Hz */
    int mag_div;            /* one mag sample every mag_div IMU samples */
    int mag_delay;          /* mag sample age in IMU samples */
    sim_vec_fn rate;        /* true body rate, rad/s */
    sim_vec_fn lin_acc;     /* external acceleration, nav frame, g (optional) */
    sim_vec_fn mag_dist;    /* magnetic disturbance, nav frame, uT (optional) */
    sim_scalar_fn temp;     /* deg C (optional, default 25) */
    double gyro_noise;      /* per-sample 1-sigma, rad/s */
    double accel_noise;     /* g */
    double mag_noise;       /* uT */
    double gyro_bias[3];    /* rad/s at 25 C */
    double gyro_tempco[3];  /* rad/s per deg C */
    double gyro_scale;      /* fractional scale error */
    double gyro_limit;      /* saturation, rad/s */
    double accel_bias[3];   /* g */
    double accel_scale;     /* fractional scale error */
    double mag_bias[3];     /* residual hard iron, body frame, uT */
    double mag_scale[3];    /* residual per-axis scale error, fraction */
    double mag_field[3];    /* local field, nav frame (ENU), uT */
    /* State. */
    double t;
    double q[4];            /* true attitude, body -> nav */
    double q_hist[SIM_HISTORY][4];
    long k;
    uint64_t rng;
} sim;

typedef struct {
    float gyro_dps[3];
    float accel_g[3];
    float temp_c;
    bool saturated;
    bool mag_valid;
    float mag_ut[3];
    float mag_age_s;
} sim_sample;

/* Fills in realistic ICM-42688-P / MMC5983MA noise levels and a mid-latitude field. */
void sim_defaults(sim *s, uint64_t seed);
void sim_step(sim *s, sim_sample *out);
/* Current true gyro bias, rad/s. */
void sim_true_bias(const sim *s, double bias[3]);
/* Errors of an estimated attitude, degrees. heading is signed. */
void sim_errors(const sim *s, const float q_est[4], double *tilt, double *heading, double *total);
double sim_gauss(sim *s);

#endif
