/* Temperature tracking and the self-learned gyro bias-versus-temperature model.
 *
 * Model, per axis:  bias(T) = c0 + c1 u + c2 u^2,  u = (T - 25 C) / 10 C.
 *
 * It is estimated by a small Kalman filter from observations made while the device is
 * at rest (the gyro then reads its bias). c0 is the turn-on bias: it changes from one
 * power-up to the next, so its variance is re-opened at every boot. c1 and c2 are
 * properties of the part and keep improving across sessions, which is why the model is
 * stored in flash.
 *
 * The attitude filter uses only the slope d(bias)/dT, as a feed-forward on its bias
 * state while the temperature moves; the bias state itself stays the single owner of
 * the bias value, so nothing is compensated twice.
 */
#ifndef INS10_THERMAL_H
#define INS10_THERMAL_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define INS10_THERMAL_T0    25.0f
#define INS10_THERMAL_SCALE 10.0f

typedef struct {
    float coef[3][3]; /* [axis][k], rad/s */
    float cov[6];     /* shared by the axes: 00 01 02 11 12 22, (rad/s)^2 */
    float t_min, t_max;
    uint32_t count;   /* observations absorbed; 0 means "never learned" */
} ins10_thermal_model;

typedef struct {
    ins10_thermal_model model;
    float tau_temp, tau_rate;
    float temp;      /* low-pass filtered temperature, deg C */
    float temp_rate; /* deg C / s */
    bool temp_valid;
} ins10_thermal;

/* slope_prior: 1-sigma of the unknown temperature coefficient, rad/s per deg C. */
void ins10_thermal_model_defaults(ins10_thermal_model *m, float slope_prior);
/* stored may be NULL or have count == 0, in which case the defaults are used. */
void ins10_thermal_init(ins10_thermal *t, const ins10_thermal_model *stored,
                        float slope_prior, float tau_temp, float tau_rate);
/* Feed every temperature sample. */
void ins10_thermal_temp(ins10_thermal *t, float temp_c, float dt);
/* A bias observation made at rest. var is its noise, bias_rw_var the turn-on bias
   random walk accumulated since the previous observation. */
void ins10_thermal_observe(ins10_thermal *t, const float bias[3], float temp_c,
                           float var, float bias_rw_var);
/* The gyro calibration changed under the model. A new bias or matrix shifts what
   the model calls the turn-on bias, so that is re-opened; a new factory temperature
   polynomial changes the remaining slope itself, so the model starts over. */
void ins10_thermal_rebase(ins10_thermal *t, bool slopes_changed, float slope_prior);
void ins10_thermal_predict(const ins10_thermal *t, float temp_c, float bias[3]);
/* d(bias)/dT [rad/s per deg C] and its 1-sigma. */
void ins10_thermal_slope(const ins10_thermal *t, float temp_c, float slope[3], float *sigma);

#ifdef __cplusplus
}
#endif

#endif
