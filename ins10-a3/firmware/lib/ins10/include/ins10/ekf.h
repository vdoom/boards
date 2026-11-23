/* Multiplicative (error-state) extended Kalman filter for attitude.
 *
 * Nominal state: attitude quaternion q (body -> navigation, ENU) and gyro bias b [rad/s].
 * Error state:   x = [dtheta (3), dbias (3)], with q_true = q (x) exp(dtheta) (body-frame
 *                error) and b_true = b + dbias.
 *
 * The filter itself is deliberately "dumb": it integrates, propagates covariance and
 * applies measurement updates with whatever noise it is given. Deciding how much to
 * trust each measurement is the job of the adaptive layer in fusion.c.
 */
#ifndef INS10_EKF_H
#define INS10_EKF_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define INS10_EKF_N 6

typedef struct {
    float q[4];
    float bias[3];
    float P[INS10_EKF_N][INS10_EKF_N];
    /* Attitude integrator and the rotation accumulated since the last covariance step. */
    float prev_rate[2][3]; /* [0] is the previous sample, [1] the one before */
    int prev_count;
    float dtheta_acc[3];
    float dt_acc;
} ins10_ekf;

/* Variances: tilt and yaw in rad^2 (yaw is rotation about the navigation vertical),
   bias in (rad/s)^2. */
void ins10_ekf_init(ins10_ekf *e, const float q[4], const float bias[3],
                    float var_tilt, float var_yaw, float var_bias);

/* Attitude kinematics. Call for every gyro sample; rate is the calibrated body rate
   [rad/s] before bias removal. Uses the two previous samples as well: a third-order
   quadrature of the rate plus the commutation (coning) term. */
void ins10_ekf_integrate(ins10_ekf *e, const float rate[3], float dt);

/* Covariance propagation over the interval integrated since the previous call.
   q_att [rad^2] and q_bias [(rad/s)^2] are the process noise totals for the interval. */
void ins10_ekf_propagate(ins10_ekf *e, float q_att, float q_bias);

/* Deterministic bias change (thermal feed-forward). */
void ins10_ekf_shift_bias(ins10_ekf *e, const float delta[3]);

/* Covariance inflation, used for gyro saturation and divergence recovery. */
void ins10_ekf_add_attitude_var(ins10_ekf *e, float var_tilt, float var_yaw);
void ins10_ekf_add_bias_var(ins10_ekf *e, float var);

/* Rotate the estimate about the navigation vertical and set the yaw variance,
   decorrelating yaw from the other states. Used to initialise heading. */
void ins10_ekf_reset_yaw(ins10_ekf *e, float angle, float var_yaw);

/* Gravity: accel is specific force in g, expected to equal the body-frame "up" vector. */
void ins10_ekf_gravity_innovation(const ins10_ekf *e, const float accel[3],
                                  float resid[3], float *hph_trace);
void ins10_ekf_update_gravity(ins10_ekf *e, const float accel[3], float var);

/* Heading from a magnetic field vector.
   north:   unit horizontal direction (east, north) of the reference field;
   tan_dip: tangent of the dip angle, positive when the field points below horizontal.
   Returns false when the field is too close to vertical to carry heading.
   phi is the horizontal angle between the rotated measurement and the reference,
   h the measurement row for phi, hph = h P h^T. */
bool ins10_ekf_heading_innovation(const ins10_ekf *e, const float mag[3],
                                  const float north[2], float tan_dip,
                                  float *phi, float h[INS10_EKF_N], float *hph);
/* yaw_only restricts the correction to rotation about the navigation vertical, so a
   magnetic disturbance can never move roll or pitch. */
void ins10_ekf_update_heading(ins10_ekf *e, const float h[INS10_EKF_N], float phi,
                              float var, bool yaw_only);

/* Zero angular rate: while the device is at rest the gyro reads its bias. How well
   "at rest" is known differs between rotation about horizontal axes (gravity would
   move) and about the vertical (only the magnetic field would), hence two variances,
   in (rad/s)^2. */
void ins10_ekf_update_zero_rate(ins10_ekf *e, const float rate[3], float var_horizontal,
                                float var_vertical);

/* Body-frame unit vector pointing up (third row of the body -> nav rotation). */
void ins10_ekf_up_body(const ins10_ekf *e, float up[3]);
/* 1-sigma tilt (per axis) and heading uncertainty [rad], bias uncertainty [rad/s]. */
void ins10_ekf_sigmas(const ins10_ekf *e, float *tilt, float *yaw, float *bias);

#ifdef __cplusplus
}
#endif

#endif
