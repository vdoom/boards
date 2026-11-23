/* Small fixed-size vector, rotation and quaternion helpers.
 *
 * Conventions used by the whole library:
 *   - quaternions are Hamilton, stored as {w, x, y, z};
 *   - q rotates body-frame vectors into the navigation frame: v_nav = R(q) v_body;
 *   - 3x3 matrices are row-major float[9].
 */
#ifndef INS10_MATH3_H
#define INS10_MATH3_H

#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

#define INS10_PI      3.14159265358979323846f
#define INS10_DEG2RAD (INS10_PI / 180.0f)
#define INS10_RAD2DEG (180.0f / INS10_PI)

static inline float ins10_dot(const float a[3], const float b[3]) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

/* out must not alias a or b. */
static inline void ins10_cross(const float a[3], const float b[3], float out[3]) {
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

static inline float ins10_norm(const float a[3]) {
    return sqrtf(ins10_dot(a, a));
}

/* out must not alias v. */
static inline void ins10_m3_mul_v(const float m[9], const float v[3], float out[3]) {
    out[0] = m[0] * v[0] + m[1] * v[1] + m[2] * v[2];
    out[1] = m[3] * v[0] + m[4] * v[1] + m[5] * v[2];
    out[2] = m[6] * v[0] + m[7] * v[1] + m[8] * v[2];
}

static inline float ins10_clampf(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}

void ins10_quat_mul(const float a[4], const float b[4], float out[4]);
void ins10_quat_normalize(float q[4]);
void ins10_quat_from_rotvec(const float v[3], float q[4]);
void ins10_quat_to_dcm(const float q[4], float r[9]);
void ins10_dcm_to_quat(const float r[9], float q[4]);
void ins10_rotvec_to_dcm(const float v[3], float r[9]);
/* v_nav = R(q) v_body */
void ins10_quat_rotate(const float q[4], const float v[3], float out[3]);
/* v_body = R(q)^T v_nav */
void ins10_quat_rotate_inv(const float q[4], const float v[3], float out[3]);
/* ZYX (yaw-pitch-roll) Euler angles of q, radians. */
void ins10_quat_to_euler(const float q[4], float *roll, float *pitch, float *yaw);

#ifdef __cplusplus
}
#endif

#endif
