#include "ins10/math3.h"

void ins10_quat_mul(const float a[4], const float b[4], float out[4]) {
    float w = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
    float x = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
    float y = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
    float z = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
    out[0] = w; out[1] = x; out[2] = y; out[3] = z;
}

void ins10_quat_normalize(float q[4]) {
    float n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (!(n > 1e-12f)) {
        q[0] = 1.0f; q[1] = q[2] = q[3] = 0.0f;
        return;
    }
    float inv = 1.0f / n;
    q[0] *= inv; q[1] *= inv; q[2] *= inv; q[3] *= inv;
}

void ins10_quat_from_rotvec(const float v[3], float q[4]) {
    float a2 = ins10_dot(v, v);
    float c, s; /* s = sin(a/2)/a */
    if (a2 < 1e-6f) {
        c = 1.0f - a2 * 0.125f;
        s = 0.5f - a2 * (1.0f / 48.0f);
    } else {
        float a = sqrtf(a2);
        c = cosf(0.5f * a);
        s = sinf(0.5f * a) / a;
    }
    q[0] = c; q[1] = s * v[0]; q[2] = s * v[1]; q[3] = s * v[2];
}

void ins10_quat_to_dcm(const float q[4], float r[9]) {
    float w = q[0], x = q[1], y = q[2], z = q[3];
    r[0] = 1.0f - 2.0f * (y * y + z * z);
    r[1] = 2.0f * (x * y - w * z);
    r[2] = 2.0f * (x * z + w * y);
    r[3] = 2.0f * (x * y + w * z);
    r[4] = 1.0f - 2.0f * (x * x + z * z);
    r[5] = 2.0f * (y * z - w * x);
    r[6] = 2.0f * (x * z - w * y);
    r[7] = 2.0f * (y * z + w * x);
    r[8] = 1.0f - 2.0f * (x * x + y * y);
}

void ins10_dcm_to_quat(const float r[9], float q[4]) {
    float trace = r[0] + r[4] + r[8];
    if (trace > 0.0f) {
        float s = sqrtf(trace + 1.0f) * 2.0f;
        q[0] = 0.25f * s;
        q[1] = (r[7] - r[5]) / s;
        q[2] = (r[2] - r[6]) / s;
        q[3] = (r[3] - r[1]) / s;
    } else if (r[0] > r[4] && r[0] > r[8]) {
        float s = sqrtf(1.0f + r[0] - r[4] - r[8]) * 2.0f;
        q[0] = (r[7] - r[5]) / s;
        q[1] = 0.25f * s;
        q[2] = (r[1] + r[3]) / s;
        q[3] = (r[2] + r[6]) / s;
    } else if (r[4] > r[8]) {
        float s = sqrtf(1.0f + r[4] - r[0] - r[8]) * 2.0f;
        q[0] = (r[2] - r[6]) / s;
        q[1] = (r[1] + r[3]) / s;
        q[2] = 0.25f * s;
        q[3] = (r[5] + r[7]) / s;
    } else {
        float s = sqrtf(1.0f + r[8] - r[0] - r[4]) * 2.0f;
        q[0] = (r[3] - r[1]) / s;
        q[1] = (r[2] + r[6]) / s;
        q[2] = (r[5] + r[7]) / s;
        q[3] = 0.25f * s;
    }
    ins10_quat_normalize(q);
}

void ins10_rotvec_to_dcm(const float v[3], float r[9]) {
    float a2 = ins10_dot(v, v);
    float sa, ca; /* sa = sin(a)/a, ca = (1 - cos(a))/a^2 */
    if (a2 < 1e-6f) {
        sa = 1.0f - a2 * (1.0f / 6.0f);
        ca = 0.5f - a2 * (1.0f / 24.0f);
    } else {
        float a = sqrtf(a2);
        sa = sinf(a) / a;
        ca = (1.0f - cosf(a)) / a2;
    }
    float x = v[0], y = v[1], z = v[2];
    r[0] = 1.0f - ca * (y * y + z * z);
    r[1] = -sa * z + ca * x * y;
    r[2] = sa * y + ca * x * z;
    r[3] = sa * z + ca * x * y;
    r[4] = 1.0f - ca * (x * x + z * z);
    r[5] = -sa * x + ca * y * z;
    r[6] = -sa * y + ca * x * z;
    r[7] = sa * x + ca * y * z;
    r[8] = 1.0f - ca * (x * x + y * y);
}

void ins10_quat_rotate(const float q[4], const float v[3], float out[3]) {
    float r[9], tmp[3];
    ins10_quat_to_dcm(q, r);
    ins10_m3_mul_v(r, v, tmp);
    out[0] = tmp[0]; out[1] = tmp[1]; out[2] = tmp[2];
}

void ins10_quat_rotate_inv(const float q[4], const float v[3], float out[3]) {
    float r[9];
    ins10_quat_to_dcm(q, r);
    float x = r[0] * v[0] + r[3] * v[1] + r[6] * v[2];
    float y = r[1] * v[0] + r[4] * v[1] + r[7] * v[2];
    float z = r[2] * v[0] + r[5] * v[1] + r[8] * v[2];
    out[0] = x; out[1] = y; out[2] = z;
}

void ins10_quat_to_euler(const float q[4], float *roll, float *pitch, float *yaw) {
    float w = q[0], x = q[1], y = q[2], z = q[3];
    *roll = atan2f(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y));
    *pitch = asinf(ins10_clampf(2.0f * (w * y - x * z), -1.0f, 1.0f));
    *yaw = atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
}
