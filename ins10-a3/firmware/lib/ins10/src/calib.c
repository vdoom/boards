#include "ins10/calib.h"
#include "ins10/math3.h"
#include <stddef.h>
#include <string.h>

static void affine_identity(ins10_affine_cal *c) {
    memset(c, 0, sizeof(*c));
    c->matrix[0] = c->matrix[4] = c->matrix[8] = 1.0f;
}

void ins10_cal_defaults(ins10_calibration *cal) {
    memset(cal, 0, sizeof(*cal));
    affine_identity(&cal->gyro);
    affine_identity(&cal->accel);
    affine_identity(&cal->mag);
    cal->gyro_tc.t_ref = 25.0f;
    cal->accel_tc.t_ref = 25.0f;
}

static void apply(const ins10_affine_cal *c, const ins10_tempco *tc, const float raw[3],
                  float temp_c, float out[3]) {
    float d[3];
    float dt = tc ? temp_c - tc->t_ref : 0.0f;
    for (int i = 0; i < 3; ++i) {
        float drift = 0.0f;
        if (tc) {
            /* Horner form of c0 dT + c1 dT^2 + c2 dT^3. */
            for (int k = INS10_TC_TERMS - 1; k >= 0; --k) drift = (drift + tc->coef[i][k]) * dt;
        }
        d[i] = raw[i] - c->bias[i] - drift;
    }
    ins10_m3_mul_v(c->matrix, d, out);
}

void ins10_cal_apply_gyro(const ins10_calibration *cal, const float raw_dps[3],
                          float temp_c, float out_rads[3]) {
    apply(&cal->gyro, &cal->gyro_tc, raw_dps, temp_c, out_rads);
    for (int i = 0; i < 3; ++i) out_rads[i] *= INS10_DEG2RAD;
}

void ins10_cal_apply_accel(const ins10_calibration *cal, const float raw_g[3],
                           float temp_c, float out_g[3]) {
    apply(&cal->accel, &cal->accel_tc, raw_g, temp_c, out_g);
}

void ins10_cal_apply_mag(const ins10_calibration *cal, const float raw_ut[3],
                         float out_ut[3]) {
    apply(&cal->mag, NULL, raw_ut, 0.0f, out_ut);
}

/* Field table: every entry is a run of floats inside ins10_calibration. */
typedef struct {
    const char *name;
    size_t offset;
    int count;
} field_def;

#define FIELD(name, member, count) {name, offsetof(ins10_calibration, member), count}

static const field_def fields[] = {
    FIELD("gyro_bias", gyro.bias, 3),
    FIELD("gyro_matrix", gyro.matrix, 9),
    FIELD("accel_bias", accel.bias, 3),
    FIELD("accel_matrix", accel.matrix, 9),
    FIELD("mag_bias", mag.bias, 3),
    FIELD("mag_matrix", mag.matrix, 9),
    FIELD("gyro_tc_tref", gyro_tc.t_ref, 1),
    FIELD("gyro_tc_x", gyro_tc.coef[0], INS10_TC_TERMS),
    FIELD("gyro_tc_y", gyro_tc.coef[1], INS10_TC_TERMS),
    FIELD("gyro_tc_z", gyro_tc.coef[2], INS10_TC_TERMS),
    FIELD("accel_tc_tref", accel_tc.t_ref, 1),
    FIELD("accel_tc_x", accel_tc.coef[0], INS10_TC_TERMS),
    FIELD("accel_tc_y", accel_tc.coef[1], INS10_TC_TERMS),
    FIELD("accel_tc_z", accel_tc.coef[2], INS10_TC_TERMS),
    FIELD("declination", declination_deg, 1),
    FIELD("mag_field", mag_field_ut, 1),
    FIELD("mag_dip", mag_dip_deg, 1),
};

#define FIELD_COUNT ((int)(sizeof(fields) / sizeof(fields[0])))

bool ins10_cal_set_field(ins10_calibration *cal, const char *name,
                         const float *values, int count) {
    if (strcmp(name, "flags") == 0) {
        if (count != 1 || !(values[0] >= 0.0f && values[0] < 256.0f)) return false;
        cal->flags = (uint32_t)values[0];
        return true;
    }
    for (int i = 0; i < FIELD_COUNT; ++i) {
        if (strcmp(name, fields[i].name) != 0) continue;
        if (count != fields[i].count) return false;
        for (int k = 0; k < count; ++k) {
            if (!isfinite(values[k])) return false;
        }
        memcpy((char *)cal + fields[i].offset, values, (size_t)count * sizeof(float));
        return true;
    }
    return false;
}

void ins10_cal_enumerate(const ins10_calibration *cal, ins10_cal_emit emit, void *ctx) {
    float flags = (float)cal->flags;
    emit(ctx, "flags", &flags, 1);
    for (int i = 0; i < FIELD_COUNT; ++i) {
        float values[9];
        memcpy(values, (const char *)cal + fields[i].offset,
               (size_t)fields[i].count * sizeof(float));
        emit(ctx, fields[i].name, values, fields[i].count);
    }
}
