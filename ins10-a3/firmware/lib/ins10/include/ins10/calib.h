/* Sensor calibration data and its application.
 *
 * Every sensor uses the same model:
 *     corrected = matrix * (raw - bias - tempco(T))
 * with raw and bias in the sensor's natural unit (gyro dps, accel g, mag uT). The
 * matrix holds scale factors, cross-axis terms and, for the magnetometer, soft iron and
 * the rotation into the IMU frame. tempco is a per-axis polynomial in (T - t_ref).
 *
 * The values are produced by the host tool (tools/ins10tool) and stored in flash.
 */
#ifndef INS10_CALIB_H
#define INS10_CALIB_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define INS10_CAL_GYRO     (1u << 0) /* gyro bias/matrix measured */
#define INS10_CAL_ACCEL    (1u << 1)
#define INS10_CAL_MAG      (1u << 2)
#define INS10_CAL_GYRO_TC  (1u << 3) /* gyro bias-vs-temperature polynomial measured */
#define INS10_CAL_ACCEL_TC (1u << 4)

#define INS10_TC_TERMS 3

typedef struct {
    float bias[3];
    float matrix[9];
} ins10_affine_cal;

typedef struct {
    float t_ref;                     /* deg C */
    float coef[3][INS10_TC_TERMS];   /* [axis][k] multiplies (T - t_ref)^(k+1) */
} ins10_tempco;

typedef struct {
    uint32_t flags;
    ins10_affine_cal gyro;
    ins10_affine_cal accel;
    ins10_affine_cal mag;
    ins10_tempco gyro_tc;
    ins10_tempco accel_tc;
    float declination_deg; /* east positive; 0 makes the output relative to magnetic north */
    float mag_field_ut;    /* local field found by the calibration, informational */
    float mag_dip_deg;
} ins10_calibration;

void ins10_cal_defaults(ins10_calibration *cal);

/* Calibrated outputs: gyro in rad/s, accel in g, mag in uT. */
void ins10_cal_apply_gyro(const ins10_calibration *cal, const float raw_dps[3],
                          float temp_c, float out_rads[3]);
void ins10_cal_apply_accel(const ins10_calibration *cal, const float raw_g[3],
                           float temp_c, float out_g[3]);
void ins10_cal_apply_mag(const ins10_calibration *cal, const float raw_ut[3],
                         float out_ut[3]);

/* Text form shared by the firmware console, the host tool and the replay utility.
   A field is "name v0 v1 ...". Returns false for an unknown name, a wrong value count
   or a non-finite value. */
bool ins10_cal_set_field(ins10_calibration *cal, const char *name,
                         const float *values, int count);
typedef void (*ins10_cal_emit)(void *ctx, const char *name, const float *values, int count);
void ins10_cal_enumerate(const ins10_calibration *cal, ins10_cal_emit emit, void *ctx);

#ifdef __cplusplus
}
#endif

#endif
