/* ICM-42688-P driver: FIFO streaming in the 20-bit high-resolution format.
 *
 * Configuration: gyro +/-2000 dps and accelerometer +/-16 g (the ranges the 20-bit
 * format is defined for), 500 Hz output data rate, low-noise mode. With the board's
 * 32.768 kHz oscillator on CLKIN the rate becomes exactly 512 Hz; without it the
 * driver falls back to the internal clock and reports the measured rate.
 *
 * The FIFO is the reason no interrupt line or timer is needed: every sample is
 * delivered in order even when the main loop is briefly busy, so gyro integration
 * never skips a sample.
 */
#ifndef INS10_ICM42688_H
#define INS10_ICM42688_H

#include <stdbool.h>
#include <stdint.h>

/* 20-bit sample scales: full range over +/-2^19 counts. */
#define ICM_GYRO_DPS_PER_LSB  (2000.0f / 524288.0f)
#define ICM_ACCEL_G_PER_LSB   (16.0f / 524288.0f)
#define ICM_TEMP_LSB_PER_C    132.48f
#define ICM_TEMP_OFFSET_C     25.0f
#define ICM_FIFO_BATCH        48

typedef struct {
    int32_t gyro[3];   /* 20-bit counts */
    int32_t accel[3];
    int16_t temp;
    bool saturated;    /* any axis at the end of its range */
} icm_sample;

typedef struct {
    bool ready;
    bool external_clock;
    float odr_hz;            /* exact 512 with CLKIN, measured otherwise */
    uint32_t samples;        /* delivered since init */
    uint32_t overflows;      /* FIFO filled up: samples were lost */
    uint32_t bad_packets;
    uint32_t bus_errors;
} icm_state;

/* Full reset and configuration; blocks for about one second. */
bool icm_init(icm_state *st);
/* Drains up to ICM_FIFO_BATCH samples. Returns the count, or -1 on a bus error.
   *lost is set when the FIFO overflowed or lost step and was flushed; *invalid counts
   packets in which the sensor marked a value as not available. */
int icm_read(icm_state *st, icm_sample *out, bool *lost, int *invalid);

#endif
