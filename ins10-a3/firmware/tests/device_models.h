/* Register-level models of the three sensors, behind the drivers' bus interface.
 *
 * They implement what the drivers rely on, from the datasheets: the ICM-42688-P FIFO
 * in 20-bit format with its two clock sources, the MMC5983MA single-shot conversion
 * with SET/RESET polarity, and the LPS22DF data-ready cycle. Tests set the physical
 * inputs and inject faults through the variables below.
 */
#ifndef INS10_DEVICE_MODELS_H
#define INS10_DEVICE_MODELS_H

#include <stdbool.h>
#include <stdint.h>

void models_reset(void);

/* ---- ICM-42688-P ---- */
extern bool icm_model_present;
extern bool icm_model_clkin;            /* a 32.768 kHz clock is on pin 9 */
extern double icm_model_internal_hz;    /* actual output rate on the internal oscillator */
extern bool icm_model_bus_fault;        /* every transfer fails */
/* Content of sample number k, 20-bit counts and raw temperature. */
extern void (*icm_model_sample)(uint32_t k, int32_t gyro[3], int32_t accel[3], int16_t *temp);
extern uint32_t icm_model_produced;     /* samples generated since the last FIFO flush */

/* ---- MMC5983MA ---- */
extern bool mmc_model_present;
extern double mmc_model_field_ut[3];    /* true field at the sensor */
extern double mmc_model_offset_counts[3]; /* bridge offset, drifts with temperature */
extern int mmc_model_stuck_conversions; /* this many conversions never report DONE */
extern bool mmc_model_stale_done;       /* DONE stays set briefly after a new command */
extern int mmc_model_polarity;          /* +1 after SET, -1 after RESET */

/* ---- LPS22DF ---- */
extern bool lps_model_present;
extern double lps_model_hpa, lps_model_temp_c;
extern bool lps_model_silent;           /* stops producing data */

#endif
