/* INS10 AHRS application: shared state between the main loop and the console. */
#ifndef INS10_APP_H
#define INS10_APP_H

#include "icm42688.h"
#include "ins10/ins10.h"
#include "lps22df.h"
#include "mmc5983ma.h"

#define APP_NAME    "INS10 AHRS"
#define APP_VERSION "2.0.0"

typedef struct {
    ins10_fusion fusion;
    ins10_fusion_config config;
    ins10_persist persist; /* calibration, thermal model and settings as stored or edited */
    bool stored;           /* a valid record was found in flash at start-up */

    icm_state icm;
    mmc_state mmc;
    lps_state lps;
    float sample_dt;       /* IMU sample period used by the filter, s */
    float measured_hz;     /* IMU rate against the Pico clock, last 10 s */
    float fusion_load;     /* fraction of CPU time spent reading the IMU and filtering, last 10 s */
    uint32_t batch_max;    /* most IMU samples found waiting in one pass, last 10 s */
    uint32_t imu_gaps;     /* FIFO overflows and sensor restarts */
    uint32_t imu_restarts, mag_restarts, baro_restarts;

    /* Output streams (see docs/PROTOCOL.md). */
    uint16_t att_rate_hz;  /* $ATT lines per second, 0 = off */
    uint16_t raw_div;      /* $RAW: sum of this many IMU samples per line, 0 = off */
    bool mag_stream, baro_stream;

    float baro_hpa, baro_temp_c;
    bool baro_valid;
    uint32_t thermal_saved_count; /* thermal model observations at the last save */
} app_context;

extern app_context app;

/* console.c */
void console_poll(void);
void console_banner(void);

/* storage.c */
bool storage_load(ins10_persist *p);
bool storage_save(const ins10_persist *p);
bool storage_erase(void);

/* main.c */
void app_apply_calibration(void);
/* Copies the live thermal model into the persistent block and writes it to flash. */
bool app_save(void);

#endif
