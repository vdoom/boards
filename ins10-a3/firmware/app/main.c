/* INS10 AHRS firmware for the Raspberry Pi Pico / Pico 2.
 *
 * A single cooperative main loop, no interrupts of its own:
 *   - drain the IMU FIFO and run the fusion library on every sample;
 *   - step the magnetometer and barometer state machines;
 *   - serve the USB console and the output streams.
 * Nothing in the loop blocks for longer than one SPI transfer, and the IMU FIFO covers
 * the rare longer pauses (a flash write), so no gyro sample is ever skipped silently.
 */
#include "app.h"
#include "board.h"
#include "bus.h"
#include "hardware/watchdog.h"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

app_context app;

#define IMU_POLL_US        1000u
#define IMU_SILENCE_US     200000u
#define RETRY_US           1000000u
#define IMU_RETRY_US       3000000u  /* a start attempt blocks for about a second */
#define RATE_WINDOW_US     10000000u
#define AUTOSAVE_US        (30u * 60u * 1000000ull)
#define AUTOSAVE_MIN_NEW   200u   /* new still observations worth a flash write */
#define MAG_QUIET_RATE     (20.0f * INS10_DEG2RAD)

static uint64_t imu_poll_time, imu_data_time, imu_retry_time, mag_retry_time, baro_retry_time;
static uint64_t rate_window_start, att_time, autosave_time;
static uint32_t rate_window_samples, rate_window_busy, rate_window_batch;
static bool rate_window_clean;
static uint32_t raw_seq, raw_first, raw_n;
static int32_t raw_sum[7];

/* ---- Calibration and storage ----------------------------------------------------- */

void app_apply_calibration(void) {
    ins10_fusion_set_calibration(&app.fusion, &app.persist.cal);
}

bool app_save(void) {
    app.persist.thermal = *ins10_fusion_thermal_model(&app.fusion);
    app.persist.settings.att_rate_hz = app.att_rate_hz;
    if (!storage_save(&app.persist)) return false;
    app.stored = true;
    app.thermal_saved_count = app.persist.thermal.count;
    autosave_time = time_us_64();
    return true;
}

/* Writes only the learned thermal model. Calibration fields changed with "cal set"
   but not saved are the user's experiment and must not reach flash behind their back,
   so the record is rebuilt from what is stored, not from RAM. */
static bool autosave(void) {
    static ins10_persist record;
    if (!storage_load(&record)) ins10_persist_defaults(&record);
    record.thermal = *ins10_fusion_thermal_model(&app.fusion);
    autosave_time = time_us_64();
    if (!storage_save(&record)) return false;
    app.stored = true;
    app.thermal_saved_count = record.thermal.count;
    return true;
}

/* ---- IMU ------------------------------------------------------------------------- */

static void imu_lost(float seconds) {
    ++app.imu_gaps;
    rate_window_clean = false;
    ins10_fusion_gap(&app.fusion, seconds);
}

static bool imu_start(void) {
    if (!icm_init(&app.icm)) return false;
    app.sample_dt = 1.0f / app.icm.odr_hz;
    uint64_t now = time_us_64();
    imu_data_time = now;
    rate_window_start = now;
    rate_window_samples = 0;
    rate_window_clean = true;
    return true;
}

static void raw_stream(const icm_sample *s) {
    /* Every IMU sample has a running index, streamed or not. A line carries the index
       of its first sample and the sums of raw_div samples, so the host can average
       exactly and can tell from the indices whether any line was lost. */
    uint32_t index = raw_seq++;
    if (app.raw_div == 0) {
        raw_n = 0;
        return;
    }
    if (raw_n == 0) {
        raw_first = index;
        memset(raw_sum, 0, sizeof(raw_sum));
    }
    for (int i = 0; i < 3; ++i) {
        raw_sum[i] += s->gyro[i];
        raw_sum[i + 3] += s->accel[i];
    }
    raw_sum[6] += s->temp;
    if (++raw_n < app.raw_div) return;
    if (stdio_usb_connected()) {
        printf("$RAW,%" PRIu32 ",%" PRIu32 ",%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32
               ",%" PRId32 ",%" PRId32 ",%" PRId32 "\n",
               raw_first, raw_n, raw_sum[0], raw_sum[1], raw_sum[2],
               raw_sum[3], raw_sum[4], raw_sum[5], raw_sum[6]);
    }
    raw_n = 0;
}

static void imu_service(uint64_t now) {
    static icm_sample samples[ICM_FIFO_BATCH];
    if (!app.icm.ready) {
        if (now - imu_retry_time < IMU_RETRY_US) return;
        imu_retry_time = now;
        if (imu_start()) {
            ++app.imu_restarts;
            ins10_fusion_restart(&app.fusion);
        }
        return;
    }
    if (now - imu_poll_time < IMU_POLL_US) return;
    imu_poll_time = now;
    bool lost = false;
    int invalid = 0;
    uint32_t busy_start = time_us_32();
    int n = icm_read(&app.icm, samples, &lost, &invalid);
    /* A full FIFO holds 0.2 s; if the loop was away for longer, more than that is gone. */
    float since_data = (float)(now - imu_data_time) * 1e-6f;
    if (lost) imu_lost(since_data > 0.2f ? since_data : 0.2f);
    if (invalid > 0) ins10_fusion_gap(&app.fusion, (float)invalid * app.sample_dt);
    for (int i = 0; i < n; ++i) {
        const icm_sample *s = &samples[i];
        float gyro[3], accel[3];
        for (int axis = 0; axis < 3; ++axis) {
            gyro[axis] = (float)s->gyro[axis] * ICM_GYRO_DPS_PER_LSB;
            accel[axis] = (float)s->accel[axis] * ICM_ACCEL_G_PER_LSB;
        }
        float temp = (float)s->temp / ICM_TEMP_LSB_PER_C + ICM_TEMP_OFFSET_C;
        ins10_fusion_imu(&app.fusion, gyro, accel, temp, app.sample_dt, s->saturated);
        raw_stream(s);
    }
    rate_window_busy += time_us_32() - busy_start;
    if (n > 0) {
        imu_data_time = now;
        rate_window_samples += (uint32_t)n;
        if ((uint32_t)n > rate_window_batch) rate_window_batch = (uint32_t)n;
    } else if (now - imu_data_time > IMU_SILENCE_US) {
        /* Bus errors or a sensor that stopped: start it again from reset, at once. */
        app.icm.ready = false;
        imu_retry_time = now - IMU_RETRY_US;
        imu_lost(since_data);
        return;
    }
    if (now - rate_window_start >= RATE_WINDOW_US) {
        float hz = 1e6f * (float)rate_window_samples / (float)(now - rate_window_start);
        if (rate_window_clean) {
            app.measured_hz = hz;
            /* On the internal oscillator the nominal rate is only good to a percent
               or so; follow the measurement. With CLKIN the rate is exact. */
            if (!app.icm.external_clock && fabsf(hz * app.sample_dt - 1.0f) < 0.03f) {
                app.sample_dt = 1.0f / (0.7f / app.sample_dt + 0.3f * hz);
            }
        }
        /* Headroom indicators for 'status': on the RP2040 all floating point is
           software, so this is the number to watch. */
        app.fusion_load = (float)rate_window_busy / (float)(now - rate_window_start);
        app.batch_max = rate_window_batch;
        rate_window_start = now;
        rate_window_samples = rate_window_busy = rate_window_batch = 0;
        rate_window_clean = true;
    }
}

/* ---- Magnetometer and barometer --------------------------------------------------- */

static void mag_service(uint64_t now) {
    if (!app.persist.settings.mag_enable) return;
    if (!app.mmc.ready) {
        if (now - mag_retry_time < 2 * RETRY_US) return;
        mag_retry_time = now;
        if (mmc_init(&app.mmc)) ++app.mag_restarts;
        return;
    }
    const float *w = app.fusion.last_rate;
    bool quiet = sqrtf(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]) < MAG_QUIET_RATE;
    if (!mmc_task(&app.mmc, now, quiet)) {
        mag_retry_time = now;
        return;
    }
    float field[3];
    uint64_t t_mid;
    if (!mmc_take(&app.mmc, field, &t_mid)) return;
    float age = (float)(now - t_mid) * 1e-6f;
    ins10_fusion_mag(&app.fusion, field, age);
    if (app.mag_stream && stdio_usb_connected()) {
        printf("$MAG,%" PRIu64 ",%.3f,%.3f,%.3f\n", t_mid / 1000,
               (double)field[0], (double)field[1], (double)field[2]);
    }
}

static void baro_service(uint64_t now) {
    if (!app.lps.ready) {
        if (now - baro_retry_time < 2 * RETRY_US) return;
        baro_retry_time = now;
        if (lps_init(&app.lps)) ++app.baro_restarts;
        return;
    }
    if (!lps_task(&app.lps, now)) {
        baro_retry_time = now;
        app.baro_valid = false;
        return;
    }
    if (!lps_take(&app.lps, &app.baro_hpa, &app.baro_temp_c)) return;
    app.baro_valid = true;
    if (app.baro_stream && stdio_usb_connected()) {
        printf("$BARO,%" PRIu64 ",%.3f,%.2f,%.2f\n", now / 1000, (double)app.baro_hpa,
               (double)app.baro_temp_c, (double)lps_altitude_m(app.baro_hpa));
    }
}

/* ---- Output and housekeeping ------------------------------------------------------ */

static void attitude_stream(uint64_t now) {
    if (app.att_rate_hz == 0 || !stdio_usb_connected()) return;
    uint64_t period = 1000000u / app.att_rate_hz;
    if (now - att_time < period) return;
    att_time += period;
    if (now - att_time > period) att_time = now;
    ins10_output o;
    ins10_fusion_get(&app.fusion, &o);
    printf("$ATT,%" PRIu64 ",%.6f,%.6f,%.6f,%.6f,%.2f,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%03" PRIX32 "\n",
           now / 1000, (double)o.q[0], (double)o.q[1], (double)o.q[2], (double)o.q[3],
           (double)o.roll_deg, (double)o.pitch_deg, (double)o.yaw_deg, (double)o.heading_deg,
           (double)o.gyro_bias_dps[0], (double)o.gyro_bias_dps[1], (double)o.gyro_bias_dps[2],
           (double)o.tilt_sigma_deg, (double)o.heading_sigma_deg, (double)o.temp_c, o.flags);
}

static void led_service(uint64_t now) {
#ifdef PICO_DEFAULT_LED_PIN
    /* Solid: IMU fault. Fast blink: aligning. Short flash every second: running.
       This runs on every pass of the loop, so it must not compute anything. */
    bool on;
    if (!app.icm.ready) on = true;
    else if (app.fusion.aligned) on = now % 1000000u < 50000u;
    else on = now % 200000u < 100000u;
    gpio_put(PICO_DEFAULT_LED_PIN, on);
#else
    (void)now;
#endif
}

/* The thermal model keeps improving while the board is in use. Write it back now and
   then, only at rest (a flash write pauses everything for tens of milliseconds) and
   only when there is something new worth the erase cycle. */
static void autosave_service(uint64_t now) {
    if (now - autosave_time < AUTOSAVE_US) return;
    const ins10_thermal_model *model = ins10_fusion_thermal_model(&app.fusion);
    if (!app.fusion.motion.still || model->count < app.thermal_saved_count + AUTOSAVE_MIN_NEW) {
        autosave_time = now - AUTOSAVE_US + 10000000u; /* look again in 10 s */
        return;
    }
    autosave();
}

static void connection_service(void) {
    static bool connected;
    bool now_connected = stdio_usb_connected();
    if (now_connected && !connected) console_banner();
    connected = now_connected;
}

int main(void) {
    bus_init();
    /* Park the optional clock-measurement pin of the bench wiring as an input. */
    gpio_init(PIN_CLOCK_TEST);
    gpio_set_dir(PIN_CLOCK_TEST, GPIO_IN);
    gpio_pull_down(PIN_CLOCK_TEST);
#ifdef PICO_DEFAULT_LED_PIN
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, true);
#endif
    stdio_init_all();
    watchdog_enable(5000, true);

    ins10_persist_defaults(&app.persist);
    app.stored = storage_load(&app.persist);
    app.att_rate_hz = app.persist.settings.att_rate_hz;
    app.thermal_saved_count = app.persist.thermal.count;

    sleep_ms(50); /* sensor power-up */
    ins10_fusion_default_config(&app.config);
    app.sample_dt = 1.0f / app.config.sample_rate_hz;
    if (imu_start()) app.config.sample_rate_hz = app.icm.odr_hz;
    watchdog_update();
    if (app.persist.settings.mag_enable) mmc_init(&app.mmc);
    lps_init(&app.lps);
    ins10_fusion_init(&app.fusion, &app.config, &app.persist.cal, &app.persist.thermal);
    autosave_time = time_us_64();

    for (;;) {
        uint64_t now = time_us_64();
        watchdog_update();
        imu_service(now);
        mag_service(now);
        baro_service(now);
        console_poll();
        attitude_stream(now);
        connection_service();
        led_service(now);
        autosave_service(now);
    }
}
