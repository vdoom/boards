/* USB console: a line-oriented text protocol, usable from a terminal and by the host
 * tool. Every command is answered by zero or more lines and then exactly one line
 * starting with "OK" or "ERR". Informational lines start with "#", stream records with
 * "$". See docs/PROTOCOL.md.
 */
#include "app.h"
#include "board.h"
#include "hardware/watchdog.h"
#include "pico/bootrom.h"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define LINE_MAX_LEN 200
#define MAX_ARGS 14

static const char *board_name(void) {
#if PICO_RP2350
    return "pico2";
#else
    return "pico";
#endif
}

void console_banner(void) {
    printf("# %s %s (%s), library %s. Type 'help'.\n", APP_NAME, APP_VERSION, board_name(),
           INS10_LIB_VERSION);
}

static void reply_ok(void) { puts("OK"); }
static void reply_err(const char *why) { printf("ERR %s\n", why); }

static void cmd_help(void) {
    puts("# info                     firmware, sensors, scale factors");
    puts("# status                   filter state and health counters");
    puts("# stream att <hz>          attitude output rate (0 = off, max 200)");
    puts("# stream raw <n>           raw IMU: sum of n samples per line (0 = off, 1 = every sample)");
    puts("# stream mag <0|1>         raw magnetometer");
    puts("# stream baro <0|1>        barometer");
    puts("# stream off               stop all streams");
    puts("# cal show                 print the calibration as 'cal set' lines");
    puts("# cal set <name> <v...>    change one calibration field (RAM only)");
    puts("# cal save                 store calibration, thermal model and settings in flash");
    puts("# cal load                 discard changes, reload from flash");
    puts("# cal erase                factory defaults, erase flash");
    puts("# thermal show | reset     learned gyro bias-vs-temperature model");
    puts("# set mag <0|1>            use the magnetometer (stored with 'cal save')");
    puts("# reset                    restart the filter from initial alignment");
    puts("# reboot | bootsel         restart the Pico / enter the USB bootloader");
}

static void cmd_info(void) {
    printf("# fw=%s version=%s board=%s lib=%s\n", "ins10-ahrs", APP_VERSION, board_name(),
           INS10_LIB_VERSION);
    printf("# imu=%s clock=%s odr_hz=%.3f measured_hz=%.3f\n", app.icm.ready ? "ok" : "fault",
           app.icm.external_clock ? "external" : "internal",
           (double)(1.0f / app.sample_dt), (double)app.measured_hz);
    printf("# gyro_dps_per_lsb=%.9g accel_g_per_lsb=%.9g temp_lsb_per_c=%.6g temp_offset_c=%.6g\n",
           (double)ICM_GYRO_DPS_PER_LSB, (double)ICM_ACCEL_G_PER_LSB,
           (double)ICM_TEMP_LSB_PER_C, (double)ICM_TEMP_OFFSET_C);
    printf("# mag=%s mag_enabled=%u baro=%s\n", app.mmc.ready ? "ok" : "fault",
           (unsigned)app.persist.settings.mag_enable, app.lps.ready ? "ok" : "fault");
    printf("# att_rate_hz=%u raw_div=%u\n", (unsigned)app.att_rate_hz, (unsigned)app.raw_div);
    /* "watchdog" here after an unexpected restart means the main loop hung. */
    printf("# last_reset=%s\n", watchdog_enable_caused_reboot() ? "watchdog" : "normal");
    printf("# cal_flags=%" PRIu32 " stored=%u thermal_count=%" PRIu32 "\n", app.persist.cal.flags,
           (unsigned)app.stored, ins10_fusion_thermal_model(&app.fusion)->count);
    reply_ok();
}

static void print_flag(uint32_t flags, uint32_t bit, const char *name) {
    if (flags & bit) printf(" %s", name);
}

static void cmd_status(void) {
    ins10_output o;
    ins10_fusion_get(&app.fusion, &o);
    printf("# flags=0x%03" PRIX32 ":", o.flags);
    print_flag(o.flags, INS10_F_ALIGNED, "aligned");
    print_flag(o.flags, INS10_F_COARSE, "coarse");
    print_flag(o.flags, INS10_F_STILL, "still");
    print_flag(o.flags, INS10_F_HEADING, "heading");
    print_flag(o.flags, INS10_F_ACC_DISTURBED, "acc-disturbed");
    print_flag(o.flags, INS10_F_MAG_DISTURBED, "mag-disturbed");
    print_flag(o.flags, INS10_F_MAG_UNCAL, "mag-uncalibrated");
    print_flag(o.flags, INS10_F_GYRO_SAT, "saturated");
    print_flag(o.flags, INS10_F_THERMAL, "thermal-transient");
    print_flag(o.flags, INS10_F_RECOVERY, "recovery");
    print_flag(o.flags, INS10_F_MAG_STALE, "mag-stale");
    putchar('\n');
    printf("# roll=%.2f pitch=%.2f yaw=%.2f heading=%.2f deg, sigma tilt=%.2f heading=%.2f deg\n",
           (double)o.roll_deg, (double)o.pitch_deg, (double)o.yaw_deg, (double)o.heading_deg,
           (double)o.tilt_sigma_deg, (double)o.heading_sigma_deg);
    printf("# gyro_bias=%.4f %.4f %.4f dps (sigma %.4f)\n", (double)o.gyro_bias_dps[0],
           (double)o.gyro_bias_dps[1], (double)o.gyro_bias_dps[2], (double)o.bias_sigma_dps);
    float an = sqrtf(o.accel_g[0] * o.accel_g[0] + o.accel_g[1] * o.accel_g[1] + o.accel_g[2] * o.accel_g[2]);
    float mn = sqrtf(o.mag_ut[0] * o.mag_ut[0] + o.mag_ut[1] * o.mag_ut[1] + o.mag_ut[2] * o.mag_ut[2]);
    printf("# accel=%.4f %.4f %.4f g (norm %.4f), disturbance %.3f g\n", (double)o.accel_g[0],
           (double)o.accel_g[1], (double)o.accel_g[2], (double)an, (double)o.accel_disturbance_g);
    printf("# mag=%.2f %.2f %.2f uT (norm %.2f), reference %.2f uT dip %.1f deg, deviation %.2f uT\n",
           (double)o.mag_ut[0], (double)o.mag_ut[1], (double)o.mag_ut[2], (double)mn,
           (double)o.mag_field_ut, (double)o.mag_dip_deg, (double)o.mag_disturbance_ut);
    printf("# temp=%.2f C, rate %.4f C/s\n", (double)o.temp_c, (double)o.temp_rate_cps);
    if (app.baro_valid) {
        printf("# baro=%.3f hPa, %.2f C, altitude %.1f m\n", (double)app.baro_hpa,
               (double)app.baro_temp_c, (double)lps_altitude_m(app.baro_hpa));
    }
    printf("# imu: samples=%" PRIu32 " gaps=%" PRIu32 " overflows=%" PRIu32 " bad=%" PRIu32
           " bus_errors=%" PRIu32 " restarts=%" PRIu32 "\n", app.icm.samples, app.imu_gaps,
           app.icm.overflows, app.icm.bad_packets, app.icm.bus_errors, app.imu_restarts);
    printf("# load: imu+filter %.1f %% of CPU, largest batch %" PRIu32 " samples (last 10 s)\n",
           (double)(100.0f * app.fusion_load), app.batch_max);
    printf("# mag: samples=%" PRIu32 " pairs=%" PRIu32 " errors=%" PRIu32 " restarts=%" PRIu32
           ", baro: samples=%" PRIu32 " errors=%" PRIu32 "\n", app.mmc.samples, app.mmc.pairs,
           app.mmc.errors, app.mag_restarts, app.lps.samples, app.lps.errors);
    printf("# filter: watchdog recoveries=%" PRIu32 " rejected samples=%" PRIu32 " restarts=%" PRIu32 "\n",
           app.fusion.recoveries, app.fusion.rejected, app.fusion.faults);
    reply_ok();
}

static void emit_field(void *ctx, const char *name, const float *values, int count) {
    (void)ctx;
    printf("cal set %s", name);
    for (int i = 0; i < count; ++i) printf(" %.9g", (double)values[i]);
    putchar('\n');
}

static bool parse_float(const char *text, float *value) {
    char *end;
    *value = strtof(text, &end);
    return end != text && *end == '\0';
}

static void cmd_cal(int argc, char **argv) {
    if (argc < 2) {
        reply_err("usage: cal show|set|save|load|erase");
    } else if (strcmp(argv[1], "show") == 0) {
        ins10_cal_enumerate(&app.persist.cal, emit_field, NULL);
        reply_ok();
    } else if (strcmp(argv[1], "set") == 0) {
        float values[9];
        int count = argc - 3;
        if (count < 1 || count > 9) {
            reply_err("usage: cal set <name> <values>");
            return;
        }
        for (int i = 0; i < count; ++i) {
            if (!parse_float(argv[3 + i], &values[i])) {
                reply_err("not a number");
                return;
            }
        }
        if (!ins10_cal_set_field(&app.persist.cal, argv[2], values, count)) {
            reply_err("unknown field or wrong number of values");
            return;
        }
        app_apply_calibration();
        reply_ok();
    } else if (strcmp(argv[1], "save") == 0) {
        if (app_save()) reply_ok();
        else reply_err("flash write failed");
    } else if (strcmp(argv[1], "load") == 0) {
        ins10_persist stored;
        if (!storage_load(&stored)) {
            reply_err("nothing stored");
            return;
        }
        app.persist.cal = stored.cal;
        app.persist.settings = stored.settings;
        app_apply_calibration();
        reply_ok();
    } else if (strcmp(argv[1], "erase") == 0) {
        bool ok = storage_erase();
        ins10_persist_defaults(&app.persist);
        app.stored = false;
        app.thermal_saved_count = 0;
        ins10_fusion_init(&app.fusion, &app.config, &app.persist.cal, NULL);
        if (ok) reply_ok();
        else reply_err("flash erase failed");
    } else {
        reply_err("usage: cal show|set|save|load|erase");
    }
}

static void cmd_thermal(int argc, char **argv) {
    if (argc >= 2 && strcmp(argv[1], "reset") == 0) {
        /* The filter starts again from alignment with an empty model. */
        ins10_calibration cal = app.persist.cal;
        ins10_fusion_init(&app.fusion, &app.config, &cal, NULL);
        app.thermal_saved_count = 0;
        reply_ok();
        return;
    }
    const ins10_thermal_model *m = ins10_fusion_thermal_model(&app.fusion);
    printf("# observations=%" PRIu32 " range=%.1f..%.1f C\n", m->count, (double)m->t_min, (double)m->t_max);
    static const char axis[3] = {'x', 'y', 'z'};
    for (int a = 0; a < 3; ++a) {
        /* bias = c0 + c1 u + c2 u^2, u = (T - 25) / 10; printed in dps. */
        printf("# %c: c0=%.5f c1=%.5f c2=%.5f dps\n", axis[a],
               (double)(m->coef[a][0] * INS10_RAD2DEG), (double)(m->coef[a][1] * INS10_RAD2DEG),
               (double)(m->coef[a][2] * INS10_RAD2DEG));
    }
    float slope[3], sigma;
    ins10_thermal_slope(&app.fusion.therm, app.fusion.therm.temp, slope, &sigma);
    printf("# tempco now: %.5f %.5f %.5f dps/C (sigma %.5f)\n", (double)(slope[0] * INS10_RAD2DEG),
           (double)(slope[1] * INS10_RAD2DEG), (double)(slope[2] * INS10_RAD2DEG),
           (double)(sigma * INS10_RAD2DEG));
    reply_ok();
}

static bool parse_uint(const char *text, unsigned max, unsigned *value) {
    char *end;
    unsigned long v = strtoul(text, &end, 10);
    if (end == text || *end != '\0' || v > max) return false;
    *value = (unsigned)v;
    return true;
}

static void cmd_stream(int argc, char **argv) {
    unsigned value;
    if (argc == 2 && strcmp(argv[1], "off") == 0) {
        app.att_rate_hz = 0;
        app.raw_div = 0;
        app.mag_stream = app.baro_stream = false;
    } else if (argc == 3 && strcmp(argv[1], "att") == 0 && parse_uint(argv[2], 200, &value)) {
        app.att_rate_hz = (uint16_t)value;
    } else if (argc == 3 && strcmp(argv[1], "raw") == 0 && parse_uint(argv[2], 1024, &value)) {
        app.raw_div = (uint16_t)value;
    } else if (argc == 3 && strcmp(argv[1], "mag") == 0 && parse_uint(argv[2], 1, &value)) {
        app.mag_stream = value != 0;
    } else if (argc == 3 && strcmp(argv[1], "baro") == 0 && parse_uint(argv[2], 1, &value)) {
        app.baro_stream = value != 0;
    } else {
        reply_err("usage: stream att <hz>|raw <n>|mag <0|1>|baro <0|1>|off");
        return;
    }
    reply_ok();
}

static void cmd_set(int argc, char **argv) {
    unsigned value;
    if (argc == 3 && strcmp(argv[1], "mag") == 0 && parse_uint(argv[2], 1, &value)) {
        app.persist.settings.mag_enable = (uint8_t)value;
        reply_ok();
    } else {
        reply_err("usage: set mag <0|1>");
    }
}

static void execute(char *line) {
    char *argv[MAX_ARGS];
    int argc = 0;
    for (char *tok = strtok(line, " \t"); tok && argc < MAX_ARGS; tok = strtok(NULL, " \t")) {
        argv[argc++] = tok;
    }
    if (argc == 0) return;
    const char *cmd = argv[0];
    if (strcmp(cmd, "help") == 0 || strcmp(cmd, "?") == 0) {
        cmd_help();
        reply_ok();
    } else if (strcmp(cmd, "info") == 0) {
        cmd_info();
    } else if (strcmp(cmd, "status") == 0) {
        cmd_status();
    } else if (strcmp(cmd, "stream") == 0) {
        cmd_stream(argc, argv);
    } else if (strcmp(cmd, "cal") == 0) {
        cmd_cal(argc, argv);
    } else if (strcmp(cmd, "thermal") == 0) {
        cmd_thermal(argc, argv);
    } else if (strcmp(cmd, "set") == 0) {
        cmd_set(argc, argv);
    } else if (strcmp(cmd, "reset") == 0) {
        ins10_fusion_restart(&app.fusion);
        reply_ok();
    } else if (strcmp(cmd, "reboot") == 0) {
        reply_ok();
        sleep_ms(50);
        watchdog_reboot(0, 0, 10);
        for (;;) tight_loop_contents();
    } else if (strcmp(cmd, "bootsel") == 0) {
        reply_ok();
        sleep_ms(50);
        reset_usb_boot(0, 0);
    } else {
        reply_err("unknown command; try 'help'");
    }
}

void console_poll(void) {
    static char line[LINE_MAX_LEN];
    static size_t length;
    static bool overflow;
    /* A few characters per pass keeps the sensor loop responsive during a paste. */
    for (int budget = 0; budget < 32; ++budget) {
        int c = getchar_timeout_us(0);
        if (c < 0) return;
        if (c == '\r' || c == '\n') {
            if (overflow) reply_err("line too long");
            else if (length > 0) {
                line[length] = '\0';
                execute(line);
            }
            length = 0;
            overflow = false;
        } else if (length + 1 < sizeof(line)) {
            line[length++] = (char)c;
        } else {
            overflow = true;
        }
    }
}
