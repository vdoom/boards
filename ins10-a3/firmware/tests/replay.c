/* Replays a raw sensor log through the fusion library on the host.
 *
 *   ins10_replay <log> [calibration] [--rate 512] > attitude.csv
 *
 * <log> is what the firmware prints with "stream raw 1" and "stream mag 1" (for example
 * a file written by "ins10tool log"). [calibration] is a saved "cal show" listing; "cal
 * set" lines inside the log itself are applied too. Useful for tuning the filter
 * against real recordings without reflashing.
 */
#include "ins10/ins10.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Scales of the ICM-42688-P high-resolution FIFO format. */
#define GYRO_LSB_PER_DPS (524288.0 / 2000.0)
#define ACCEL_LSB_PER_G  (524288.0 / 16.0)
#define TEMP_LSB_PER_C   132.48

static int parse_floats(char *text, float *values, int max) {
    int n = 0;
    for (char *tok = strtok(text, " ,\t\r\n"); tok && n < max; tok = strtok(NULL, " ,\t\r\n")) {
        values[n++] = (float)atof(tok);
    }
    return n;
}

static void apply_cal_line(ins10_calibration *cal, char *line) {
    /* "cal set <name> <values...>" */
    char name[32];
    int used = 0;
    if (sscanf(line, "cal set %31s %n", name, &used) != 1) return;
    float values[9];
    int n = parse_floats(line + used, values, 9);
    if (!ins10_cal_set_field(cal, name, values, n)) {
        fprintf(stderr, "ignored calibration line: %s\n", name);
    }
}

int main(int argc, char **argv) {
    const char *log_path = NULL, *cal_path = NULL;
    double rate = 512.0;
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--rate") == 0 && i + 1 < argc) rate = atof(argv[++i]);
        else if (!log_path) log_path = argv[i];
        else if (!cal_path) cal_path = argv[i];
    }
    if (!log_path || rate <= 0.0) {
        fprintf(stderr, "usage: ins10_replay <log> [calibration] [--rate hz]\n");
        return 2;
    }
    ins10_calibration cal;
    ins10_cal_defaults(&cal);
    char line[512];
    if (cal_path) {
        FILE *cf = fopen(cal_path, "r");
        if (!cf) {
            fprintf(stderr, "cannot open %s\n", cal_path);
            return 1;
        }
        while (fgets(line, sizeof(line), cf)) apply_cal_line(&cal, line);
        fclose(cf);
    }
    FILE *in = fopen(log_path, "r");
    if (!in) {
        fprintf(stderr, "cannot open %s\n", log_path);
        return 1;
    }
    ins10_fusion_config cfg;
    ins10_fusion_default_config(&cfg);
    cfg.sample_rate_hz = (float)rate;
    static ins10_fusion fusion;
    ins10_fusion_init(&fusion, &cfg, &cal, NULL);

    printf("t_s,qw,qx,qy,qz,roll_deg,pitch_deg,yaw_deg,heading_deg,"
           "bias_x_dps,bias_y_dps,bias_z_dps,tilt_sigma_deg,heading_sigma_deg,flags\n");
    double t = 0.0;
    long expected_seq = -1, lost = 0, samples = 0;
    while (fgets(line, sizeof(line), in)) {
        if (strncmp(line, "cal set ", 8) == 0) {
            apply_cal_line(&cal, line);
            ins10_fusion_set_calibration(&fusion, &cal);
        } else if (strncmp(line, "$RAW,", 5) == 0) {
            long seq, n, v[7];
            if (sscanf(line + 5, "%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld", &seq, &n,
                       &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6]) != 9 || n <= 0) continue;
            if (expected_seq >= 0 && seq != expected_seq) lost += seq - expected_seq;
            expected_seq = seq + n;
            float gyro[3], accel[3];
            for (int i = 0; i < 3; ++i) {
                gyro[i] = (float)((double)v[i] / (double)n / GYRO_LSB_PER_DPS);
                accel[i] = (float)((double)v[i + 3] / (double)n / ACCEL_LSB_PER_G);
            }
            float temp = (float)((double)v[6] / (double)n / TEMP_LSB_PER_C + 25.0);
            float dt = (float)((double)n / rate);
            ins10_fusion_imu(&fusion, gyro, accel, temp, dt, false);
            t += (double)dt;
            ++samples;
            ins10_output o;
            ins10_fusion_get(&fusion, &o);
            if (!(o.flags & INS10_F_ALIGNED)) continue;
            printf("%.4f,%.6f,%.6f,%.6f,%.6f,%.3f,%.3f,%.3f,%.3f,%.5f,%.5f,%.5f,%.3f,%.3f,0x%03X\n",
                   t, (double)o.q[0], (double)o.q[1], (double)o.q[2], (double)o.q[3],
                   (double)o.roll_deg, (double)o.pitch_deg, (double)o.yaw_deg,
                   (double)o.heading_deg, (double)o.gyro_bias_dps[0],
                   (double)o.gyro_bias_dps[1], (double)o.gyro_bias_dps[2],
                   (double)o.tilt_sigma_deg, (double)o.heading_sigma_deg, (unsigned)o.flags);
        } else if (strncmp(line, "$MAG,", 5) == 0) {
            float v[4];
            if (parse_floats(line + 5, v, 4) == 4) ins10_fusion_mag(&fusion, v + 1, 0.0f);
        }
    }
    fclose(in);
    fprintf(stderr, "%ld IMU records, %.1f s, %ld samples lost in the log\n", samples, t, lost);
    return 0;
}
