/* Host tests of the sensor drivers against register-level device models.
 *
 * The drivers are compiled unchanged; only the bus and the clock are simulated
 * (device_models.c). This checks the driver logic that can be checked without the
 * board: configuration sequences, data decoding, clock-source detection, the
 * magnetometer state machine and the handling of faults. It cannot prove that the
 * models match the silicon; the first-run checklist in README.md covers that.
 */
#include "bus.h"
#include "device_models.h"
#include "icm42688.h"
#include "lps22df.h"
#include "mmc5983ma.h"
#include "pico/stdlib.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static int failures;

#define CHECK(cond) \
    do { \
        if (!(cond)) { \
            printf("    FAIL line %d: %s\n", __LINE__, #cond); \
            ++failures; \
        } \
    } while (0)

/* ---- ICM-42688-P ----------------------------------------------------------------- */

/* A recognisable pattern: every value depends on the sample index and the axis. */
static void ramp_sample(uint32_t k, int32_t gyro[3], int32_t accel[3], int16_t *temp) {
    for (int axis = 0; axis < 3; ++axis) {
        gyro[axis] = (int32_t)(k % 1000) * 37 - 20000 + axis * 111;
        accel[axis] = -(int32_t)(k % 500) * 53 + 9000 - axis * 77;
    }
    *temp = (int16_t)(k % 300) - 150;
}

static void edge_sample(uint32_t k, int32_t gyro[3], int32_t accel[3], int16_t *temp) {
    static const int32_t values[] = {0, 1, -1, 15, -16, 524287, -524287, 262144, -262145, 4660};
    int32_t v = values[k % 10];
    gyro[0] = v; gyro[1] = -v; gyro[2] = 7;
    accel[0] = 3; accel[1] = v; accel[2] = -v;
    *temp = (int16_t)(k % 2 ? -32768 : 32767);
    if (k % 10 == 9) gyro[2] = -524288; /* the sensor's "no data" marker */
}

static int drain(icm_state *st, icm_sample *out, int capacity, bool *lost, int *invalid) {
    icm_sample batch[ICM_FIFO_BATCH];
    bool l = false;
    int bad = 0, n = icm_read(st, batch, &l, &bad);
    *lost = l;
    *invalid = bad;
    for (int i = 0; i < n && i < capacity; ++i) out[i] = batch[i];
    return n;
}

static void test_icm_clock_and_rate(void) {
    icm_state st;
    models_reset();
    CHECK(icm_init(&st));
    CHECK(st.ready && st.external_clock && st.odr_hz == 512.0f);
    printf("    with CLKIN: external clock, %.3f Hz\n", (double)st.odr_hz);

    /* No oscillator: the driver must notice and fall back, measuring the real rate. */
    models_reset();
    icm_model_clkin = false;
    icm_model_internal_hz = 497.3;
    CHECK(icm_init(&st));
    CHECK(st.ready && !st.external_clock);
    CHECK(fabsf(st.odr_hz - 497.3f) < 2.0f);
    printf("    without CLKIN: internal clock, measured %.2f Hz (true 497.30)\n", (double)st.odr_hz);

    /* A clock far from anything plausible is a fault, not a rate to adopt. */
    models_reset();
    icm_model_clkin = false;
    icm_model_internal_hz = 380.0;
    CHECK(!icm_init(&st));

    models_reset();
    icm_model_present = false;
    CHECK(!icm_init(&st));
}

static void test_icm_decoding(void) {
    icm_state st;
    icm_sample s[ICM_FIFO_BATCH];
    bool lost;
    int invalid;
    models_reset();
    icm_model_sample = ramp_sample;
    CHECK(icm_init(&st));
    /* Read in the rhythm of the main loop and compare every sample with the pattern.
       The first index delivered is whatever the model had reached after init. */
    long checked = 0, first = -1, mismatches = 0;
    for (int pass = 0; pass < 2000; ++pass) {
        fake_time_us += 1000;
        int n = drain(&st, s, ICM_FIFO_BATCH, &lost, &invalid);
        CHECK(n >= 0 && !lost && invalid == 0);
        for (int i = 0; i < n; ++i) {
            int32_t g[3], a[3];
            int16_t t;
            /* Find the pattern index of the first sample, then expect a contiguous run. */
            if (first < 0) {
                for (uint32_t k = 0; k < 5000 && first < 0; ++k) {
                    ramp_sample(k, g, a, &t);
                    if (memcmp(g, s[i].gyro, sizeof(g)) == 0 && memcmp(a, s[i].accel, sizeof(a)) == 0 &&
                        t == s[i].temp) first = (long)k;
                }
                CHECK(first >= 0);
                if (first < 0) return;
            }
            ramp_sample((uint32_t)(first + checked), g, a, &t);
            if (memcmp(g, s[i].gyro, sizeof(g)) || memcmp(a, s[i].accel, sizeof(a)) || t != s[i].temp) ++mismatches;
            CHECK(!s[i].saturated);
            ++checked;
        }
    }
    printf("    %ld consecutive samples decoded, %ld mismatches\n", checked, mismatches);
    CHECK(checked > 900 && mismatches == 0);
    CHECK(st.samples == (uint32_t)checked && st.overflows == 0 && st.bad_packets == 0);

    /* Sign extension, range ends, and the invalid marker. */
    models_reset();
    icm_model_sample = edge_sample;
    CHECK(icm_init(&st));
    int seen[10] = {0}, dropped = 0, saturated = 0;
    for (int pass = 0; pass < 400; ++pass) {
        fake_time_us += 2000;
        int n = drain(&st, s, ICM_FIFO_BATCH, &lost, &invalid);
        dropped += invalid;
        for (int i = 0; i < n; ++i) {
            static const int32_t values[] = {0, 1, -1, 15, -16, 524287, -524287, 262144, -262145, 4660};
            int match = -1;
            for (int k = 0; k < 9; ++k) {
                if (s[i].gyro[0] == values[k] && s[i].gyro[1] == -values[k] && s[i].accel[1] == values[k] &&
                    s[i].accel[2] == -values[k] && s[i].gyro[2] == 7 && s[i].accel[0] == 3) match = k;
            }
            CHECK(match >= 0);
            if (match < 0) continue;
            ++seen[match];
            bool at_end = values[match] == 524287 || values[match] == -524287;
            CHECK(s[i].saturated == at_end);
            saturated += s[i].saturated;
        }
    }
    for (int k = 0; k < 9; ++k) CHECK(seen[k] > 10);
    /* The marker is dropped and counted; a reading at the negative end of the range
       (-524287) is kept and reported as saturated, not mistaken for the marker. */
    CHECK(dropped > 10 && saturated > 20);
    CHECK(seen[9] == 0);
    printf("    edge values: %d invalid packets dropped, %d saturated samples flagged\n", dropped, saturated);
    CHECK(fabsf(524287.0f * ICM_GYRO_DPS_PER_LSB - 2000.0f) < 0.01f);
    CHECK(fabsf(32768.0f * ICM_ACCEL_G_PER_LSB - 1.0f) < 1e-6f);
}

static void test_icm_overflow_and_faults(void) {
    icm_state st;
    icm_sample s[ICM_FIFO_BATCH];
    bool lost;
    int invalid;
    models_reset();
    icm_model_sample = ramp_sample;
    CHECK(icm_init(&st));
    /* The loop is away for half a second: more than the FIFO holds. */
    fake_time_us += 500000;
    int n = drain(&st, s, ICM_FIFO_BATCH, &lost, &invalid);
    CHECK(n == 0 && lost && st.overflows == 1);
    /* Afterwards the stream is clean again and arrives at the output data rate. */
    long total = 0;
    uint64_t start = fake_time_us;
    for (int pass = 0; pass < 200; ++pass) {
        fake_time_us += 2000;
        n = drain(&st, s, ICM_FIFO_BATCH, &lost, &invalid);
        CHECK(n >= 0 && !lost);
        total += n;
    }
    double rate = (double)total / ((double)(fake_time_us - start) * 1e-6);
    printf("    after an overflow: %ld samples at %.1f Hz\n", total, rate);
    CHECK(fabs(rate - 512.0) < 4.0);

    /* A backlog shorter than the FIFO is delivered completely, over several calls. */
    start = fake_time_us;
    fake_time_us += 150000;
    long backlog = 0;
    for (int pass = 0; pass < 4; ++pass) {
        n = drain(&st, s, ICM_FIFO_BATCH, &lost, &invalid);
        CHECK(!lost);
        backlog += n;
    }
    double expected = (double)(fake_time_us - start) * 1e-6 * 512.0;
    printf("    150 ms backlog: %ld samples delivered, %.1f expected\n", backlog, expected);
    CHECK(fabs((double)backlog - expected) <= 1.5);

    icm_model_bus_fault = true;
    n = drain(&st, s, ICM_FIFO_BATCH, &lost, &invalid);
    CHECK(n == -1 && st.bus_errors >= 1);
}

/* ---- MMC5983MA ------------------------------------------------------------------- */

static double field_angle; /* the board turns: the field rotates in the sensor frame */

static void set_field(void) {
    mmc_model_field_ut[0] = 20.0 * cos(field_angle) + 3.0;
    mmc_model_field_ut[1] = 20.0 * sin(field_angle);
    mmc_model_field_ut[2] = -45.0;
}

/* Runs the driver for a while, checking every delivered sample against the model. */
static void run_mmc(mmc_state *st, double seconds, double turn_rate, bool quiet, int *samples,
                    double *worst, bool expect_ok) {
    uint64_t end = fake_time_us + (uint64_t)(seconds * 1e6);
    while (fake_time_us < end) {
        fake_time_us += 100;
        field_angle += turn_rate * 100e-6;
        set_field();
        bool ok = mmc_task(st, fake_time_us, quiet);
        if (expect_ok) CHECK(ok);
        float field[3];
        uint64_t t_mid;
        if (!mmc_take(st, field, &t_mid)) continue;
        ++*samples;
        CHECK(t_mid <= fake_time_us && fake_time_us - t_mid < 20000);
        for (int axis = 0; axis < 3; ++axis) {
            double e = fabs((double)field[axis] - mmc_model_field_ut[axis]);
            if (e > *worst) *worst = e;
        }
    }
}

static void test_mmc(void) {
    mmc_state st;
    models_reset();
    field_angle = 0.4;
    set_field();
    mmc_model_offset_counts[0] = 900.0;   /* 5.5 uT of bridge offset */
    mmc_model_offset_counts[1] = -1400.0;
    mmc_model_offset_counts[2] = 300.0;
    CHECK(mmc_init(&st));

    int samples = 0;
    double worst = 0.0;
    run_mmc(&st, 5.0, 0.0, true, &samples, &worst, true);
    printf("    at rest: %d samples in 5 s, %u SET/RESET pairs, worst error %.3f uT\n", samples,
           (unsigned)st.pairs, worst);
    CHECK(samples > 220 && samples < 260);     /* about 50 Hz */
    CHECK(st.pairs >= 4 && st.pairs <= 7);     /* about one per second */
    CHECK(worst < 0.02 && st.errors == 0);

    /* The offset drifts (temperature): the pairs follow it. */
    mmc_model_offset_counts[0] += 400.0;       /* 2.4 uT */
    samples = 0;
    worst = 0.0;
    run_mmc(&st, 12.0, 0.0, true, &samples, &worst, true);
    double settled = 0.0;
    samples = 0;
    run_mmc(&st, 2.0, 0.0, true, &samples, &settled, true);
    printf("    after a 2.4 uT offset step: error %.3f uT at first, %.3f uT once tracked\n", worst, settled);
    CHECK(worst > 1.0 && settled < 0.1);

    /* Turning at 200 dps: pairs taken in motion would be wrong by the rotation between
       their two halves, so they are avoided, and the samples stay right. */
    unsigned pairs_before = st.pairs;
    samples = 0;
    worst = 0.0;
    run_mmc(&st, 5.0, 3.5, false, &samples, &worst, true);
    printf("    turning at 200 dps for 5 s: %u pairs taken, worst error %.3f uT\n",
           (unsigned)(st.pairs - pairs_before), worst);
    CHECK(st.pairs - pairs_before == 0);
    CHECK(worst < 0.1);

    /* A conversion hangs right after the RESET pulse of a pair, and at that moment the
       board starts to turn, so no new pair is due for seconds. Without care the sensor
       would be left magnetised the wrong way and every ordinary sample until the next
       pair would come out with its sign flipped. */
    run_mmc(&st, 2.0, 0.0, true, &samples, &worst, true);
    uint64_t guard = fake_time_us + 3000000;
    while (st.phase != 1 /* pair, RESET half */ && fake_time_us < guard) {
        fake_time_us += 100;
        mmc_task(&st, fake_time_us, true);
    }
    CHECK(st.phase == 1);
    mmc_model_stuck_conversions = 1;
    samples = 0;
    worst = 0.0;
    run_mmc(&st, 3.0, 0.0, false, &samples, &worst, true);
    printf("    hung conversion inside a pair: %u error(s), worst sample error afterwards %.3f uT\n",
           (unsigned)st.errors, worst);
    CHECK(st.errors == 1 && samples > 100);
    CHECK(worst < 0.1);
    CHECK(mmc_model_polarity == 1);

    /* A sensor that stops answering altogether is reported, and recovers after init. */
    mmc_model_stuck_conversions = 1000;
    bool alive = true;
    for (int i = 0; i < 20000 && alive; ++i) {
        fake_time_us += 100;
        alive = mmc_task(&st, fake_time_us, true);
    }
    CHECK(!alive && !st.ready);
    mmc_model_stuck_conversions = 0;
    CHECK(mmc_init(&st));
    samples = 0;
    worst = 0.0;
    run_mmc(&st, 1.0, 0.0, true, &samples, &worst, true);
    CHECK(samples > 40 && worst < 0.02);

    models_reset();
    mmc_model_present = false;
    CHECK(!mmc_init(&st));
}

/* ---- LPS22DF --------------------------------------------------------------------- */

static void test_lps(void) {
    lps_state st;
    models_reset();
    lps_model_hpa = 987.654;
    lps_model_temp_c = -12.34;
    CHECK(lps_init(&st));
    int samples = 0;
    float p = 0.0f, t = 0.0f;
    for (int i = 0; i < 20000; ++i) {
        fake_time_us += 100;
        CHECK(lps_task(&st, fake_time_us));
        if (lps_take(&st, &p, &t)) ++samples;
    }
    printf("    %d samples in 2 s: %.3f hPa, %.2f C, altitude %.1f m\n", samples, (double)p, (double)t,
           (double)lps_altitude_m(p));
    CHECK(samples >= 45 && samples <= 52);
    CHECK(fabsf(p - 987.654f) < 0.001f && fabsf(t + 12.34f) < 0.006f);
    CHECK(fabsf(lps_altitude_m(1013.25f)) < 0.01f);
    CHECK(fabsf(lps_altitude_m(898.75f) - 1000.0f) < 2.0f); /* standard atmosphere at 1 km */

    lps_model_silent = true;
    bool alive = true;
    for (int i = 0; i < 15000 && alive; ++i) {
        fake_time_us += 100;
        alive = lps_task(&st, fake_time_us);
    }
    CHECK(!alive);

    models_reset();
    lps_model_present = false;
    CHECK(!lps_init(&st));
}

/* ---- Registry -------------------------------------------------------------------- */

typedef struct {
    const char *name;
    void (*run)(void);
} test_case;

static const test_case tests[] = {
    {"icm_clock", test_icm_clock_and_rate},
    {"icm_decoding", test_icm_decoding},
    {"icm_faults", test_icm_overflow_and_faults},
    {"mmc", test_mmc},
    {"lps", test_lps},
};

int main(int argc, char **argv) {
    int ran = 0;
    for (size_t i = 0; i < sizeof(tests) / sizeof(tests[0]); ++i) {
        if (argc > 1 && strcmp(argv[1], tests[i].name) != 0) continue;
        int before = failures;
        printf("[ RUN  ] %s\n", tests[i].name);
        tests[i].run();
        printf("[ %s ] %s\n", failures == before ? " OK " : "FAIL", tests[i].name);
        ++ran;
    }
    if (ran == 0) {
        printf("no test named '%s'\n", argc > 1 ? argv[1] : "");
        return 2;
    }
    printf("%d test(s), %d failed check(s)\n", ran, failures);
    return failures ? 1 : 0;
}
