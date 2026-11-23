#include "icm42688.h"
#include "bus.h"
#include "pico/stdlib.h"
#include <string.h>

/* Register map, TDK DS-000347. Bank 0 unless noted. */
enum {
    REG_DEVICE_CONFIG = 0x11,
    REG_FIFO_CONFIG = 0x16,
    REG_INT_STATUS = 0x2d,
    REG_FIFO_COUNTH = 0x2e,
    REG_FIFO_DATA = 0x30,
    REG_SIGNAL_PATH_RESET = 0x4b,
    REG_INTF_CONFIG0 = 0x4c,
    REG_INTF_CONFIG1 = 0x4d,
    REG_PWR_MGMT0 = 0x4e,
    REG_GYRO_CONFIG0 = 0x4f,
    REG_ACCEL_CONFIG0 = 0x50,
    REG_FIFO_CONFIG1 = 0x5f,
    REG_WHO_AM_I = 0x75,
    REG_BANK_SEL = 0x76,
    REG_B1_INTF_CONFIG5 = 0x7b,

    WHO_AM_I_VALUE = 0x47,
    RESET_DONE = 0x10,
    FIFO_FLUSH = 0x02,
    FIFO_MODE_STOP_ON_FULL = 0xc0,
    /* Big-endian count and data, I2C disabled; the count is in bytes. */
    INTF0_VALUE = 0x33,
    INTF0_MASK = 0x73,
    RTC_MODE = 0x04,
    CLKSEL_PLL = 0x01,
    PIN9_CLKIN = 0x04,
    /* Partial reads resume, 20-bit packets, temperature, gyro, accel. */
    FIFO1_VALUE = 0x40 | 0x10 | 0x04 | 0x02 | 0x01,
    ODR_500HZ = 0x0f, /* with FS_SEL = 0: +/-2000 dps, +/-16 g */

    PACKET_SIZE = 20,
    FIFO_BYTES = 2048,
    HEADER_EMPTY = 0x80,
    HEADER_EXPECTED = 0x70, /* accel + gyro + 20-bit */
};

/* End-of-range detection, in 20-bit counts: 1990 dps and 15.9 g. */
#define GYRO_LIMIT  521666
#define ACCEL_LIMIT 521011
/* The sensor's "no data" marker: the most negative 20-bit value. A real reading at
   the negative end of the range stops one count short of it, so the marker has to be
   recognised on all 20 bits; on the upper 16 alone a saturated axis would be thrown
   away instead of being reported as saturated. */
#define INVALID_20BIT (-524288)

static bool flush_fifo(void) {
    return bus_write(BUS_IMU, REG_SIGNAL_PATH_RESET, FIFO_FLUSH);
}

static bool start(bool external_clock) {
    /* Sensors off while the clock source changes, as in the bench firmware. */
    if (!bus_write(BUS_IMU, REG_PWR_MGMT0, 0)) return false;
    sleep_ms(10);
    if (!bus_update(BUS_IMU, REG_INTF_CONFIG1, RTC_MODE | 0x03,
                    (uint8_t)((external_clock ? RTC_MODE : 0) | CLKSEL_PLL)) ||
        !bus_write(BUS_IMU, REG_PWR_MGMT0, 0x0f)) return false; /* gyro and accel low-noise */
    sleep_ms(100); /* gyro start-up, and no register writes right after power-on */
    uint8_t power;
    return bus_read(BUS_IMU, REG_PWR_MGMT0, &power, 1) && (power & 0x0f) == 0x0f &&
           flush_fifo();
}

static int fifo_records(void) {
    uint8_t raw[2];
    if (!bus_read(BUS_IMU, REG_FIFO_COUNTH, raw, 2)) return -1;
    return (int)(((unsigned)raw[0] << 8 | raw[1]) / PACKET_SIZE);
}

/* Output data rate measured against the Pico's clock by counting FIFO records. */
static bool measure_rate(float *hz) {
    static uint8_t scratch[ICM_FIFO_BATCH * PACKET_SIZE];
    uint64_t first = 0, last = 0;
    unsigned counted = 0;
    bool started = false;
    if (!flush_fifo()) return false;
    absolute_time_t deadline = make_timeout_time_ms(450);
    while (!time_reached(deadline)) {
        int n = fifo_records();
        if (n < 0) return false;
        if (n == 0) {
            busy_wait_us_32(200);
            continue;
        }
        if (n > ICM_FIFO_BATCH) n = ICM_FIFO_BATCH;
        uint64_t now = time_us_64();
        if (!bus_read(BUS_IMU, REG_FIFO_DATA, scratch, (size_t)n * PACKET_SIZE)) return false;
        if (!started) {
            /* The first batch only marks the starting edge. */
            started = true;
            first = now;
        } else {
            counted += (unsigned)n;
            last = now;
        }
    }
    if (counted < 100 || last <= first) return false;
    *hz = 1e6f * (float)counted / (float)(last - first);
    return true;
}

bool icm_init(icm_state *st) {
    uint8_t id = 0;
    memset(st, 0, sizeof(*st));
    if (!bus_write(BUS_IMU, REG_BANK_SEL, 0) || !bus_write(BUS_IMU, REG_DEVICE_CONFIG, 1)) return false;
    sleep_ms(10);
    if (!bus_wait_bits(BUS_IMU, REG_INT_STATUS, RESET_DONE, RESET_DONE, 100) ||
        !bus_read(BUS_IMU, REG_WHO_AM_I, &id, 1) || id != WHO_AM_I_VALUE) return false;

    /* Pin 9 is wired to the 32.768 kHz oscillator: make it CLKIN, never INT2. */
    if (!bus_write(BUS_IMU, REG_BANK_SEL, 1)) return false;
    bool clkin = bus_update(BUS_IMU, REG_B1_INTF_CONFIG5, 0x06, PIN9_CLKIN);
    if (!bus_write(BUS_IMU, REG_BANK_SEL, 0) || !clkin) return false;

    if (!bus_update(BUS_IMU, REG_INTF_CONFIG0, INTF0_MASK, INTF0_VALUE) ||
        !bus_write_checked(BUS_IMU, REG_GYRO_CONFIG0, ODR_500HZ, 0xef) ||
        !bus_write_checked(BUS_IMU, REG_ACCEL_CONFIG0, ODR_500HZ, 0xef) ||
        !bus_write_checked(BUS_IMU, REG_FIFO_CONFIG1, FIFO1_VALUE, 0x7f) ||
        !bus_write_checked(BUS_IMU, REG_FIFO_CONFIG, FIFO_MODE_STOP_ON_FULL, 0xc0)) return false;

    /* Prefer the external clock: 500 Hz nominal becomes exactly 512 Hz. Accept it
       only if the rate confirms that the oscillator is really driving the PLL. */
    float hz = 0.0f;
    if (start(true) && measure_rate(&hz) && hz > 509.0f && hz < 515.0f) {
        st->external_clock = true;
        st->odr_hz = 512.0f;
    } else if (start(false) && measure_rate(&hz) && hz > 460.0f && hz < 540.0f) {
        st->external_clock = false;
        st->odr_hz = hz;
    } else {
        return false;
    }
    st->ready = flush_fifo();
    return st->ready;
}

static int32_t extend20(uint8_t high, uint8_t low, uint8_t nibble) {
    int32_t word = (int16_t)((uint16_t)high << 8 | low);
    return word * 16 + nibble;
}

int icm_read(icm_state *st, icm_sample *out, bool *lost, int *invalid) {
    static uint8_t buffer[ICM_FIFO_BATCH * PACKET_SIZE];
    *lost = false;
    *invalid = 0;
    uint8_t raw[2];
    if (!bus_read(BUS_IMU, REG_FIFO_COUNTH, raw, 2)) {
        ++st->bus_errors;
        return -1;
    }
    unsigned bytes = (unsigned)raw[0] << 8 | raw[1];
    if (bytes >= FIFO_BYTES - PACKET_SIZE) {
        /* Stop-on-full: the FIFO stopped taking samples. Start afresh. */
        ++st->overflows;
        *lost = true;
        if (!flush_fifo()) ++st->bus_errors;
        return 0;
    }
    int n = (int)(bytes / PACKET_SIZE);
    if (n == 0) return 0;
    if (n > ICM_FIFO_BATCH) n = ICM_FIFO_BATCH;
    if (!bus_read(BUS_IMU, REG_FIFO_DATA, buffer, (size_t)n * PACKET_SIZE)) {
        ++st->bus_errors;
        return -1;
    }
    int count = 0;
    for (int i = 0; i < n; ++i) {
        const uint8_t *p = buffer + i * PACKET_SIZE;
        if (p[0] & HEADER_EMPTY) break; /* read past the last record */
        if ((p[0] & HEADER_EXPECTED) != HEADER_EXPECTED) {
            /* Not the packet format that was configured: the stream is out of step. */
            ++st->bad_packets;
            *lost = true;
            if (!flush_fifo()) ++st->bus_errors;
            break;
        }
        icm_sample *s = &out[count];
        bool missing = false;
        s->saturated = false;
        for (int axis = 0; axis < 3; ++axis) {
            uint8_t ext = p[17 + axis]; /* accel low nibble in bits 7:4, gyro in 3:0 */
            s->accel[axis] = extend20(p[1 + 2 * axis], p[2 + 2 * axis], ext >> 4);
            s->gyro[axis] = extend20(p[7 + 2 * axis], p[8 + 2 * axis], ext & 0x0f);
            missing |= s->accel[axis] == INVALID_20BIT || s->gyro[axis] == INVALID_20BIT;
            s->saturated |= s->gyro[axis] > GYRO_LIMIT || s->gyro[axis] < -GYRO_LIMIT ||
                            s->accel[axis] > ACCEL_LIMIT || s->accel[axis] < -ACCEL_LIMIT;
        }
        s->temp = (int16_t)((uint16_t)p[13] << 8 | p[14]);
        if (missing) {
            /* One sample period without data; the caller tells the filter. */
            ++st->bad_packets;
            ++*invalid;
            continue;
        }
        ++count;
    }
    st->samples += (uint32_t)count;
    return count;
}
