#include "mmc5983ma.h"
#include "bus.h"
#include "pico/stdlib.h"
#include <string.h>

enum {
    REG_DATA = 0x00, REG_STATUS = 0x08, REG_CTRL0 = 0x09, REG_CTRL1 = 0x0a,
    REG_CTRL2 = 0x0b, REG_CTRL3 = 0x0c, REG_ID = 0x2f,
    ID_VALUE = 0x30,
    MEAS_DONE = 0x01,
    CMD_MEASURE = 0x01, CMD_INT_ENABLE = 0x04, CMD_SET = 0x08, CMD_RESET = 0x10,
    SW_RESET = 0x80,
    MID_SCALE = 131072, FULL_SCALE = 0x3ffff,
};

enum { ST_IDLE, ST_PULSE_WAIT, ST_CONVERTING };
enum { PH_NORMAL, PH_PAIR_RESET, PH_PAIR_SET };

#define SAMPLE_PERIOD_US 20000u  /* about 50 Hz; a conversion takes 8 ms at BW = 00 */
#define PAIR_PERIOD_US   1000000u
#define PAIR_OVERDUE_US  10000000u
#define PULSE_WAIT_US    1000u
#define POLL_PERIOD_US   250u
#define CLEAR_TIMEOUT_US 20000u
#define DONE_TIMEOUT_US  100000u
#define MAX_ERRORS       5

bool mmc_init(mmc_state *st) {
    uint8_t id = 0;
    memset(st, 0, sizeof(*st));
    if (!bus_write(BUS_MAG, REG_CTRL1, SW_RESET)) return false;
    sleep_ms(20);
    for (unsigned i = 0; i < 4; ++i) {
        if (!bus_read(BUS_MAG, REG_ID, &id, 1) || id != ID_VALUE) return false;
    }
    /* The control registers are write-only; behaviour is validated by conversions. */
    if (!bus_write(BUS_MAG, REG_CTRL1, 0) || /* BW = 00: 8 ms, lowest noise */
        !bus_write(BUS_MAG, REG_CTRL2, 0) || /* single-shot */
        !bus_write(BUS_MAG, REG_CTRL3, 0)) return false;
    st->ready = true;
    st->state = ST_IDLE;
    st->next_sample = time_us_64();
    return true;
}

static bool fault(mmc_state *st, uint64_t now) {
    ++st->errors;
    /* A cycle that failed between its RESET and SET pulses leaves the sensor
       magnetised the other way round, and an ordinary measurement would then come out
       with the opposite sign. Make the next cycle a full RESET/SET pair, which ends
       in a known state. */
    st->polarity_unknown = true;
    st->state = ST_IDLE;
    st->next_sample = now + SAMPLE_PERIOD_US;
    if (++st->consecutive_errors < MAX_ERRORS) return true;
    st->ready = false;
    return false;
}

static bool pulse(mmc_state *st, uint8_t command, uint64_t now) {
    if (!bus_write(BUS_MAG, REG_CTRL0, command)) return fault(st, now);
    st->state = ST_PULSE_WAIT;
    st->t_state = now;
    return true;
}

static bool start_conversion(mmc_state *st, uint64_t now) {
    uint8_t previous;
    /* Clearing the interrupt can leave the previous DONE flag set for a moment after
       the new command; remember it so that a stale flag is not taken for the result. */
    if (!bus_write(BUS_MAG, REG_STATUS, MEAS_DONE) ||
        !bus_read(BUS_MAG, REG_STATUS, &previous, 1) ||
        !bus_write(BUS_MAG, REG_CTRL0, CMD_MEASURE | CMD_INT_ENABLE)) return fault(st, now);
    st->wait_clear = (previous & MEAS_DONE) != 0;
    st->state = ST_CONVERTING;
    st->t_start = st->t_poll = now;
    return true;
}

static void unpack(const uint8_t data[7], int32_t raw[3]) {
    for (unsigned axis = 0; axis < 3; ++axis) {
        raw[axis] = ((int32_t)data[2 * axis] << 10) | ((int32_t)data[2 * axis + 1] << 2) |
                    ((data[6] >> (6 - 2 * axis)) & 3);
    }
}

static bool finish_conversion(mmc_state *st, uint64_t now, bool quiet) {
    uint8_t data[7];
    int32_t raw[3];
    if (!bus_read(BUS_MAG, REG_DATA, data, sizeof(data)) ||
        !bus_write(BUS_MAG, REG_STATUS, MEAS_DONE)) return fault(st, now);
    unpack(data, raw);
    for (unsigned axis = 0; axis < 3; ++axis) {
        if (raw[axis] == 0 || raw[axis] == FULL_SCALE) return fault(st, now); /* railed */
    }
    if (st->phase == PH_PAIR_RESET) {
        memcpy(st->reset_raw, raw, sizeof(raw));
        st->phase = PH_PAIR_SET;
        return pulse(st, CMD_SET, now);
    }
    if (st->phase == PH_PAIR_SET) {
        /* offset = (SET + RESET) / 2. The first pair is taken as is; later ones are
           blended, lightly if the board was moving. */
        float weight = !st->offset_valid ? 1.0f : (quiet ? 0.3f : 0.1f);
        for (unsigned axis = 0; axis < 3; ++axis) {
            float offset = 0.5f * (float)(raw[axis] + st->reset_raw[axis]) - (float)MID_SCALE;
            st->offset[axis] += weight * (offset - st->offset[axis]);
        }
        st->offset_valid = true;
        st->polarity_unknown = false;
        st->last_pair = now;
        ++st->pairs;
    }
    for (unsigned axis = 0; axis < 3; ++axis) {
        st->field_ut[axis] = ((float)(raw[axis] - MID_SCALE) - st->offset[axis]) * MMC_UT_PER_LSB;
    }
    st->t_mid = st->t_start + (now - st->t_start) / 2;
    st->fresh = true;
    ++st->samples;
    st->consecutive_errors = 0;
    st->phase = PH_NORMAL;
    st->state = ST_IDLE;
    st->next_sample += SAMPLE_PERIOD_US;
    if (st->next_sample < now) st->next_sample = now; /* a pair took two periods */
    return true;
}

bool mmc_task(mmc_state *st, uint64_t now, bool quiet) {
    if (!st->ready) return false;
    switch (st->state) {
    case ST_IDLE: {
        if (now < st->next_sample) return true;
        uint64_t since_pair = now - st->last_pair;
        bool pair = !st->offset_valid || st->polarity_unknown ||
                    (quiet && since_pair > PAIR_PERIOD_US) ||
                    since_pair > PAIR_OVERDUE_US;
        if (pair) {
            st->phase = PH_PAIR_RESET;
            return pulse(st, CMD_RESET, now);
        }
        st->phase = PH_NORMAL;
        return start_conversion(st, now);
    }
    case ST_PULSE_WAIT:
        if (now - st->t_state < PULSE_WAIT_US) return true;
        return start_conversion(st, now);
    case ST_CONVERTING: {
        if (now - st->t_poll < POLL_PERIOD_US) return true;
        st->t_poll = now;
        uint8_t status;
        if (!bus_read(BUS_MAG, REG_STATUS, &status, 1)) return fault(st, now);
        bool done = (status & MEAS_DONE) != 0;
        if (st->wait_clear) {
            if (!done) st->wait_clear = false;
            else if (now - st->t_start > CLEAR_TIMEOUT_US) return fault(st, now);
            return true;
        }
        if (done) return finish_conversion(st, now, quiet);
        if (now - st->t_start > DONE_TIMEOUT_US) return fault(st, now);
        return true;
    }
    default:
        st->state = ST_IDLE;
        return true;
    }
}

bool mmc_take(mmc_state *st, float field_ut[3], uint64_t *t_mid_us) {
    if (!st->fresh) return false;
    st->fresh = false;
    memcpy(field_ut, st->field_ut, sizeof(st->field_ut));
    *t_mid_us = st->t_mid;
    return true;
}
