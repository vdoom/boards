#include "lps22df.h"
#include "bus.h"
#include "pico/stdlib.h"
#include <math.h>
#include <string.h>

enum {
    REG_IF_CTRL = 0x0e, REG_WHO_AM_I = 0x0f, REG_CTRL1 = 0x10, REG_CTRL2 = 0x11,
    REG_CTRL3 = 0x12, REG_STATUS = 0x27, REG_PRESS_OUT = 0x28, REG_TEMP_OUT = 0x2b,
    ID_VALUE = 0xb4,
};

#define POLL_PERIOD_US 10000u
#define SILENCE_US     1000000u

bool lps_init(lps_state *st) {
    uint8_t id = 0;
    memset(st, 0, sizeof(*st));
    st->ready = bus_write(BUS_BARO, REG_CTRL2, 0x04) && /* software reset */
                bus_wait_bits(BUS_BARO, REG_CTRL2, 0x04, 0, 100) &&
                bus_read(BUS_BARO, REG_WHO_AM_I, &id, 1) && id == ID_VALUE &&
                bus_update(BUS_BARO, REG_IF_CTRL, 0x60, 0x40) &&        /* SPI only, four-wire */
                bus_update(BUS_BARO, REG_CTRL2, 0x08, 0x08) &&          /* block data update */
                bus_write_checked(BUS_BARO, REG_CTRL3, 0x01, 0x0b) &&   /* address auto-increment */
                bus_write_checked(BUS_BARO, REG_CTRL1, 0x20, 0x7f);     /* 25 Hz, average of 4 */
    st->t_last = time_us_64();
    return st->ready;
}

bool lps_task(lps_state *st, uint64_t now) {
    if (!st->ready) return false;
    if (now - st->t_poll < POLL_PERIOD_US) return true;
    st->t_poll = now;
    uint8_t status, temp[2], pressure[3];
    if (!bus_read(BUS_BARO, REG_STATUS, &status, 1)) {
        ++st->errors;
    } else if ((status & 0x03) == 0x03) {
        /* With block data update the last address read must be PRESS_OUT_H, so read
           the temperature first. */
        if (bus_read(BUS_BARO, REG_TEMP_OUT, temp, sizeof(temp)) &&
            bus_read(BUS_BARO, REG_PRESS_OUT, pressure, sizeof(pressure))) {
            int32_t raw = (int32_t)pressure[0] | ((int32_t)pressure[1] << 8) | ((int32_t)pressure[2] << 16);
            if (raw & 0x800000) raw -= 0x1000000;
            st->pressure_hpa = (float)raw / 4096.0f;
            st->temp_c = (float)(int16_t)((uint16_t)temp[0] | (uint16_t)temp[1] << 8) / 100.0f;
            st->fresh = true;
            st->t_last = now;
            ++st->samples;
        } else {
            ++st->errors;
        }
    }
    if (now - st->t_last > SILENCE_US) {
        st->ready = false;
        return false;
    }
    return true;
}

bool lps_take(lps_state *st, float *pressure_hpa, float *temp_c) {
    if (!st->fresh) return false;
    st->fresh = false;
    *pressure_hpa = st->pressure_hpa;
    *temp_c = st->temp_c;
    return true;
}

float lps_altitude_m(float pressure_hpa) {
    if (!(pressure_hpa > 0.0f)) return 0.0f;
    return 44330.77f * (1.0f - powf(pressure_hpa / 1013.25f, 0.190263f));
}
