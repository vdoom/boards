#include "device_models.h"
#include "bus.h"
#include "pico/stdlib.h"
#include <math.h>
#include <string.h>

uint64_t fake_time_us;

/* ====================================================================================
 * ICM-42688-P
 * ==================================================================================== */

bool icm_model_present, icm_model_clkin, icm_model_bus_fault;
double icm_model_internal_hz;
void (*icm_model_sample)(uint32_t k, int32_t gyro[3], int32_t accel[3], int16_t *temp);
uint32_t icm_model_produced;

#define ICM_PACKET 20
#define ICM_FIFO_PACKETS (2048 / ICM_PACKET)

static uint8_t icm_reg[2][128];
static uint8_t icm_bank;
static uint64_t icm_origin;        /* time of the last flush or (re)start */
static uint32_t icm_consumed;      /* packets read since then */
static uint32_t icm_sample_index;  /* running index over the model's life */
static bool icm_overflowed;

static void icm_defaults(void) {
    memset(icm_reg, 0, sizeof(icm_reg));
    icm_reg[0][0x75] = 0x47; /* WHO_AM_I */
    icm_reg[0][0x4c] = 0x30; /* INTF_CONFIG0 */
    icm_reg[0][0x4d] = 0x91; /* INTF_CONFIG1 */
    icm_reg[0][0x2d] = 0x10; /* INT_STATUS: RESET_DONE */
    icm_reg[0][0x4f] = 0x06;
    icm_reg[0][0x50] = 0x06;
    icm_bank = 0;
    icm_origin = fake_time_us;
    icm_consumed = icm_model_produced = 0;
    icm_overflowed = false;
}

static double icm_rate(void) {
    bool sensors_on = (icm_reg[0][0x4e] & 0x0f) == 0x0f;
    bool fifo_on = (icm_reg[0][0x16] & 0xc0) != 0;
    bool hires = (icm_reg[0][0x5f] & 0x17) == 0x17;
    if (!sensors_on || !fifo_on || !hires) return 0.0;
    if (icm_reg[0][0x4d] & 0x04) {
        /* RTC mode: the rate follows the external clock; without one, nothing useful. */
        bool pin9_clkin = (icm_reg[1][0x7b] & 0x06) == 0x04;
        return icm_model_clkin && pin9_clkin ? 512.0 : 0.0;
    }
    return icm_model_internal_hz;
}

/* Brings the FIFO up to the present. Returns packets available. */
static uint32_t icm_advance(void) {
    uint32_t total = (uint32_t)((double)(fake_time_us - icm_origin) * 1e-6 * icm_rate());
    if (total > icm_model_produced) {
        uint32_t fresh = total - icm_model_produced;
        uint32_t room = ICM_FIFO_PACKETS - (icm_model_produced - icm_consumed);
        if (fresh > room) {
            /* Stop-on-full: the excess is never stored. */
            icm_overflowed = true;
            icm_sample_index += fresh - room;
            icm_origin += (uint64_t)((double)(fresh - room) / icm_rate() * 1e6);
            fresh = room;
        }
        icm_model_produced += fresh;
    }
    return icm_model_produced - icm_consumed;
}

static void icm_flush(void) {
    icm_advance();
    icm_sample_index += icm_model_produced - icm_consumed;
    icm_origin = fake_time_us;
    icm_consumed = icm_model_produced = 0;
}

static void icm_packet(uint8_t *p) {
    int32_t gyro[3] = {0, 0, 0}, accel[3] = {0, 0, 0};
    int16_t temp = 0;
    if (icm_model_sample) icm_model_sample(icm_sample_index, gyro, accel, &temp);
    ++icm_sample_index;
    ++icm_consumed;
    p[0] = 0x78; /* accel, gyro, 20-bit, ODR timestamp */
    for (int axis = 0; axis < 3; ++axis) {
        uint32_t a = (uint32_t)accel[axis] & 0xfffff, g = (uint32_t)gyro[axis] & 0xfffff;
        p[1 + 2 * axis] = (uint8_t)(a >> 12);
        p[2 + 2 * axis] = (uint8_t)(a >> 4);
        p[7 + 2 * axis] = (uint8_t)(g >> 12);
        p[8 + 2 * axis] = (uint8_t)(g >> 4);
        p[17 + axis] = (uint8_t)((a & 0x0f) << 4 | (g & 0x0f));
    }
    p[13] = (uint8_t)((uint16_t)temp >> 8);
    p[14] = (uint8_t)temp;
    p[15] = p[16] = 0;
}

static bool icm_read(uint8_t reg, uint8_t *data, size_t length) {
    if (!icm_model_present || icm_model_bus_fault) {
        memset(data, 0xff, length); /* MISO floats high */
        return !icm_model_bus_fault;
    }
    if (icm_bank == 0 && reg == 0x30) { /* FIFO_DATA: no address increment */
        uint32_t available = icm_advance();
        for (size_t offset = 0; offset + ICM_PACKET <= length; offset += ICM_PACKET) {
            if (available == 0) {
                memset(data + offset, 0xff, ICM_PACKET); /* header says "FIFO empty" */
                continue;
            }
            icm_packet(data + offset);
            --available;
        }
        return true;
    }
    for (size_t i = 0; i < length; ++i) {
        uint8_t address = (uint8_t)((reg + i) & 0x7f);
        if (icm_bank == 0 && (address == 0x2e || address == 0x2f)) {
            uint32_t bytes = icm_advance() * ICM_PACKET;
            data[i] = address == 0x2e ? (uint8_t)(bytes >> 8) : (uint8_t)bytes;
        } else if (address == 0x76) {
            data[i] = icm_bank;
        } else {
            data[i] = icm_reg[icm_bank & 1][address];
            if (icm_bank == 0 && address == 0x2d) icm_reg[0][0x2d] = 0; /* clear on read */
        }
    }
    return true;
}

static bool icm_write(uint8_t reg, uint8_t value) {
    if (icm_model_bus_fault) return false;
    if (!icm_model_present) return true;
    if (reg == 0x76) {
        icm_bank = value & 7;
        return true;
    }
    if (icm_bank == 0 && reg == 0x11 && (value & 1)) {
        icm_defaults();
        return true;
    }
    if (icm_bank == 0 && reg == 0x4b) {
        if (value & 0x02) icm_flush();
        return true;
    }
    if (icm_bank == 0 && (reg == 0x4e || reg == 0x4d || reg == 0x16 || reg == 0x5f)) {
        /* Anything that changes the data flow restarts the FIFO time base. */
        icm_flush();
        icm_reg[0][reg] = value;
        icm_origin = fake_time_us;
        return true;
    }
    icm_reg[icm_bank & 1][reg & 0x7f] = value;
    return true;
}

/* ====================================================================================
 * MMC5983MA
 * ==================================================================================== */

bool mmc_model_present, mmc_model_stale_done;
double mmc_model_field_ut[3], mmc_model_offset_counts[3];
int mmc_model_stuck_conversions, mmc_model_polarity;

static uint8_t mmc_data[7];
static bool mmc_done, mmc_converting, mmc_stuck;
static uint64_t mmc_done_at, mmc_stale_until;

static void mmc_finish(void) {
    if (!mmc_converting || mmc_stuck || fake_time_us < mmc_done_at) return;
    int32_t raw[3];
    for (int axis = 0; axis < 3; ++axis) {
        double counts = 131072.0 + mmc_model_polarity * mmc_model_field_ut[axis] * 163.84 +
                        mmc_model_offset_counts[axis];
        raw[axis] = (int32_t)lround(counts);
        if (raw[axis] < 0) raw[axis] = 0;
        if (raw[axis] > 0x3ffff) raw[axis] = 0x3ffff;
        mmc_data[2 * axis] = (uint8_t)(raw[axis] >> 10);
        mmc_data[2 * axis + 1] = (uint8_t)(raw[axis] >> 2);
    }
    mmc_data[6] = (uint8_t)((raw[0] & 3) << 6 | (raw[1] & 3) << 4 | (raw[2] & 3) << 2);
    mmc_converting = false;
    mmc_done = true;
}

static bool mmc_read(uint8_t reg, uint8_t *data, size_t length) {
    if (!mmc_model_present) {
        memset(data, 0xff, length);
        return true;
    }
    mmc_finish();
    for (size_t i = 0; i < length; ++i) {
        uint8_t address = (uint8_t)(reg + i);
        if (address <= 6) data[i] = mmc_data[address];
        else if (address == 0x08) {
            bool stale = fake_time_us < mmc_stale_until;
            data[i] = (uint8_t)(0x10 | (mmc_done || stale ? 0x01 : 0)); /* OTP read done */
        } else if (address == 0x2f) data[i] = 0x30;
        else data[i] = 0; /* control registers are write-only */
    }
    return true;
}

static bool mmc_write(uint8_t reg, uint8_t value) {
    if (!mmc_model_present) return true;
    mmc_finish();
    if (reg == 0x08) {
        if (value & 0x01) mmc_done = false;
    } else if (reg == 0x09) {
        if (value & 0x08) mmc_model_polarity = 1;
        if (value & 0x10) mmc_model_polarity = -1;
        if (value & 0x01) {
            /* A new measurement clears DONE; on the real part the old flag can linger. */
            if (mmc_done && mmc_model_stale_done) mmc_stale_until = fake_time_us + 300;
            mmc_done = false;
            mmc_converting = true;
            mmc_stuck = mmc_model_stuck_conversions > 0;
            if (mmc_stuck) --mmc_model_stuck_conversions;
            mmc_done_at = fake_time_us + 8000;
        }
    } else if (reg == 0x0a && (value & 0x80)) {
        mmc_done = mmc_converting = false;
        mmc_model_polarity = 1;
    }
    return true;
}

/* ====================================================================================
 * LPS22DF
 * ==================================================================================== */

bool lps_model_present, lps_model_silent;
double lps_model_hpa, lps_model_temp_c;

static uint8_t lps_reg[128];
static uint64_t lps_next;

static bool lps_read(uint8_t reg, uint8_t *data, size_t length) {
    if (!lps_model_present) {
        memset(data, 0xff, length);
        return true;
    }
    bool running = (lps_reg[0x10] & 0x78) != 0 && !lps_model_silent;
    for (size_t i = 0; i < length; ++i) {
        uint8_t address = (uint8_t)((reg + i) & 0x7f);
        int32_t pressure = (int32_t)lround(lps_model_hpa * 4096.0);
        int16_t temp = (int16_t)lround(lps_model_temp_c * 100.0);
        switch (address) {
        case 0x0f: data[i] = 0xb4; break;
        case 0x27: data[i] = running && fake_time_us >= lps_next ? 0x03 : 0x00; break;
        case 0x28: data[i] = (uint8_t)pressure; break;
        case 0x29: data[i] = (uint8_t)(pressure >> 8); break;
        case 0x2a:
            data[i] = (uint8_t)(pressure >> 16);
            if (fake_time_us >= lps_next) lps_next = fake_time_us + 40000; /* 25 Hz */
            break;
        case 0x2b: data[i] = (uint8_t)temp; break;
        case 0x2c: data[i] = (uint8_t)((uint16_t)temp >> 8); break;
        default: data[i] = lps_reg[address]; break;
        }
    }
    return true;
}

static bool lps_write(uint8_t reg, uint8_t value) {
    if (!lps_model_present) return true;
    if (reg == 0x11 && (value & 0x04)) {
        memset(lps_reg, 0, sizeof(lps_reg)); /* software reset, self-clearing */
        lps_next = fake_time_us;
        return true;
    }
    lps_reg[reg & 0x7f] = value;
    return true;
}

/* ====================================================================================
 * Bus interface (drivers/bus.h) on top of the models
 * ==================================================================================== */

void models_reset(void) {
    fake_time_us = 1000000;
    icm_model_present = mmc_model_present = lps_model_present = true;
    icm_model_clkin = true;
    icm_model_internal_hz = 500.0;
    icm_model_bus_fault = false;
    icm_model_sample = NULL;
    icm_sample_index = 0;
    icm_defaults();
    memset(mmc_model_field_ut, 0, sizeof(mmc_model_field_ut));
    memset(mmc_model_offset_counts, 0, sizeof(mmc_model_offset_counts));
    mmc_model_stuck_conversions = 0;
    mmc_model_stale_done = true;
    mmc_model_polarity = 1;
    mmc_done = mmc_converting = false;
    mmc_stale_until = 0;
    lps_model_hpa = 1013.25;
    lps_model_temp_c = 25.0;
    lps_model_silent = false;
    memset(lps_reg, 0, sizeof(lps_reg));
    lps_next = 0;
}

void bus_init(void) {}

bool bus_read(bus_device dev, uint8_t reg, uint8_t *data, size_t length) {
    fake_time_us += 20 + 8 * (uint64_t)length; /* 1 MHz SPI */
    switch (dev) {
    case BUS_IMU: return icm_read(reg, data, length);
    case BUS_MAG: return mmc_read(reg, data, length);
    default: return lps_read(reg, data, length);
    }
}

bool bus_write(bus_device dev, uint8_t reg, uint8_t value) {
    fake_time_us += 36;
    switch (dev) {
    case BUS_IMU: return icm_write(reg, value);
    case BUS_MAG: return mmc_write(reg, value);
    default: return lps_write(reg, value);
    }
}

bool bus_write_checked(bus_device dev, uint8_t reg, uint8_t value, uint8_t mask) {
    uint8_t actual;
    return bus_write(dev, reg, value) && bus_read(dev, reg, &actual, 1) &&
           (actual & mask) == (value & mask);
}

bool bus_update(bus_device dev, uint8_t reg, uint8_t mask, uint8_t value) {
    uint8_t old;
    return bus_read(dev, reg, &old, 1) &&
           bus_write_checked(dev, reg, (uint8_t)((old & ~mask) | (value & mask)), mask);
}

bool bus_wait_bits(bus_device dev, uint8_t reg, uint8_t mask, uint8_t value, uint32_t timeout_ms) {
    absolute_time_t deadline = make_timeout_time_ms(timeout_ms);
    do {
        uint8_t actual;
        if (!bus_read(dev, reg, &actual, 1)) return false;
        if ((actual & mask) == value) return true;
        busy_wait_us_32(100);
    } while (!time_reached(deadline));
    return false;
}
