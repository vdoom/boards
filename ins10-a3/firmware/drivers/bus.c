#include "bus.h"
#include "board.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"

static const uint cs_pins[BUS_DEVICE_COUNT] = {PIN_IMU_CS, PIN_MAG_CS, PIN_BARO_CS};

void bus_init(void) {
    /* Chip selects high before the bus is touched: the board has no pull-ups. */
    for (unsigned i = 0; i < BUS_DEVICE_COUNT; ++i) {
        gpio_init(cs_pins[i]);
        gpio_put(cs_pins[i], true);
        gpio_set_dir(cs_pins[i], GPIO_OUT);
    }
    const uint interrupt_pins[] = {PIN_IMU_INT, PIN_MAG_INT, PIN_BARO_INT};
    for (unsigned i = 0; i < 3; ++i) {
        gpio_init(interrupt_pins[i]);
        gpio_pull_down(interrupt_pins[i]);
    }
    spi_init(spi0, INS10_SPI_BAUD);
    spi_set_format(spi0, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
    gpio_set_function(PIN_SPI_RX, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SPI_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SPI_TX, GPIO_FUNC_SPI);
}

static void select(bus_device dev) {
    if (dev == BUS_MAG) spi_set_format(spi0, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_put(cs_pins[dev], false);
    busy_wait_us_32(1);
}

static void deselect(bus_device dev) {
    busy_wait_us_32(1);
    gpio_put(cs_pins[dev], true);
    busy_wait_us_32(1);
    if (dev == BUS_MAG) spi_set_format(spi0, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
}

bool bus_read(bus_device dev, uint8_t reg, uint8_t *data, size_t length) {
    uint8_t address = reg | 0x80;
    select(dev);
    bool ok = spi_write_blocking(spi0, &address, 1) == 1 &&
              spi_read_blocking(spi0, 0, data, length) == (int)length;
    deselect(dev);
    return ok;
}

bool bus_write(bus_device dev, uint8_t reg, uint8_t value) {
    uint8_t frame[2] = {(uint8_t)(reg & 0x7f), value};
    select(dev);
    bool ok = spi_write_blocking(spi0, frame, 2) == 2;
    deselect(dev);
    return ok;
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
