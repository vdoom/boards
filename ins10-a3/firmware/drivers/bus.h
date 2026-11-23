/* Shared SPI bus of the INS10 sensor board (SPI0 of the Pico).
 *
 * The ICM-42688-P and LPS22DF use SPI mode 3. The MMC5983MA is driven in mode 0: in
 * mode 3 its SET pulse flips only the X axis (a known issue of the chip, see 'SPI notes' in
 * ../../docs/en/datasheet.md). The format is
 * switched while every chip select is high and restored after each MMC transfer.
 *
 * All access is from the main loop; there is no locking.
 */
#ifndef INS10_BUS_H
#define INS10_BUS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum { BUS_IMU, BUS_MAG, BUS_BARO, BUS_DEVICE_COUNT } bus_device;

void bus_init(void);
/* Burst read starting at reg. No length limit; register auto-increment is the
   device's business (the ICM FIFO data register does not increment). */
bool bus_read(bus_device dev, uint8_t reg, uint8_t *data, size_t length);
bool bus_write(bus_device dev, uint8_t reg, uint8_t value);
/* Write, read back and compare under mask. */
bool bus_write_checked(bus_device dev, uint8_t reg, uint8_t value, uint8_t mask);
/* Read-modify-write of the bits in mask, verified. */
bool bus_update(bus_device dev, uint8_t reg, uint8_t mask, uint8_t value);
/* Blocking poll until (reg & mask) == value. Initialisation only. */
bool bus_wait_bits(bus_device dev, uint8_t reg, uint8_t mask, uint8_t value, uint32_t timeout_ms);

#endif
