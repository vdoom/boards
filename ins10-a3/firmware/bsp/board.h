#ifndef INS10_BOARD_H
#define INS10_BOARD_H

/* GPIO numbers, not physical header pin numbers. See README.md for both. */
enum {
    PIN_MAG_CS = 10, PIN_BARO_CS = 11,
    PIN_MAG_INT = 12, PIN_BARO_INT = 13,
    PIN_SPI_RX = 16, PIN_IMU_CS = 17, PIN_SPI_SCK = 18, PIN_SPI_TX = 19,
    PIN_IMU_INT = 20, PIN_CLOCK_TEST = 21
};

#define INS10_SPI_BAUD 1000000u

#endif
