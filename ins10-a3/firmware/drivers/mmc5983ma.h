/* MMC5983MA driver: non-blocking single-shot measurements with SET/RESET offset
 * tracking.
 *
 * The sensor's bridge offset drifts with temperature and jumps after exposure to a
 * strong field. A SET pulse followed by a measurement gives  offset + H, a RESET pulse
 * gives  offset - H,  so a SET/RESET pair yields the offset with the field cancelled.
 * The driver measures in the SET state at about 50 Hz and inserts a RESET measurement
 * about once a second to refresh the offset. This is the magnetometer's temperature
 * compensation: the offset is re-measured, not modelled.
 *
 * A pair taken while the board rotates sees a different field in each half, which
 * would corrupt the offset; the caller therefore says when the board is turning
 * slowly enough, and pairs taken in motion are used only when overdue and with a
 * small weight.
 *
 * Bus sequences are the ones verified on the INS10-A3 by the board's bench test firmware
 * (SPI mode 0, single-shot, DONE flag handling).
 */
#ifndef INS10_MMC5983MA_H
#define INS10_MMC5983MA_H

#include <stdbool.h>
#include <stdint.h>

#define MMC_UT_PER_LSB (100.0f / 16384.0f) /* 18-bit output, 16384 counts per gauss */

typedef struct {
    bool ready;
    uint8_t state, phase;
    bool wait_clear;
    uint64_t t_state, t_start, t_poll, next_sample, last_pair;
    int32_t reset_raw[3];
    float offset[3]; /* counts */
    bool offset_valid;
    bool polarity_unknown; /* a cycle failed part-way: the sensor may be left in RESET */
    uint8_t consecutive_errors;
    /* Latest result. */
    bool fresh;
    float field_ut[3];
    uint64_t t_mid;
    /* Statistics. */
    uint32_t samples, pairs, errors;
} mmc_state;

bool mmc_init(mmc_state *st);
/* Advance the measurement state machine; call often. quiet: the board is rotating
   slowly enough for a clean SET/RESET pair. Returns false after repeated faults; the
   caller should then call mmc_init again. */
bool mmc_task(mmc_state *st, uint64_t now_us, bool quiet);
/* Fetch a new sample, uncalibrated, in uT, with the mid-time of its conversion. */
bool mmc_take(mmc_state *st, float field_ut[3], uint64_t *t_mid_us);

#endif
