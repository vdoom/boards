/* Persistent storage in the last two flash sectors.
 *
 * Two slots are written alternately, each record carrying a sequence number and a
 * CRC. A power loss during a write can only damage the slot being written; the other
 * still holds the previous valid record.
 */
#include "app.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/flash.h"
#include "pico/stdlib.h"
#include <string.h>

#define SLOT_COUNT  2u
#define RECORD_AREA 1024u /* bytes programmed per slot; a multiple of the 256-byte page */

_Static_assert(INS10_PERSIST_RECORD_SIZE <= RECORD_AREA, "persistent record too large");
_Static_assert(RECORD_AREA % FLASH_PAGE_SIZE == 0, "record area must be whole pages");

static uint32_t slot_offset(unsigned slot) {
    return PICO_FLASH_SIZE_BYTES - (SLOT_COUNT - slot) * FLASH_SECTOR_SIZE;
}

static const uint8_t *slot_data(unsigned slot) {
    return (const uint8_t *)(XIP_BASE + slot_offset(slot));
}

/* Newest valid slot, or -1. */
static int newest(ins10_persist *p, uint32_t *sequence) {
    int best = -1;
    uint32_t best_seq = 0;
    for (unsigned slot = 0; slot < SLOT_COUNT; ++slot) {
        ins10_persist candidate;
        uint32_t seq;
        if (!ins10_persist_decode(slot_data(slot), RECORD_AREA, &candidate, &seq)) continue;
        if (best < 0 || seq > best_seq) {
            best = (int)slot;
            best_seq = seq;
            if (p) *p = candidate;
        }
    }
    if (sequence) *sequence = best_seq;
    return best;
}

bool storage_load(ins10_persist *p) {
    return newest(p, NULL) >= 0;
}

typedef struct {
    uint32_t offset;
    const uint8_t *data; /* NULL: erase only */
} flash_job;

static void run_job(void *param) {
    const flash_job *job = (const flash_job *)param;
    flash_range_erase(job->offset, FLASH_SECTOR_SIZE);
    if (job->data) flash_range_program(job->offset, job->data, RECORD_AREA);
}

static bool execute(flash_job *job) {
    /* Interrupts are off and flash is unavailable for the duration (tens of
       milliseconds); the IMU FIFO absorbs most of it and the filter is told about
       any lost samples. */
    return flash_safe_execute(run_job, job, 1000) == PICO_OK;
}

bool storage_save(const ins10_persist *p) {
    static uint8_t buffer[RECORD_AREA];
    uint32_t sequence = 0;
    int current = newest(NULL, &sequence);
    unsigned target = current < 0 ? 0u : ((unsigned)current + 1u) % SLOT_COUNT;
    memset(buffer, 0xff, sizeof(buffer));
    if (ins10_persist_encode(p, sequence + 1, buffer, sizeof(buffer)) == 0) return false;
    flash_job job = {slot_offset(target), buffer};
    if (!execute(&job)) return false;
    ins10_persist check;
    uint32_t written;
    return ins10_persist_decode(slot_data(target), RECORD_AREA, &check, &written) &&
           written == sequence + 1;
}

bool storage_erase(void) {
    bool ok = true;
    for (unsigned slot = 0; slot < SLOT_COUNT; ++slot) {
        flash_job job = {slot_offset(slot), NULL};
        ok = execute(&job) && ok;
    }
    return ok;
}
