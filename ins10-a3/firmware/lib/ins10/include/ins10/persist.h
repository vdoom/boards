/* Everything the device keeps across power cycles, and its on-flash record format.
 *
 * Record: header {magic, version, payload length, sequence} + payload + CRC-32 of both.
 * The sequence number lets the storage layer alternate between two flash sectors and
 * pick the newer valid record after a power loss during a write.
 */
#ifndef INS10_PERSIST_H
#define INS10_PERSIST_H

#include "ins10/calib.h"
#include "ins10/thermal.h"
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define INS10_PERSIST_MAGIC   0x30314e49u /* "IN10" */
#define INS10_PERSIST_VERSION 1u

typedef struct {
    uint16_t att_rate_hz;  /* attitude output rate after power-up, 0 = off */
    uint8_t mag_enable;    /* 0 disables magnetometer fusion */
    uint8_t reserved;
} ins10_settings;

typedef struct {
    ins10_calibration cal;
    ins10_thermal_model thermal;
    ins10_settings settings;
} ins10_persist;

#define INS10_PERSIST_RECORD_SIZE (16u + sizeof(ins10_persist) + 4u)

void ins10_persist_defaults(ins10_persist *p);
uint32_t ins10_crc32(const void *data, size_t length);
/* Returns the record length, or 0 if the buffer is too small. */
size_t ins10_persist_encode(const ins10_persist *p, uint32_t sequence,
                            uint8_t *buffer, size_t capacity);
bool ins10_persist_decode(const uint8_t *buffer, size_t capacity,
                          ins10_persist *p, uint32_t *sequence);

#ifdef __cplusplus
}
#endif

#endif
