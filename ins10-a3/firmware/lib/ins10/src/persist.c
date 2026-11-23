#include "ins10/persist.h"
#include <string.h>

void ins10_persist_defaults(ins10_persist *p) {
    memset(p, 0, sizeof(*p));
    ins10_cal_defaults(&p->cal);
    p->settings.att_rate_hz = 50;
    p->settings.mag_enable = 1;
}

uint32_t ins10_crc32(const void *data, size_t length) {
    /* CRC-32/ISO-HDLC (zlib), bitwise: the record is a few hundred bytes. */
    const uint8_t *bytes = (const uint8_t *)data;
    uint32_t crc = 0xffffffffu;
    for (size_t i = 0; i < length; ++i) {
        crc ^= bytes[i];
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc >> 1) ^ (0xedb88320u & (0u - (crc & 1u)));
        }
    }
    return ~crc;
}

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t length;
    uint32_t sequence;
} record_header;

size_t ins10_persist_encode(const ins10_persist *p, uint32_t sequence,
                            uint8_t *buffer, size_t capacity) {
    record_header header = {INS10_PERSIST_MAGIC, INS10_PERSIST_VERSION,
                            (uint32_t)sizeof(*p), sequence};
    size_t body = sizeof(header) + sizeof(*p);
    if (capacity < body + 4) return 0;
    memcpy(buffer, &header, sizeof(header));
    memcpy(buffer + sizeof(header), p, sizeof(*p));
    uint32_t crc = ins10_crc32(buffer, body);
    memcpy(buffer + body, &crc, sizeof(crc));
    return body + 4;
}

bool ins10_persist_decode(const uint8_t *buffer, size_t capacity,
                          ins10_persist *p, uint32_t *sequence) {
    record_header header;
    size_t body = sizeof(header) + sizeof(*p);
    if (capacity < body + 4) return false;
    memcpy(&header, buffer, sizeof(header));
    if (header.magic != INS10_PERSIST_MAGIC || header.version != INS10_PERSIST_VERSION ||
        header.length != sizeof(*p)) return false;
    uint32_t crc;
    memcpy(&crc, buffer + body, sizeof(crc));
    if (crc != ins10_crc32(buffer, body)) return false;
    memcpy(p, buffer + sizeof(header), sizeof(*p));
    if (sequence) *sequence = header.sequence;
    return true;
}
