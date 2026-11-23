/* LPS22DF barometer driver: 25 Hz continuous conversion, polled without blocking.
 * The pressure is not used by the attitude filter; it is reported for the user. */
#ifndef INS10_LPS22DF_H
#define INS10_LPS22DF_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool ready, fresh;
    uint64_t t_poll, t_last;
    float pressure_hpa, temp_c;
    uint32_t samples, errors;
} lps_state;

bool lps_init(lps_state *st);
/* Returns false after the sensor has been silent for a second; re-initialise then. */
bool lps_task(lps_state *st, uint64_t now_us);
bool lps_take(lps_state *st, float *pressure_hpa, float *temp_c);
/* International barometric formula, metres above the 1013.25 hPa level. */
float lps_altitude_m(float pressure_hpa);

#endif
