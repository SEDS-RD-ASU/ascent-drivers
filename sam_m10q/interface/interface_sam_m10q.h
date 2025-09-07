#ifndef INTERFACE_SAM_M10Q_H
#define INTERFACE_SAM_M10Q_H

#include "driver_SAM_M10Q.h"

void GPS_init(void);

void GPS_read(uint32_t *UTCtstamp, int32_t *lon, int32_t *lat, int32_t *height, int32_t *hMSL, uint8_t *fixType, uint8_t *numSV);

void GPS_timing_debug(void);

void GPS_reset(void);

void GPS_low_power_mode(void);

void GPS_high_power_mode(void);

#endif /* INTERFACE_SAM_M10Q_H */
