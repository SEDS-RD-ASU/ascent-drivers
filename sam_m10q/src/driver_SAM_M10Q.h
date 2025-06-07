#ifndef DRIVER_SAM_M10Q_H
#define DRIVER_SAM_M10Q_H

uint16_t ubxAvailableBytes();

void ubxReadBytes(uint8_t *buf, uint16_t num_bytes);

void ubxDisableNMEA();

void ubxEnableNavPVT();

void ubxReadStream(uint32_t *UTCtstamp, int32_t *lon, int32_t *lat, int32_t *height, int32_t *hMSL, uint8_t *fixType, uint8_t *numSV);

void ubxReadStreamTiming();

void ubxResetGPS();

void ubx10HzGPS();

// void ubx25hzGPS(); // DOO NOOTTT USE UNLESS YOU KNOW WHAT YOURE DOING.

void ubxConstellations();

void ubxMsgOutCfg();

void ubxFreezeTimePulse();

void ubxDisableBDS_B1();

#endif /* DRIVER_SAM_M10Q_H */
