#ifndef DRIVER_SAM_M10Q_H
#define DRIVER_SAM_M10Q_H

uint16_t ubxAvailableBytes();

void ubxReadBytes(uint8_t *buf, uint16_t num_bytes);

void ubxDisableNMEA();

void ubxEnableNavPVT();

void ubxReadStream();

void ubxReadStreamTiming();

void ubxResetGPS();

void ubx10HzGPS();

// void ubx25hzGPS(); // DOO NOOTTT USE UNLESS YOU KNOW WHAT YOURE DOING.

void ubxConstellations();

void ubxMsgOutCfg();

void ubxFreezeTimePulse();

#endif /* DRIVER_SAM_M10Q_H */