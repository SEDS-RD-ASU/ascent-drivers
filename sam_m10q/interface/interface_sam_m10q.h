#ifndef INTERFACE_SAM_M10Q_H
#define INTERFACE_SAM_M10Q_H

#include "driver_SAM_M10Q.h"

esp_err_t gps_init();

bool readUbxPort(ubxFrame *frame, int max_retries);

void ubxDisableNMEA();

bool ubxPollLocation();

void NMEAstfu();

void ubx10hz();

void freeUbxFrame(ubxFrame *frame);

void sendUbxFrame(ubxFrame frame);

#endif /* INTERFACE_SAM_M10Q_H */
