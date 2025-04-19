#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "driver_SAM_M10Q.h"

void gps_init() {
    sam_m10q_set_10hz();
    printf("Set 10hz for GPS!");
}

void parse_NMEA(float *gps_latitude, float *gps_longitude, uint32_t *gps_altitude) {
    char *sentences = readNmeaStream();
    char *sentence = strtok(sentences, "\n");

    while (sentence != NULL) {
        if (strncmp(sentence, "$GNRMC", 6) == 0) {
            char *token = strtok(sentence, ",");
            int fieldIndex = 0;
            char lat_dir = 'N';
            char lon_dir = 'E';
            float raw_lat = 0.0f;
            float raw_lon = 0.0f;

            while (token != NULL) {
                switch (fieldIndex) {
                    case 3: // Latitude in ddmm.mmmm
                        raw_lat = token[0] == '\0' ? 0.0f : atof(token);
                        break;
                    case 4: // N/S indicator
                        lat_dir = token[0];
                        break;
                    case 5: // Longitude in dddmm.mmmm
                        raw_lon = token[0] == '\0' ? 0.0f : atof(token);
                        break;
                    case 6: // E/W indicator
                        lon_dir = token[0];
                        break;
                }
                token = strtok(NULL, ",");
                fieldIndex++;
            }

            // Convert to decimal degrees
            int lat_deg = (int)(raw_lat / 100);
            float lat_min = raw_lat - (lat_deg * 100);
            *gps_latitude = lat_deg + (lat_min / 60.0f);
            if (lat_dir == 'S') *gps_latitude *= -1;

            int lon_deg = (int)(raw_lon / 100);
            float lon_min = raw_lon - (lon_deg * 100);
            *gps_longitude = lon_deg + (lon_min / 60.0f);
            if (lon_dir == 'W') *gps_longitude *= -1;
        }

        if (strncmp(sentence, "$GNGGA", 6) == 0) {
            char *token = strtok(sentence, ",");
            int fieldIndex = 0;
            while (token != NULL) {
                if (fieldIndex == 9) { // Altitude in meters
                    *gps_altitude = token[0] == '\0' ? 0 : (uint32_t)atof(token);
                    break;
                }
                token = strtok(NULL, ",");
                fieldIndex++;
            }
        }

        sentence = strtok(NULL, "\n");
    }

    free(sentences); // Free sentence buffer
}



