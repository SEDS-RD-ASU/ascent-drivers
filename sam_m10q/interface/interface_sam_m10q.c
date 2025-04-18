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
            while (token != NULL) {
                switch (fieldIndex) {
                    case 3: // Latitude
                        *gps_latitude = token[0] == '\0' ? 0 : atof(token);
                        break;
                    case 5: // Longitude
                        *gps_longitude = token[0] == '\0' ? 0 : atof(token);
                        break;
                    case 9: // Altitude
                        *gps_altitude = token[0] == '\0' ? 0 : (uint32_t)atof(token);
                        break;
                }
                token = strtok(NULL, ",");
                fieldIndex++;
            }
            free(token);
        }

        sentence = strtok(NULL, "\n");
    }

    free(sentences); // Ensure memory is freed after use
    
}



