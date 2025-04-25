#ifndef INTERFACE_SAM_M10Q_H
#define INTERFACE_SAM_M10Q_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the SAM-M10Q GNSS module.
 */
void gps_init(void);

/**
 * @brief Request and decode a UBX-NAV-PVT message to extract GNSS data.
 *
 * @param pLatitudeX1e7                   Pointer to store latitude (degrees × 1e7)
 * @param pLongitudeX1e7                  Pointer to store longitude (degrees × 1e7)
 * @param pAltitudeMillimetres            Pointer to store altitude in millimetres
 * @param pRadiusMillimetres              Pointer to store horizontal uncertainty radius in mm
 * @param pAltitudeUncertaintyMillimetres Pointer to store vertical uncertainty in mm
 * @param pSpeedMillimetresPerSecond      Pointer to store speed in mm/s
 * @param pSvs                            Pointer to store number of satellites used
 * @param pTimeUtc                        Pointer to store UTC time (seconds since UNIX epoch)
 * @param printIt                         If true, prints debug info to stdout
 * @return
 *   *  ≥0 expected payload length (92) on success  
 *   *  –10 if frame received but wrong length  
 *   *  –9, –6, etc., for other errors
 */
int32_t readUBX(int32_t *pLatitudeX1e7,
                int32_t *pLongitudeX1e7,
                int32_t *pAltitudeMillimetres,
                int32_t *pRadiusMillimetres,
                int32_t *pAltitudeUncertaintyMillimetres,
                int32_t *pSpeedMillimetresPerSecond,
                int32_t *pSvs,
                int64_t *pTimeUtc,
                bool printIt);

#ifdef __cplusplus
}
#endif

#endif // INTERFACE_SAM_M10Q_H