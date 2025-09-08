#ifndef DRIVER_SAM_M10Q_H
#define DRIVER_SAM_M10Q_H

typedef struct {
    uint8_t class;
    uint8_t id;
    uint16_t length;
    bool valid_checksum;
} sam_m10q_msginfo_t;


// black magic to help parse navpvt

#define U2(o) ((uint16_t)payload[o] | ((uint16_t)payload[o+1] << 8))
#define U4(o) ((uint32_t)payload[o] | ((uint32_t)payload[o+1] << 8) | ((uint32_t)payload[o+2] << 16) | ((uint32_t)payload[o+3] << 24))
#define I4(o) ((int32_t)( (uint32_t)payload[o]        | \
                              ((uint32_t)payload[o+1] << 8)  | \
                              ((uint32_t)payload[o+2] << 16) | \
                              ((uint32_t)payload[o+3] << 24)))




typedef struct {
    uint32_t iTOW;          // GPS time of week of the navigation epoch (ms)
    uint16_t year;          // Year (UTC)
    uint8_t  month;         // Month, range 1..12 (UTC)
    uint8_t  day;           // Day of month, range 1..31 (UTC)
    uint8_t  hour;          // Hour of day, range 0..23 (UTC)
    uint8_t  min;           // Minute of hour, range 0..59 (UTC)
    uint8_t  sec;           // Seconds of minute, range 0..60 (UTC)
    uint8_t  valid;         // Validity flags
    uint32_t tAcc;          // Time accuracy estimate (ns)
    int32_t  nano;          // Fraction of second, range -1e9..1e9 (ns)
    uint8_t  fixType;       // GNSS fix type
    uint8_t  flags;         // Fix status flags
    uint8_t  flags2;        // Additional flags
    uint8_t  numSV;         // Number of satellites used in Nav Solution
    double  lon;            // Longitude
    double  lat;            // Latitude
    int32_t  height;        // Height above ellipsoid (mm)
    int32_t  hMSL;          // Height above mean sea level (mm)
    uint32_t hAcc;          // Horizontal accuracy estimate (mm)
    uint32_t vAcc;          // Vertical accuracy estimate (mm)
    int32_t  velN;          // NED north velocity (mm/s)
    int32_t  velE;          // NED east velocity (mm/s)
    int32_t  velD;          // NED down velocity (mm/s)
    int32_t  gSpeed;        // Ground speed (2-D) (mm/s)
    int32_t  headMot;       // Heading of motion (2-D, 1e-5 deg)
    uint32_t sAcc;          // Speed accuracy estimate (mm/s)
    uint32_t headAcc;       // Heading accuracy estimate (1e-5 deg)
    uint16_t pDOP;          // Position DOP (0.01)
    uint16_t reserved1;     // Alignment padding (2 bytes, from UBX spec)
    uint32_t flags3;        // Additional flags
} sam_m10q_navpvt_t;


/**
 * @brief Send a buffer of bytes via I2C;
*/
esp_err_t sendGPSBytes(uint8_t *buf, uint16_t num_bytes);

/**
 * @brief Read a buffer of bytes via I2C;
*/
esp_err_t readGPSBytes(uint8_t *buf, uint16_t num_bytes);

/**
 * @brief Reads the I2C packet stream for avaliable bytes
 */
esp_err_t readNextGPSPacket(sam_m10q_msginfo_t *msginfo, uint8_t *buf, uint16_t *buf_length);

/**
 * @brief Request UART Baudrate. This is a dummy function that I'm gonna use to try and activate the chip's data stream?
 */
esp_err_t requestUARTBaudrate(void);

/**
 * @brief Disable I2C timeout on I2C
 */
esp_err_t disableI2Ctimeout(void);

/**
 * @brief Disable NMEA output
 */
esp_err_t disableNMEAMessages(void);

/**
 * @brief Get the message information for a given buffer
 */
sam_m10q_msginfo_t gpsIdentifyMessage(uint8_t *buf, uint16_t bufsize);

/**
 * @brief Set the GPS to 10Hz
 */
esp_err_t setGPS10hz(void);

/**
 * @brief Request the NAV-PVT message
 */
esp_err_t reqNAVPVT(void);

/**
 * @brief Parse the NAV-PVT message
 */
sam_m10q_navpvt_t gpsParseNavPVT(uint8_t *gps_packet_buf);

#endif /* DRIVER_SAM_M10Q_H */
