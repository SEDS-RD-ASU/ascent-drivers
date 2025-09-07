#ifndef DRIVER_SAM_M10Q_H
#define DRIVER_SAM_M10Q_H

typedef struct {
    uint8_t class;
    uint8_t id;
    uint16_t length;
    bool valid_checksum;
} sam_m10q_msginfo_t;


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
esp_err_t readNextGPSPacket(void);

/**
 * @brief Request UART Baudrate. This is a dummy function that I'm gonna use to try and activate the chip's data stream?
 */
esp_err_t requestUARTBaudrate(void);

/**
 * @brief Disable NMEA output on I2C
 */
esp_err_t disableNMEAoutprot(void);

/**
 * @brief Disable I2C timeout on I2C
 */
esp_err_t disableI2Ctimeout(void);

/**
 * @brief Get the message information for a given buffer
 */
sam_m10q_msginfo_t sam_m10q_get_msginfo(uint8_t *buf, uint16_t bufsize);

#endif /* DRIVER_SAM_M10Q_H */
