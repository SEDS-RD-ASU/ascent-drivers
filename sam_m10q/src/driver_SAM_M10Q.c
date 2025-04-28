#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_manager.h"
#include "esp_err.h"
#include "ascent_r2_hardware_definition.h"


static const char gDaysInMonth[] = {31, 28, 31, 30, 31, 30, 31,
                                    31, 30, 31, 30, 31
                                   };
static const char gDaysInMonthLeapYear[] = {31, 29, 31, 30, 31, 30,
                                            31, 31, 30, 31, 30, 31
                                           };

static const char *TAG = "SAM-M10Q";

typedef struct {
    int32_t cls;
    int32_t id;
    char **ppBody;   /**< a pointer to a pointer that the received
                          message body will be written to. If *ppBody is NULL
                          memory will be allocated.  If ppBody is NULL
                          then the response is not captured (but this
                          structure may still be used for cls/id matching). */
    size_t bodySize; /**< the number of bytes of storage at *ppBody;
                          must be zero if ppBody is NULL or
                          *ppBody is NULL.  If non-zero it MUST be
                          large enough to fit the body in or the
                          CRC calculation will fail. */
} uGnssPrivateUbxReceiveMessage_t;

int32_t uUbxProtocolEncode(int32_t messageClass, int32_t messageId,
                           const char *pMessage, size_t messageBodyLengthBytes,
                           char *pBuffer)
{
    int32_t errorCodeOrLength = (int32_t) -5;
    // Use a uint8_t pointer for maths, more certain of its behaviour than char
    uint8_t *pWrite = (uint8_t *) pBuffer;
    int32_t ca = 0;
    int32_t cb = 0;

    if (((messageBodyLengthBytes == 0) || (pMessage != NULL)) &&
        (pBuffer != NULL)) {

        // Complete the header
        *pWrite++ = 0xb5;
        *pWrite++ = 0x62;
        *pWrite++ = (uint8_t) messageClass;
        *pWrite++ = (uint8_t) messageId;
        *pWrite++ = (uint8_t) (messageBodyLengthBytes & (uint8_t) 0xff);
        *pWrite++ = (uint8_t) (messageBodyLengthBytes >> 8);

        if (pMessage != NULL) {
            // Copy in the message body
            memcpy(pWrite, pMessage, messageBodyLengthBytes);
            pWrite += messageBodyLengthBytes;
        }

        // Work out the CRC over the variable elements of the
        // header and the body
        pBuffer += 2;
        for (size_t x = 0; x < messageBodyLengthBytes + 4; x++) {
            ca += (uint8_t) *pBuffer; // *NOPAD*
            cb += ca;
            pBuffer++;
        }

        // Write in the CRC
        *pWrite++ = (uint8_t) (ca & (uint8_t) 0xff);
        *pWrite = (uint8_t) (cb & (uint8_t) 0xff);

        errorCodeOrLength = (int32_t) (8 + messageBodyLengthBytes);
    }

    return errorCodeOrLength;
}


// Decode a UBX protocol message.
int32_t uUbxProtocolDecode(const char *pBufferIn, size_t bufferLengthBytes,
                           int32_t *pMessageClass, int32_t *pMessageId,
                           char *pMessage, size_t maxMessageLengthBytes,
                           const char **ppBufferOut)
{
    int32_t sizeOrErrorCode = (int32_t) -11;
    // Use a uint8_t pointer for maths, more certain of its behaviour than char
    const uint8_t *pInput = (const uint8_t *) pBufferIn;
    int32_t overheadByteCount = 0;
    bool updateCrc = false;
    size_t expectedMessageByteCount = 0;
    size_t messageByteCount = 0;
    int32_t ca = 0;
    int32_t cb = 0;

    for (size_t x = 0; (x < bufferLengthBytes) &&
         (overheadByteCount < 8); x++) {
        switch (overheadByteCount) {
            case 0:
                //lint -e{650} Suppress warning about 0xb5 being out of range for char
                if (*pInput == 0xb5) {
                    // Got first byte of header, increment count
                    overheadByteCount++;
                }
                break;
            case 1:
                if (*pInput == 0x62) {
                    // Got second byte of header, increment count
                    overheadByteCount++;
                } else {
                    // Not a valid message, start again
                    overheadByteCount = 0;
                }
                break;
            case 2:
                // Got message class, store it, start CRC
                // calculation and increment count
                if (pMessageClass != NULL) {
                    *pMessageClass = *pInput;
                }
                ca = 0;
                cb = 0;
                updateCrc = true;
                overheadByteCount++;
                break;
            case 3:
                // Got message ID, store it, update CRC and
                // increment count
                if (pMessageId != NULL) {
                    *pMessageId = *pInput;
                }
                updateCrc = true;
                overheadByteCount++;
                break;
            case 4:
                // Got first byte of length, store it, update
                // CRC and increment count
                expectedMessageByteCount = *pInput;
                updateCrc = true;
                overheadByteCount++;
                break;
            case 5:
                // Got second byte of length, add it to the first,
                // updat CRC, increment count and reset the
                // message byte count ready for the body to come next.
                // Cast twice to keep Lint happy
                expectedMessageByteCount += ((size_t) *pInput) << 8; // *NOPAD*
                messageByteCount = 0;
                updateCrc = true;
                overheadByteCount++;
                break;
            case 6:
                if (messageByteCount < expectedMessageByteCount) {
                    // Store the next byte of the message and
                    // update CRC
                    if ((pMessage != NULL) && (messageByteCount < maxMessageLengthBytes)) {
                        *pMessage++ = (char) *pInput; // *NOPAD*
                    }
                    updateCrc = true;
                    messageByteCount++;
                } else {
                    // First byte of CRC, check it
                    ca &= 0xff;
                    if ((uint8_t) ca == *pInput) {
                        overheadByteCount++;
                    } else {
                        // Not a valid message, start again
                        overheadByteCount = 0;
                    }
                }
                break;
            case 7:
                // Second byte of CRC, check it
                cb &= 0xff;
                if ((uint8_t) cb == *pInput) {
                    overheadByteCount++;
                } else {
                    // Not a valid message, start again
                    overheadByteCount = 0;
                }
                break;
            default:
                overheadByteCount = 0;
                break;
        }

        if (updateCrc) {
            ca += *pInput;
            cb += ca;
            updateCrc = false;
        }

        // Next byte
        pInput++;
    }

    if (overheadByteCount > 0) {
        // We got some parts of the message overhead, so
        // could be a message
        sizeOrErrorCode = (int32_t) -9;
        if (overheadByteCount == 8) {
            // We got all the overhead bytes, this is a complete message
            sizeOrErrorCode = (int32_t) messageByteCount;
        }
    }

    if (ppBufferOut != NULL) {
        *ppBufferOut =  (const char *) pInput;
    }

    return sizeOrErrorCode;
}

uint32_t uUbxProtocolUint32Decode(const char *pByte)
{
    // Use a uint8_t pointer for maths, more certain of its behaviour than char
    const uint8_t *pInput = (const uint8_t *) pByte;
    uint32_t retValue;

    retValue  = *pInput;
    // Cast twice to keep Lint happy
    retValue += ((uint32_t) *(pInput + 1)) << 8;  // *NOPAD*
    retValue += ((uint32_t) *(pInput + 2)) << 16; // *NOPAD*
    retValue += ((uint32_t) *(pInput + 3)) << 24; // *NOPAD*

    return retValue;
}

// what
bool uUbxProtocolIsLittleEndian()
{
    int32_t x = 1;

    return (*((char *) (&x)) == 1);
}

uint16_t uUbxProtocolUint16Encode(uint16_t uint16)
{
    uint16_t retValue = uint16;

    if (!uUbxProtocolIsLittleEndian()) {
        retValue  = (uint16 & 0xFF00) >> 8;
        retValue += (uint16 & 0x00FF) << 8;
    }

    return retValue;
}

uint16_t uUbxProtocolUint16Decode(const char *pByte)
{
    // Use a uint8_t pointer for maths, more certain of its behaviour than char
    const uint8_t *pInput = (const uint8_t *) pByte;
    uint16_t retValue;

    retValue  = *pInput;
    // Cast twice to keep Lint happy
    retValue += (uint16_t) (((uint16_t) *(pInput + 1)) << 8); // *NOPAD*

    return  retValue;
}

bool uTimeIsLeapYear(int32_t year)
{
    bool isLeapYear = false;

    if (((year % 4 == 0) && (year % 100 != 0)) || (year % 400 == 0)) {
        isLeapYear = true;
    }

    return isLeapYear;
}

int64_t uTimeMonthsToSecondsUtc(int32_t monthsUtc)
{
    int64_t secondsUtc = 0;

    for (int32_t x = 0; x < monthsUtc; x++) {
        if (uTimeIsLeapYear((x / 12) + 1970)) {
            secondsUtc += gDaysInMonthLeapYear[x % 12] * 3600 * 24;
        } else {
            secondsUtc += gDaysInMonth[x % 12] * 3600 * 24;
        }
    }

    return secondsUtc;
}

int32_t sendReceiveUbxMessage_I2C(int32_t messageClass,
                                         int32_t messageId,
                                         const char *pMessageBody,
                                         size_t messageBodyLengthBytes,
                                         uGnssPrivateUbxReceiveMessage_t *pResponse)
{
    int32_t                result = (int32_t) -5;
    char                  *pTxBuf = NULL;
    int32_t                txLength;
    esp_err_t              err;
    uint8_t                header[6];
    uint16_t               payloadLen;
    uint8_t               *pRxBuf = NULL;

    printf("Starting sendReceiveUbxMessage_I2C with messageClass: %ld, messageId: %ld, pMessageBody: %p, messageBodyLengthBytes: %zu, pResponse: %p\n", messageClass, messageId, pMessageBody, messageBodyLengthBytes, pResponse);

    // 1) Parameter checks
    if ((((pMessageBody == NULL) && (messageBodyLengthBytes == 0)) ||
         (pMessageBody != NULL && messageBodyLengthBytes > 0)) &&
        (pResponse != NULL)) {

        printf("Parameter checks passed\n");

        // 2) Allocate TX buffer
        pTxBuf = malloc(messageBodyLengthBytes + 8);
        if (pTxBuf == NULL) {
            printf("Failed to allocate TX buffer\n");
            return (int32_t) -6;
        }

        printf("TX buffer allocated successfully\n");

        // 3) Encode UBX frame
        txLength = uUbxProtocolEncode(messageClass,
                                      messageId,
                                      pMessageBody,
                                      messageBodyLengthBytes,
                                      pTxBuf);
        if (txLength < 0) {
            printf("Failed to encode UBX frame\n");
            result = txLength;
            goto clean_up;
        }

        printf("UBX frame encoded successfully\n");

        // 4) Send over I2C (register 0x00 = write-data)
        err = i2c_manager_write_register(I2C_MASTER_PORT,
                                         SAM_M10Q_I2C_ADDR,
                                         0x00,
                                         (uint8_t *) pTxBuf,
                                         (size_t) txLength);
        if (err != ESP_OK) {
            printf("Failed to send over I2C\n");
            result = (int32_t) err;
            goto clean_up;
        }

        printf("Sent over I2C successfully\n");

        // 5) Read the 6-byte UBX header in a loop until the correct sync header is received
        while (1) {
            err = i2c_manager_read_register(I2C_MASTER_PORT,
                                            SAM_M10Q_I2C_ADDR,
                                            /* read-reg */ 0xFF,
                                            header,
                                            sizeof(header));
            if (err != ESP_OK) {
                printf("Failed to read UBX header\n");
                result = (int32_t) -69;
                goto clean_up;
            }

            // 6) Validate sync chars
            if (header[0] == 0xB5 && header[1] == 0x62) {
                printf("Correct sync header received\n");
                break; // Correct sync header received, exit the loop
            }
        }

        printf("UBX header read successfully\n");

        // 7) Extract payload length
        payloadLen = (uint16_t) header[4] | ((uint16_t) header[5] << 8);

        printf("Payload length extracted: %u\n", payloadLen);

        // 8) Ensure pResponse has room for payload + checksum (or allocate)
        size_t totalLen = (size_t) (payloadLen + 10);

        printf("Total length needed for pResponse: %zu\n", totalLen);

        printf("bodySize: %d\n", pResponse->bodySize);

        // if (pResponse->bodySize < totalLen) {
        //     printf("pResponse buffer size is insufficient\n");
        //     // free old buffer if any
        //     if (*pResponse->ppBody != NULL) {
        //         free(*pResponse->ppBody);
        //         printf("Freed pResponse!\n");
        //         *pResponse->ppBody = NULL;
        //     }
        //     *pResponse->ppBody = malloc(totalLen);
        //     if (*pResponse->ppBody == NULL) {
        //         printf("Failed to allocate new buffer for pResponse\n");
        //         result = (int32_t) -6;
        //         goto clean_up;
        //     }
        //     pResponse->bodySize = totalLen;
        // }
        // pRxBuf = (uint8_t *) *pResponse->ppBody;

        // printf("pResponse buffer size adjusted successfully\n");

        // 9) Read payload + 2-byte checksum
        err = i2c_manager_read_register(I2C_MASTER_PORT,
                                        SAM_M10Q_I2C_ADDR,
                                        /* read-reg */ 0xFF,
                                        pRxBuf,
                                        totalLen);
        if (err != ESP_OK) {
            printf("Failed to read payload + checksum\n");
            result = (int32_t) -69;
            goto clean_up;
        }

        printf("Payload + checksum read successfully\n");

        // 11) Fill out the response struct
        pResponse->cls = header[2];
        pResponse->id  = header[3];
        // bodySize already set
        result = (int32_t) payloadLen;

        printf("Response struct filled successfully\n");
    }

clean_up:
    if (pTxBuf != NULL) {
        free(pTxBuf);
        pTxBuf = NULL;
        printf("TX buffer freed\n");
    }
    return result;
}


int32_t posDecode(char *pMessage,
                         int32_t *pLatitudeX1e7, int32_t *pLongitudeX1e7,
                         int32_t *pAltitudeMillimetres,
                         int32_t *pRadiusMillimetres,
                         int32_t *pAltitudeUncertaintyMillimetres,
                         int32_t *pSpeedMillimetresPerSecond,
                         int32_t *pSvs, int64_t *pTimeUtc, bool printIt)
{
    int32_t errorCode = (int32_t) -9;
    int32_t months;
    int32_t year;
    int32_t y;
    int64_t t = -1;

    if ((*(pMessage + 11) & 0x03) == 0x03) {
        // Time and date are valid; we don't indicate
        // success based on this but we report it anyway
        // if it is valid
        t = 0;
        // Year is 1999-2099, so need to adjust to get year since 1970
        year = ((int32_t) uUbxProtocolUint16Decode(pMessage + 4) - 1999) + 29;
        // Month (1 to 12), so take away 1 to make it zero-based
        months = *(pMessage + 6) - 1;
        months += year * 12;
        // Work out the number of seconds due to the year/month count
        t += uTimeMonthsToSecondsUtc(months);
        // Day (1 to 31)
        t += ((int32_t) * (pMessage + 7) - 1) * 3600 * 24;
        // Hour (0 to 23)
        t += ((int32_t) * (pMessage + 8)) * 3600;
        // Minute (0 to 59)
        t += ((int32_t) * (pMessage + 9)) * 60;
        // Second (0 to 60)
        t += *(pMessage + 10);
        if (printIt) {
            printf("U_GNSS_POS: UTC time = %ld.\n", (int32_t) t);
        }
    }
    if (pTimeUtc != NULL) {
        *pTimeUtc = t;
    }
    // From here onwards Lint complains about accesses
    // into message[] and it doesn't seem to be possible
    // to suppress those warnings with -esym(690, message)
    // or even -e(690), hence do it the blunt way
    //lint -save -e690
    if ((t >= 0) && (*(pMessage + 21) & 0x01)) {
        if (printIt) {
            printf("U_GNSS_POS: %dD fix achieved.\n", *(pMessage + 20));
        }
        y = (int32_t) * (pMessage + 23);
        if (printIt) {
            printf("U_GNSS_POS: satellite(s) = %ld.\n", y);
        }
        if (pSvs != NULL) {
            *pSvs = y;
        }
        y = (int32_t) uUbxProtocolUint32Decode(pMessage + 24);
        if (printIt) {
            printf("U_GNSS_POS: longitude = %ld (degrees * 10^7).\n", y);
        }
        if (pLongitudeX1e7 != NULL) {
            *pLongitudeX1e7 = y;
        }
        y = (int32_t) uUbxProtocolUint32Decode(pMessage + 28);
        if (printIt) {
            printf("U_GNSS_POS: latitude = %ld (degrees * 10^7).\n", y);
        }
        if (pLatitudeX1e7 != NULL) {
            *pLatitudeX1e7 = y;
        }
        y = INT_MIN;
        if (*(pMessage + 20) == 0x03) {
            y = (int32_t) uUbxProtocolUint32Decode(pMessage + 36);
            if (printIt) {
                printf("U_GNSS_POS: altitude = %ld (mm).\n", y);
            }
        }
        if (pAltitudeMillimetres != NULL) {
            *pAltitudeMillimetres = y;
        }
        y = (int32_t) uUbxProtocolUint32Decode(pMessage + 40);
        if (printIt) {
            printf("U_GNSS_POS: radius = %ld (mm).\n", y);
        }
        if (pRadiusMillimetres != NULL) {
            *pRadiusMillimetres = y;
        }
        y = (int32_t) uUbxProtocolUint32Decode(pMessage + 44);
        if (printIt) {
            printf("U_GNSS_POS: altitude uncertainty = %ld (mm).\n", y);
        }
        if (pAltitudeUncertaintyMillimetres != NULL) {
            *pAltitudeUncertaintyMillimetres = y;
        }
        y = (int32_t) uUbxProtocolUint32Decode(pMessage + 60);
        if (printIt) {
            printf("U_GNSS_POS: speed = %ld (mm/s).\n", y);
        }
        if (pSpeedMillimetresPerSecond != NULL) {
            *pSpeedMillimetresPerSecond = y;
        }
        errorCode = (int32_t) 0;
        //lint -restore
    }

    return errorCode;
}