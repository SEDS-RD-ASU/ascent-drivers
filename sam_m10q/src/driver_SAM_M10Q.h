#ifndef DRIVER_SAM_M10Q_H
#define DRIVER_SAM_M10Q_H

#include "esp_err.h"

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
                           char *pBuffer);

int32_t uUbxProtocolDecode(const char *pBufferIn, size_t bufferLengthBytes,
                           int32_t *pMessageClass, int32_t *pMessageId,
                           char *pMessage, size_t maxMessageLengthBytes,
                           const char **ppBufferOut);

uint32_t uUbxProtocolUint32Decode(const char *pByte);

int32_t sendReceiveUbxMessage_I2C(int32_t messageClass,
                                         int32_t messageId,
                                         const char *pMessageBody,
                                         size_t messageBodyLengthBytes,
                                         uGnssPrivateUbxReceiveMessage_t *pResponse);

int32_t posDecode(char *pMessage,
                         int32_t *pLatitudeX1e7, int32_t *pLongitudeX1e7,
                         int32_t *pAltitudeMillimetres,
                         int32_t *pRadiusMillimetres,
                         int32_t *pAltitudeUncertaintyMillimetres,
                         int32_t *pSpeedMillimetresPerSecond,
                         int32_t *pSvs, int64_t *pTimeUtc, bool printIt);

#endif /* DRIVER_SAM_M10Q_H */ 