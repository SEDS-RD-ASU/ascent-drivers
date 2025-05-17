#ifndef DRIVER_SAM_M10Q_H
#define DRIVER_SAM_M10Q_H


#define SYNC_CHARS 0x62B5

// UBX Data Types
typedef uint8_t U1;  // unsigned 8-bit integer
typedef int8_t I1;   // signed 8-bit integer
typedef uint8_t X1;  // 8-bit bitfield
typedef uint16_t U2; // unsigned little-endian 16-bit integer
typedef int16_t I2;  // signed little-endian 16-bit integer
typedef uint16_t X2; // 16-bit little-endian bitfield
typedef uint32_t U4; // unsigned little-endian 32-bit integer
typedef int32_t I4;  // signed little-endian 32-bit integer
typedef uint32_t X4; // 32-bit little-endian bitfield

typedef float R4;    // IEEE 754 single (32-bit) precision
typedef double R8;   // IEEE 754 double (64-bit) precision
typedef char CH;     // ASCII / ISO 8859-1 character (8-bit)

typedef uint8_t *Un;  // unsigned bitfield of n bits width (variable size)
typedef int8_t *In;   // signed bitfield of n bits width (variable size)
typedef int8_t *Sn;   // signed bitfield value of n bits width (variable size)

// i am sincerely sorry for anyone that has to read this code with these weird definitions (ublox moment).

// UBX Frame Structure
typedef struct {
    U2 sync_chars;
    U1 message_class;
    U1 message_id;
    U2 length;
    Un payload;
    U2 checksum;
} ubxFrame;

// Config layer enum
typedef enum {
    RAM = 0,
    BBR = 1,
    Flash = 2,
} ubxCfgLayer;

// Config data structure
typedef struct {
    U4 key_id;
    Un value;
    U1 value_size; // size of the value in bytes
} ubxCfgData;

void ubxChecksum(uint8_t *buffer, int packet_length, U2 *checksum);

void ubxAckAck(ubxFrame *ackframe, ubxFrame sentframe);

void ubxAckNak(ubxFrame *nakframe, ubxFrame sentframe);

void ubxCfgValset(ubxFrame *frame, U1 layer, ubxCfgData data, ubxCfgData data2, uint8_t repeat_times);

void sendUbxFrame(ubxFrame frame);

void ubxCfgRstStartGNSS(ubxFrame *frame);

#endif /* DRIVER_SAM_M10Q_H */