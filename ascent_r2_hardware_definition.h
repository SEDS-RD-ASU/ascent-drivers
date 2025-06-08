#ifndef ASCENT_R2_PINS_H
#define ASCENT_R2_PINS_H

#include "driver/gpio.h"
#include "driver/i2c.h"

/* I2C Bus Configuration */
#define I2C_MASTER_SCL_IO           GPIO_NUM_6      // I2C Clock pin
#define I2C_MASTER_SDA_IO           GPIO_NUM_5      // I2C Data pin
#define I2C_MASTER_FREQ_HZ          400000          // I2C master clock frequency
#define I2C_MASTER_PORT             I2C_NUM_0       // I2C port number (I2C_NUM_0 or I2C_NUM_1)

/* SPI Bus Configuration */
#define PIN_SPI_SCK                 GPIO_NUM_11     // SPI Clock pin
#define PIN_SPI_MOSI                GPIO_NUM_13     // SPI MOSI pin
#define PIN_SPI_MISO                GPIO_NUM_12     // SPI MISO pin

/* Sensor-Specific Pins */
// BNO055 IMU
#define PIN_BNO055_INT              GPIO_NUM_18     // BNO055 interrupt pin
#define BNO055_I2C_ADDR             0x28            // BNO055 I2C address

// BMP390L Barometer
#define PIN_BMP390_INT              GPIO_NUM_45     // BMP390L interrupt pin
#define BMP390_I2C_ADDR             0x76            // BMP390L I2C address

// H3LIS331DL Accelerometer
#define PIN_H3LIS_INT1              GPIO_NUM_38     // H3LIS331DL interrupt 1 pin
#define H3LIS331DL_I2C_ADDR         0x18            // H3LIS331DL I2C address

// SAM-M10Q GPS
#define SAM_M10Q_I2C_ADDR           0x42            // SAM-M10Q I2C address

//W25Q512 Flash
#define FLASH_CS                    GPIO_NUM_8     // W25Q512 Flash CS pin

//HopeRF RFM95W LoRa SX1276 Radio
#define LORA_CS                     GPIO_NUM_10     //LoRa CS
#define LORA_DIO0                   GPIO_NUM_9      //LoRa DIO0

//PSU Pins
#define VBATT                     GPIO_NUM_17     // VBATT Enable

//Pyrotechnic channel Pins
#define PYRO1_OUT                   GPIO_NUM_39     // Pyro channel 1 (APOGEE)
#define PYRO2_OUT                   GPIO_NUM_40     // Pyro channel 2 (MAINS)
#define PYRO3_OUT                   GPIO_NUM_41     // Pyro channel 3
#define PYRO4_OUT                   GPIO_NUM_42     // Pyro channel 4
#define PYRO1_CONT                  GPIO_NUM_1      // Pyro Continuity channel 1 (APOGEE)
#define PYRO2_CONT                  GPIO_NUM_2      // Pyro Continuity channel 2 (MAINS)
#define PYRO3_CONT                  GPIO_NUM_3      // Pyro Continuity channel 3
#define PYRO4_CONT                  GPIO_NUM_4      // Pyro Continuity channel 4

//Indicator Pins
#define PIN_LED                     GPIO_NUM_21      // Onboard LED
#define PIN_BUZZER                  GPIO_NUM_7       //Buzzer

//Interposer Pins
#define UART_TXD                    GPIO_NUM_4      // UART TX pin
#define UART_RXD                    GPIO_NUM_5      // UART RX pin
#define UART_PORT_NUM              UART_NUM_1      // UART port number
#define UART_BAUD_RATE             115200          // UART baud rate

#define DIODE_DROP 0.35f
#define DIVIDER_RATIO 3.778f
#define MAGIC 1.333
#define PYRO_RESISTANCE_COEFFICIENT 360000.0f
#define MUTEX_TIMEOUT 100

#endif /* ASCENT_R2_PINS_H */ 