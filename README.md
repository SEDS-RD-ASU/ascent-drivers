# Ascent R2 Drivers

This directory contains the driver libraries for the Ascent R2 flight computer hardware components. Each library is organized in its own directory with source files and documentation.

## Library Structure

```
|--lib
   |
   |--ascent_r2_hardware_definition.h    # Centralized pin and hardware definitions
   |
   |--bno055                             # 9-DOF IMU sensor
   |  |--src
   |     |--driver_bno055.c
   |     |--driver_bno055.h
   |
   |--bmp390l                            # High-precision barometric pressure sensor
   |  |--src
   |     |--driver_BMP390L.c
   |     |--driver_BMP390L.h
   |
   |--h3lis331dl                         # High-g accelerometer
   |  |--src
   |     |--driver_H3LIS331DL.c
   |     |--driver_H3LIS331DL.h
   |
   |--sam_m10q                           # GNSS/GPS module
   |  |--src
   |     |--driver_SAM_M10Q.c
   |     |--driver_SAM_M10Q.h
   |
   |--spi                                # SPI bus abstraction layer
   |  |--src
   |     |--manual_spi_bus.c
   |     |--manual_spi_bus.h
   |
   |--w25qxx                             # Flash memory
   |  |--src
   |     |--driver_w25qxx_advance.c
   |     |--driver_w25qxx_advance.h
```

## Driver Components

### BNO055
- 9-DOF IMU sensor with sensor fusion
- Provides orientation, acceleration, and magnetic field data
- Supports multiple operation modes including NDOF fusion
- I2C interface with configurable pins

### BMP390L
- High-precision barometric pressure sensor
- Provides pressure and temperature measurements
- Used for altitude determination
- I2C interface with configurable pins

### H3LIS331DL
- High-g accelerometer (up to ±100g/±200g/±400g)
- Measures acceleration in three axes
- Used for launch and impact detection
- I2C interface with configurable pins

### SAM-M10Q
- GNSS/GPS module
- Provides position, velocity, and timing data
- Supports 10Hz update rate
- I2C interface with NMEA message parsing

### SPI Bus
- Manual SPI bus implementation
- Provides low-level SPI communication
- Used by W25QXX flash memory
- Configurable pins for MISO, MOSI, SCLK, and CS

### W25QXX
- High-speed flash memory
- Supports standard and quad SPI modes
- Used for data logging and configuration storage
- Advanced features including security registers and block protection

## Hardware Definition

The `ascent_r2_hardware_definition.h` file contains centralized pin definitions and hardware configurations for all components, including:
- I2C bus configuration
- SPI bus configuration
- Sensor-specific pins
- Pyrotechnic channel pins
- Indicator pins (LED, buzzer)
- Communication interface pins

## Building

PlatformIO will automatically compile these libraries into static libraries and link them into the executable file. Dependencies are managed through the `CMakeLists.txt` files in each component directory.

For more information about PlatformIO Library Dependency Finder:
- https://docs.platformio.org/page/librarymanager/ldf.html
