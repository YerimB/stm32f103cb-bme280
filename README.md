# STM32F103CB + BME280 Sensor Project

This project provides a bare-metal firmware example for interfacing an STM32F103CB microcontroller (on a WeAct "Blue Pill Plus" board) with a Bosch BME280 environmental sensor. It measures temperature, pressure, and humidity, displaying the data on a 16x2 I2C LCD and sending it over a UART serial connection.

The firmware is written in C using CMSIS headers and focuses on a clean, modular driver design for peripherals (SPI, I2C, UART, Clock, LED) and external components (BME280, LCD1602).

## Features

-   **System Clock**: Configurable clock using HSE crystal with PLL (e.g., 48MHz or 72MHz).
-   **BME280 Driver**:
    -   Supports both I2C (default) and SPI communication, selectable at compile time.
    -   Supports "Normal" mode (continuous sampling) and "Forced" mode (on-demand sampling).
    -   Implements Bosch's official compensation algorithms for high-accuracy readings.
-   **Decoupled LCD Driver**:
    -   **HD44780**: A generic, protocol-agnostic driver that manages the LCD's command set and logic. It is fully portable to any hardware.
    -   **PCF8574**: A hardware-specific driver that handles I2C communication with the popular LCD backpack.
    -   **LCD Facade**: A simple, top-level API (`lcd.h`) that makes the entire subsystem easy to use in the main application.
-   **UART**: Transmits human-readable data (115200 baud, 8N1) for monitoring on a PC.
-   **Error Handling**: Uses enum-based result codes to report errors via UART and a blinking onboard LED.
-   **Timing**: Employs a SysTick-based timer for precise delays and timeouts.
-   **Modular Design**: The project is structured with a clear separation between hardware-agnostic drivers, MCU peripheral drivers, and the main application logic for maximum reusability.

## Hardware Requirements

-   **MCU Board**: WeAct "Blue Pill Plus" (STM32F103CBT6) or a similar clone.
-   **Sensor**: GY-BME280 breakout board.
-   **Display**: 16x2 LCD with a PCF8574 I2C backpack.
-   **Serial Adapter**: USB-to-UART (TTL level) adapter for viewing serial output and/or flashing.
-   **Crystal**: The board must have an 8MHz HSE crystal populated.

## Pinout and Wiring

### I2C Wiring (Default Setup)

This is the default configuration for the project. The BME280 and LCD share the same I2C bus.

-   **BME280 Sensor:**
    -   `SCL` -> `PB6` (I2C1 SCL)
    -   `SDA` -> `PB7` (I2C1 SDA)
    -   `VCC` -> `3.3V`
    -   `GND` -> `GND`
    -   `CSB` -> `3.3V` (Selects I2C mode on the BME280 board)
    -   `SDO` -> `GND` (Sets the I2C address to `0x76`)

-   **LCD with I2C Backpack:**
    -   `SCL` -> `PB6` (Shared with BME280)
    -   `SDA` -> `PB7` (Shared with BME280)
    -   `VCC` -> `5V`
    -   `GND` -> `GND`

### SPI Wiring (Alternative)

To use this, you must build the firmware with the `BME280_PROTOCOL=SPI` option.

-   **BME280 Sensor:**
    -   `SCL`  -> `PA5` (SPI1 SCLK)
    -   `SDA`  -> `PA7` (SPI1 MOSI)
    -   `SDO`  -> `PA6` (SPI1 MISO)
    -   `CSB`  -> `PA4` (SPI1 CS)
    -   `VCC`  -> `3.3V`
    -   `GND`  -> `GND`

### Serial (UART) for Monitoring

Required for viewing sensor data on a computer.

-   **USB-to-UART Adapter:**
    -   `RX` -> `PA9` (STM32 TX)
    -   `TX` -> `PA10` (STM32 RX)
    -   `GND` -> `GND`

## Software Requirements

-   **Toolchain**: GNU Arm Embedded Toolchain (`arm-none-eabi-gcc`).
-   **Build System**: GNU Make.
-   **Flashing Tools**: `stm32flash` (for UART) or `st-flash` (for ST-Link).
-   **Serial Terminal**: A terminal emulator like `minicom`, `PuTTY`, or `screen`.

## Build Configuration

The `Makefile` allows customization of the BME280 communication protocol and operating mode.

-   `BME280_PROTOCOL`: Sets the communication interface.
    -   `I2C` (default)
    -   `SPI`
-   `BME280_SDO`: Sets the I2C address when `BME280_PROTOCOL=I2C`.
    -   `LOW` (Address `0x76`, default)
    -   `HIGH` (Address `0x77`)
-   `BME280_MODE`: Sets the sensor's measurement mode.
    -   `FORCED` (On-demand measurements, default)
    -   `NORMAL` (Continuous measurements)

To override the defaults, pass the variables on the command line:
```bash
# Build to use SPI in Normal mode
make BME280_PROTOCOL=SPI BME280_MODE=NORMAL
```

## Build Instructions

1.  **Clone the Repository:**
    ```bash
    git clone --recurse-submodules https://github.com/YerimB/stm32f103cb-bme280.git
    cd stm32f103cb-bme280
    ```
    *(The `--recurse-submodules` flag is needed to pull in the STM32-base dependency).*

2.  **Install Toolchain:**
    Ensure the `arm-none-eabi-` toolchain and `make` are installed and available in your system's PATH.

3.  **Build the Firmware:**
    ```bash
    make
    ```
    This generates `stm32_executable.elf` and `stm32_bin_image.bin` in the `bin/` directory.

4.  **Other Make Commands:**
    -   `make clean`: Removes generated object and binary files.
    -   `make re`: Performs a `clean` followed by a full `build`.

## Flashing the Firmware

### Option 1: Using the UART Bootloader

1.  Connect your USB-to-UART adapter to pins `PA9` (TX) and `PA10` (RX).
2.  Set the `BOOT0` jumper to the `1` (High) position and `BOOT1` to `0` (Low).
3.  Reset the board to enter the factory bootloader mode.
4.  Update the `FLASHING_SERIAL_PORT` variable in the `Makefile` to match your system's serial port (e.g., `/dev/ttyUSB0` or `COM3`).
5.  Run the flash command:
    ```bash
    make flash FLASHING_TOOL=stm32flash
    ```
6.  Once flashing is complete, return the `BOOT0` jumper to the `0` position and reset the board to run the new application.

### Option 2: Using an ST-Link

1.  Connect the ST-Link programmer to the SWD pins (`SWDIO`, `SWCLK`, `GND`) of the STM32 board.
2.  Ensure `st-flash` (part of the [stlink-tools](https://github.com/stlink-org/stlink) package) is installed.
3.  Run the flash command:
    ```bash
    make flash FLASHING_TOOL=st-flash
    ```
    *(This command explicitly tells the Makefile to use the `st-flash` utility).*

## Usage

1.  Power the board and connect a UART terminal to the serial port at **115200 baud**.
2.  On startup, a welcome message is displayed on the LCD and sent via UART.
3.  The main loop begins, taking measurements based on the configured mode:
    -   **Forced Mode**: A new measurement can be triggered by pressing the user button (`KEY`).
    -   **Normal Mode**: The sensor measures continuously, and data is read every 30 seconds.
4.  The LCD displays the data in a compact format:
    ```
    T:25.3°C H:60%
    P:1013.25 hPa
    ```
5.  The UART terminal shows more detailed output:
    ```
    Temperature: 25.30 °C, Pressure: 1013.25 hPa, Humidity: 60.0 %
    ```
6.  If an error occurs (e.g., sensor not found), the onboard LED on pin `PB2` will blink a pattern corresponding to the error code, and a message will be printed to the UART.

---

### License

This project is licensed under the MIT License. See the `LICENSE` file for details.
