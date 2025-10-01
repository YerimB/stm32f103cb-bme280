# Use globally installed Arm GNU Toolchain
TOOLCHAIN_PATH =
CXXFLAGS += -Os # Optimize for binary size
USE_ST_CMSIS = true

DEVICE = STM32F103x8
FLASH = 0x08000000
FLASHING_TOOL ?= stm32flash
FLASHING_SERIAL_PORT ?= /dev/ttyUSB0 -b 115200

BME280_PROTOCOL ?= I2C
ifeq ($(filter $(BME280_PROTOCOL),I2C SPI),)
    $(error Unsupported BME280_PROTOCOL '$(BME280_PROTOCOL)'. Use 'I2C' or 'SPI')
endif
CPPFLAGS += -D BME280_USE_$(BME280_PROTOCOL) # Defines protocol used to communicate with BME280 sensor

ifeq ($(BME280_PROTOCOL),I2C)
    # Set BME280_SDO pin level (LOW or HIGH)
    BME280_SDO ?= LOW
    ifeq ($(filter $(BME280_SDO),LOW HIGH),)
        $(error Unsupported BME280_SDO '$(BME280_SDO)'. Use 'LOW' or 'HIGH')
    endif
    CPPFLAGS += -D BME280_SDO_$(BME280_SDO)
endif

BME280_MODE ?= FORCED
ifeq ($(filter $(BME280_MODE),NORMAL FORCED),)
    $(error Unsupported BME280_MODE '$(BME280_MODE)'. Use 'normal' or 'forced')
endif
CPPFLAGS += -D BME280_$(BME280_MODE)_MODE

# Include the main makefile
include ./STM32-base/make/common.mk

re: clean all

.PHONY: all clean flash re
