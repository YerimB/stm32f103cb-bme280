DEVICE          = STM32F103x8
FLASH           = 0x08000000
FLASHING_TOOL = stm32flash
FLASHING_SERIAL_PORT = /dev/ttyUSB0 -b 115200

CXXFLAGS += -Os
USE_ST_CMSIS    = true

# Use globally installed Arm GNU Toolchain
TOOLCHAIN_PATH  =

MODE ?= forced

ifeq ($(filter $(MODE),normal forced),)
    $(error Unsupported MODE '$(MODE)'. Use 'normal' or 'forced')
endif

ifeq ($(MODE),normal)
	CPPFLAGS += -D BME280_NORMAL_MODE
else # MODE=forced
	CPPFLAGS += -D BME280_FORCED_MODE
endif

# Include the main makefile
include ./STM32-base/make/common.mk

re: clean all

.PHONY: all clean flash re
