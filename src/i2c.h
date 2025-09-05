#ifndef I2C_H
#define I2C_H

#include "stm32f103xb.h"
#include "result.h"

#define NULL ((void *)0)

#define I2C_MAX_FREQUENCY_Sm 100000 // 100kHz
#define I2C_TIMEOUT_MS 100          // 100ms

Result i2c1_init(int remap);
Result i2c1_write(const uint8_t slave_addr, const uint8_t *data, const uint32_t size, const uint8_t wait_for_bus, const uint8_t gen_stop);
Result i2c1_read(const uint8_t slave_addr, uint8_t *data, const uint32_t size, const uint8_t wait_for_bus, const uint8_t gen_stop);

#endif // I2C_H
