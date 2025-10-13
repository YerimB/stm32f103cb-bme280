#ifndef I2C_H
#define I2C_H

#include "stm32f103xb.h"
#include "utils/result.h"

#define I2C_MAX_FREQUENCY_Sm 100000                                                 // 100kHz
#define I2C_TIMEOUT_MS 100                                                          // 100ms
#define I2C_BUS_ERROR_MASK (I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_AF | I2C_SR1_OVR) // Mask for detectable hardware errors (SR1 register)

typedef struct I2C_ops_params
{
    uint8_t restart;
    uint8_t generate_stop;
} I2C_ops_params_t;

Result i2c1_init(int remap);
Result i2c2_init(void);
Result i2c_write(I2C_TypeDef *i2c, const uint8_t slave_addr, const uint8_t *data, const uint32_t size, const I2C_ops_params_t params);
Result i2c_read(I2C_TypeDef *i2c, const uint8_t slave_addr, uint8_t *data, const uint32_t size, const I2C_ops_params_t params);

#endif // I2C_H
