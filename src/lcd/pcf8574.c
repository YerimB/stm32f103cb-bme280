#include "lcd/pcf8574.h"

static Result _hd44780_send_nibbles(const hd44780_rs_t rs, const uint8_t *nibbles, const uint32_t size);

static I2C_TypeDef *s_i2c = I2C2;                     // Default value: I2C2
static uint8_t s_backlight_state = LCD_BACKLIGHT_OFF; // Default value: OFF
static const hd44780_interface_t s_hd44780_interface = (hd44780_interface_t){.send_nibbles = _hd44780_send_nibbles, .delay_ms = delay_ms};

Result pcf8574_set_i2c_instance(I2C_TypeDef *i2c_instance)
{
    if (!i2c_instance)
        return INVALID_PARAMETER;
    s_i2c = i2c_instance;
    return OK;
}

Result pcf8574_set_backlight(const uint8_t on)
{
    const uint8_t data = on ? LCD_BACKLIGHT_ON : LCD_BACKLIGHT_OFF;

    s_backlight_state = data;
    return i2c_write(s_i2c, PCF8574_I2C_ADDR, &data, 1, (I2C_ops_params_t){0, 1});
}

const hd44780_interface_t *pcf8574_get_hd44780_interface(void)
{
    return &s_hd44780_interface;
}

static Result _hd44780_send_nibbles(const hd44780_rs_t rs, const uint8_t *nibbles, const uint32_t size)
{
    if (size == 0)
        return OK;

    uint8_t i2c_payload[size * 2];
    uint8_t rs_mask = (rs == HD44780_RS_DATA) ? LCD_RS_DATA : LCD_RS_CMD;

    // Building the I2C payload
    for (uint32_t i = 0; i < size; ++i)
    {
        uint8_t data_nibble = nibbles[i] & 0xF0;

        i2c_payload[i * 2] = data_nibble | s_backlight_state | LCD_E_ENABLE | LCD_RW_WRITE | rs_mask;
        i2c_payload[i * 2 + 1] = data_nibble | s_backlight_state | LCD_E_DISABLE | LCD_RW_WRITE | rs_mask;
    }

    return i2c_write(
        s_i2c,
        PCF8574_I2C_ADDR,
        i2c_payload,
        size * 2,
        (I2C_ops_params_t){.restart = 0, .generate_stop = 1});
}
