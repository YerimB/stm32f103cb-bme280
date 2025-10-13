#ifndef LCD_I2C_PCF8574_H_
#define LCD_I2C_PCF8574_H_

#include "stm32f103xb.h"
#include "mcu/clock.h"
#include "mcu/i2c.h"
#include "utils/result.h"
#include "lcd/hd44780.h"

#define PCF8574_I2C_ADDR 0x27

// These masks define the physical pin mapping on the I2C backpack.
// They control which bit in the I2C byte goes to which LCD pin.
#define LCD_PIN_RS 0x01        // Register Select is on P0
#define LCD_PIN_RW 0x02        // Read/Write is on P1
#define LCD_PIN_EN 0x04        // Enable is on P2
#define LCD_PIN_BACKLIGHT 0x08 // Backlight is on P3

// Convenience macros for the above pins
#define LCD_RS_DATA LCD_PIN_RS
#define LCD_RS_CMD 0x00
#define LCD_RW_READ LCD_PIN_RW
#define LCD_RW_WRITE 0x00
#define LCD_E_ENABLE LCD_PIN_EN
#define LCD_E_DISABLE 0x00
#define LCD_BACKLIGHT_ON LCD_PIN_BACKLIGHT
#define LCD_BACKLIGHT_OFF 0x00

Result pcf8574_set_i2c_instance(I2C_TypeDef *i2c_instance);
Result pcf8574_set_backlight(const uint8_t on);
const hd44780_interface_t *pcf8574_get_hd44780_interface(void);

#endif // LCD_I2C_PCF8574_H_
