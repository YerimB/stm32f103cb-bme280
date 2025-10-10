#include "lcd1602.h"
#include "uart.h"

#define UPPER_NIBBLE_Msk 0xF0

static I2C_TypeDef *lcd1602_i2c = I2C2;                  // Default value
static uint8_t g_backlight_state = LCD1602_BACKLIGHT_ON; // Default value

static Result lcd1602_send_nibble(const uint8_t nibble, const uint8_t rs)
{
    const uint8_t data = nibble & UPPER_NIBBLE_Msk;

    uint8_t data_t[2];
    data_t[0] = data | g_backlight_state | LCD1602_E_ENABLE | LCD1602_RW_WRITE | rs;
    data_t[1] = data | g_backlight_state | LCD1602_E_DISABLE | LCD1602_RW_WRITE | rs;

    return i2c_write(lcd1602_i2c, LCD1602_I2C_ADDR, data_t, 2, (I2C_ops_params_t){0, 1});
}

static Result lcd1602_send_byte(const uint8_t byte, const uint8_t rs)
{
    const uint8_t data_upper_nibble = byte & UPPER_NIBBLE_Msk;
    const uint8_t data_lower_nibble = byte << 4;
    const uint8_t rs_val = rs & LCD1602_RS_Msk;

    uint8_t data_t[4];
    data_t[0] = data_upper_nibble | g_backlight_state | LCD1602_E_ENABLE | LCD1602_RW_WRITE | rs_val;
    data_t[1] = data_upper_nibble | g_backlight_state | LCD1602_E_DISABLE | LCD1602_RW_WRITE | rs_val;
    data_t[2] = data_lower_nibble | g_backlight_state | LCD1602_E_ENABLE | LCD1602_RW_WRITE | rs_val;
    data_t[3] = data_lower_nibble | g_backlight_state | LCD1602_E_DISABLE | LCD1602_RW_WRITE | rs_val;

    return i2c_write(lcd1602_i2c, LCD1602_I2C_ADDR, data_t, 4, (I2C_ops_params_t){0, 1});
}

/**
 * Sets the I2C peripheral instance used by the LCD1602 driver.
 *
 * This function allows the user to specify which I2C peripheral instance
 * the LCD1602 driver should use for communication. If a valid (non-NULL)
 * pointer is provided, the internal reference will be updated to use the
 * specified I2C instance. By default, the driver uses I2C2.
 *
 * @param i2c_instance Pointer to the I2C peripheral instance to use (e.g., I2C1, I2C2).
 */
void lcd1602_set_i2c_instance(I2C_TypeDef *i2c_instance)
{
    if (i2c_instance)
        lcd1602_i2c = i2c_instance;
}

// 4-bit interface initialisation (see datasheet fig. 24, https://cdn.sparkfun.com/assets/9/5/f/7/b/HD44780.pdf)
Result lcd1602_init(void)
{
    delay_ms(50); // Wait for power up (> 40ms)

    // Set DL 3 times as recommended in datasheet
    OK_OR_PROPAGATE(lcd1602_send_nibble(LCD1602_CMD_FN_SET_8_BIT, LCD1602_RS_CMD));
    delay_ms(5); // Wait for more than 4.1ms (datasheet)
    OK_OR_PROPAGATE(lcd1602_send_nibble(LCD1602_CMD_FN_SET_8_BIT, LCD1602_RS_CMD));
    delay_ms(1); // Wait for more than 100µs
    OK_OR_PROPAGATE(lcd1602_send_nibble(LCD1602_CMD_FN_SET_8_BIT, LCD1602_RS_CMD));
    delay_ms(1);
    // Unset DL => 4-bit mode
    OK_OR_PROPAGATE(lcd1602_send_nibble(LCD1602_CMD_FN_SET_4_BIT, LCD1602_RS_CMD));
    delay_ms(5);

    // Display initialisation
    // Function set: keep DL=0 (4 bit mode), N=1 (2 line display), F=0 (5x8 font)
    OK_OR_PROPAGATE(lcd1602_send_byte(LCD1602_CMD_FN_SET_4_BIT | LCD1602_CMD_FN_SET_2_LINES | LCD1602_CMD_FN_SET_5x8_DOTS, LCD1602_RS_CMD));
    // Display off: D=0,C=0,B=0 (display off, cursor off, blink off)
    OK_OR_PROPAGATE(lcd1602_send_byte(LCD1602_CMD_DISPLAY_OFF, LCD1602_RS_CMD));
    // Display clear
    OK_OR_PROPAGATE(lcd1602_clear());
    // Entry mode set: I/D=1,S=0 (increment, no shift)
    OK_OR_PROPAGATE(lcd1602_send_byte(LCD1602_CMD_ENTRY_MODE_SET_INCREMENT | LCD1602_CMD_ENTRY_MODE_SET_SHIFT_OFF, LCD1602_RS_CMD));
    // Display on: D=1,C=0,B=0 (display on, cursor off, blink off)
    OK_OR_PROPAGATE(lcd1602_send_byte(LCD1602_CMD_DISPLAY_ON, LCD1602_RS_CMD));
    return OK;
}

Result lcd1602_set_backlight(const uint8_t state)
{
    const uint8_t data = state ? LCD1602_BACKLIGHT_ON : LCD1602_BACKLIGHT_OFF;

    g_backlight_state = data;
    return i2c_write(lcd1602_i2c, LCD1602_I2C_ADDR, &data, 1, (I2C_ops_params_t){0, 1});
}

Result lcd1602_clear(void)
{
    OK_OR_PROPAGATE(lcd1602_send_byte(LCD1602_CMD_CLEAR_DISPLAY, LCD1602_RS_CMD));
    delay_ms(2); // Wait for clear to complete (~1.52ms)
    return OK;
}

Result lcd1602_put_cursor(const uint8_t row, const uint8_t col)
{
    if (col > 15 || row > 1)
        return LCD1602_INVALID_POS; // Invalid position

    const uint8_t pos = col | (row & 0x1 ? 0xC0 : 0x80);

    return lcd1602_send_byte(pos, LCD1602_RS_CMD);
}

Result lcd1602_put_str(const char *str)
{
    while (*str)
    {
        OK_OR_PROPAGATE(lcd1602_send_byte(*str++, LCD1602_RS_DATA));
    }

    return OK;
}

Result lcd1602_put_int(int32_t n)
{
    if (n == 0)
        return lcd1602_send_byte('0', LCD1602_RS_DATA);

    char buffer[11];
    uint8_t i = 0;

    if (n >> 31) // Negative check
    {
        OK_OR_PROPAGATE(lcd1602_send_byte('-', LCD1602_RS_DATA));
        n = -n; // Make n positive
    }

    while (n > 0)
    {
        buffer[i++] = (n % 10) + '0';
        n /= 10;
    }
    while (i > 0)
    {
        OK_OR_PROPAGATE(lcd1602_send_byte(buffer[--i], LCD1602_RS_DATA));
    }

    return OK;
}
