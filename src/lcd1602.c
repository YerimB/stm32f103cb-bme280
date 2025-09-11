#include "lcd1602.h"
#include "uart.h"

Result lcd1602_send_cmd(const uint8_t cmd)
{
    uint8_t data_u, data_l;
    uint8_t data_t[4];

    data_u = (cmd & 0xf0);
    data_l = ((cmd << 4) & 0xf0);
    data_t[0] = data_u | 0x0C; // en=1, rs=0
    data_t[1] = data_u | 0x08; // en=0, rs=0
    data_t[2] = data_l | 0x0C; // en=1, rs=0
    data_t[3] = data_l | 0x08; // en=0, rs=0

    return i2c_write(I2C2, LCD1602_I2C_ADDR, data_t, 4, (I2C_ops_params_t){0, 1});
}

Result lcd1602_send_data(const char data)
{
    uint8_t data_u, data_l;
    uint8_t data_t[4];

    data_u = (data & 0xf0);
    data_l = ((data << 4) & 0xf0);
    data_t[0] = data_u | 0x0D; // en=1, rs=1
    data_t[1] = data_u | 0x09; // en=0, rs=1
    data_t[2] = data_l | 0x0D; // en=1, rs=1
    data_t[3] = data_l | 0x09; // en=0, rs=1

    return i2c_write(I2C2, LCD1602_I2C_ADDR, data_t, 4, (I2C_ops_params_t){0, 1});
}

Result lcd1602_set_backlight(const uint8_t state)
{
    const uint8_t data = state ? LCD1602_BACKLIGHT_ON : LCD1602_BACKLIGHT_OFF;

    return i2c_write(I2C2, LCD1602_I2C_ADDR, &data, 1, (I2C_ops_params_t){0, 1});
}

Result lcd1602_clear(void)
{
    Result r = lcd1602_send_cmd(LCD1602_CMD_CLEAR_DISPLAY);

    if (r != OK)
        return r;
    delay_ms(2); // Wait for clear to complete (~1.52ms)
    return OK;
}

Result lcd1602_put_cursor(const uint8_t row, const uint8_t col)
{
    if (col > 15 || row > 1)
        return LCD1602_INVALID_POS; // Invalid position

    const uint8_t pos = col | (row & 0x1 ? 0xC0 : 0x80);

    lcd1602_send_cmd(pos);
    return OK;
}

Result lcd1602_put_str(const char *str)
{
    Result r;

    while (*str)
    {
        r = lcd1602_send_data(*str++);
        if (r != OK)
            return r;
    }

    return OK;
}

Result lcd1602_put_int(int32_t n)
{
    Result r;

    if (n == 0)
        return lcd1602_send_data('0');

    char buffer[11];
    uint8_t i = 0;

    if (n >> 31) // Negative check
    {
        r = lcd1602_send_data('-');
        if (r != OK)
            return r;
        n = -n; // Make n positive
    }

    while (n > 0)
    {
        buffer[i++] = (n % 10) + '0';
        n /= 10;
    }
    while (i > 0)
    {
        r = lcd1602_send_data(buffer[--i]);
        if (r != OK)
            return r;
    }

    return OK;
}

// 4-bit interface initialisation (see datasheet fig. 24)
// https://cdn.sparkfun.com/assets/9/5/f/7/b/HD44780.pdf
Result lcd1602_init(void)
{
    Result r;

    delay_ms(50); // Wait for power up (> 40ms)

    // Set DL 3 times as recommended in datasheet
    if ((r = lcd1602_send_cmd(LCD1602_CMD_FN_SET_8_BIT)) != OK)
        return r;
    delay_ms(5); // Wait for more than 4.1ms (datasheet)
    if ((r = lcd1602_send_cmd(LCD1602_CMD_FN_SET_8_BIT)) != OK)
        return r;
    delay_ms(1); // Wait for more than 100µs
    if ((r = lcd1602_send_cmd(LCD1602_CMD_FN_SET_8_BIT)) != OK)
        return r;
    delay_ms(10);

    // Unset DL => 4-bit mode
    if ((r = lcd1602_send_cmd(LCD1602_CMD_FN_SET_4_BIT)) != OK)
        return r;
    delay_ms(10);

    // Display initialisation

    // Function set: keep DL=0 (4 bit mode), N=1 (2 line display), F=0 (5x8 font)
    if ((r = lcd1602_send_cmd(LCD1602_CMD_FN_SET_4_BIT | LCD1602_CMD_FN_SET_2_LINES | LCD1602_CMD_FN_SET_5x8_DOTS)) != OK)
        return r;
    delay_ms(1);
    // Display off: D=0,C=0,B=0 (display off, cursor off, blink off)
    if ((r = lcd1602_send_cmd(LCD1602_CMD_DISPLAY_OFF)) != OK)
        return r;
    delay_ms(1);
    // Display clear
    if ((r = lcd1602_send_cmd(LCD1602_CMD_CLEAR_DISPLAY)) != OK)
        return r;
    delay_ms(1);
    // Entry mode set: I/D=1,S=0 (increment, no shift)
    if ((r = lcd1602_send_cmd(LCD1602_CMD_ENTRY_MODE_SET_INCREMENT | LCD1602_CMD_ENTRY_MODE_SET_SHIFT_OFF)) != OK)
        return r;
    delay_ms(1);
    // Display on: D=1,C=0,B=0 (display on, cursor off, blink off)
    if ((r = lcd1602_send_cmd(LCD1602_CMD_DISPLAY_ON)) != OK)
        return r;

    return OK;
}
