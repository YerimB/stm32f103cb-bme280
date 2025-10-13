#include "lcd/hd44780.h"

#define UPPER_NIBBLE_Msk 0xF0U

static const hd44780_interface_t *s_interface = 0x0;

#define _S_INTERFACE_CHECK()                  \
    do                                        \
    {                                         \
        if (!s_interface)                     \
            return HD44780_MISSING_INTERFACE; \
    } while (0)

static Result _hd44780_send_nibble(const hd44780_rs_t rs, const uint8_t nibble);
static Result _hd44780_send_byte(const hd44780_rs_t rs, const uint8_t byte);

Result hd44780_init(const hd44780_interface_t *interface)
{
    if (!interface || !interface->delay_ms || !interface->send_nibbles)
        return INVALID_PARAMETER;
    s_interface = interface;
    return OK;
}

Result hd44780_reset(void)
{
    _S_INTERFACE_CHECK();
    s_interface->delay_ms(50); // Wait for power up (> 40ms)
    // Set DL 3 times as recommended in datasheet
    OK_OR_PROPAGATE(_hd44780_send_nibble(HD44780_RS_CMD, HD44780_FN_SET_8_BIT));
    s_interface->delay_ms(5); // Wait for more than 4.1ms (datasheet)
    OK_OR_PROPAGATE(_hd44780_send_nibble(HD44780_RS_CMD, HD44780_FN_SET_8_BIT));
    s_interface->delay_ms(1); // Wait for more than 100µs
    OK_OR_PROPAGATE(_hd44780_send_nibble(HD44780_RS_CMD, HD44780_FN_SET_8_BIT));
    s_interface->delay_ms(1);
    // Unset DL => 4-bit mode
    OK_OR_PROPAGATE(_hd44780_send_nibble(HD44780_RS_CMD, HD44780_FN_SET_4_BIT));
    s_interface->delay_ms(5);

    // Display initialisation
    // Function set: keep DL=0 (4 bit mode), N=1 (2 line display), F=0 (5x8 font)
    OK_OR_PROPAGATE(_hd44780_send_byte(HD44780_RS_CMD, HD44780_FN_SET_4_BIT | HD44780_FN_SET_2_LINES | HD44780_FN_SET_5x8_DOTS));
    // Display off: D=0,C=0,B=0 (display off, cursor off, blink off)
    OK_OR_PROPAGATE(_hd44780_send_byte(HD44780_RS_CMD, HD44780_DISPLAY_OFF));
    // Display clear
    OK_OR_PROPAGATE(hd44780_clear_display());
    // Entry mode set: I/D=1,S=0 (increment, no shift)
    OK_OR_PROPAGATE(_hd44780_send_byte(HD44780_RS_CMD, HD44780_ENTRY_MODE_SET_INCREMENT | HD44780_ENTRY_MODE_SET_SHIFT_OFF));
    // Turn on display
    OK_OR_PROPAGATE(hd44780_display_control((hd44780_dc_t){.display_on = 1, .cursor_on = 0, .blink_on = 0}));

    return OK;
}

Result hd44780_display_control(const hd44780_dc_t dc)
{
    const uint8_t cmd = (dc.display_on ? HD44780_DISPLAY_ON : HD44780_DISPLAY_OFF) |
                        (dc.cursor_on ? HD44780_CURSOR_ON : HD44780_CURSOR_OFF) |
                        (dc.blink_on ? HD44780_BLINK_ON : HD44780_BLINK_OFF);

    return _hd44780_send_byte(HD44780_RS_CMD, cmd);
}

Result hd44780_clear_display(void)
{
    OK_OR_PROPAGATE(_hd44780_send_byte(HD44780_RS_CMD, HD44780_CMD_CLEAR_DISPLAY));
    // Safe to use `s_interface` after `_hd44780_send_byte`
    s_interface->delay_ms(2); // > 1.52ms delay (max exec time)

    return OK;
}

Result hd44780_return_home(void)
{
    OK_OR_PROPAGATE(_hd44780_send_byte(HD44780_RS_CMD, HD44780_CMD_RETURN_HOME));
    // Safe to use `s_interface` after `_hd44780_send_byte`
    s_interface->delay_ms(2); // > 1.52ms delay (max exec time)

    return OK;
}

Result hd44780_set_cursor(const uint8_t row, const uint8_t col)
{
    if (col > 15 || row > 1)
        return LCD_INVALID_POS; // Invalid position

    const uint8_t pos = col | (row & 0x1 ? 0xC0 : 0x80); // TODO: defines

    return _hd44780_send_byte(HD44780_RS_CMD, pos);
}

Result hd44780_putchar(const char c)
{
    return _hd44780_send_byte(HD44780_RS_DATA, c);
}

Result hd44780_putstr(const char *str)
{
    while (*str)
    {
        OK_OR_PROPAGATE(hd44780_putchar(*str++));
    }

    return OK;
}

Result hd44780_putint(int32_t n)
{
    if (n == 0)
        return _hd44780_send_byte(HD44780_RS_DATA, '0');

    char buffer[11];
    uint8_t i = 0;

    if (n >> 31) // Negative check
    {
        OK_OR_PROPAGATE(_hd44780_send_byte(HD44780_RS_DATA, '-'));
        n = -n; // Make n positive
    }

    while (n > 0)
    {
        buffer[i++] = (n % 10) + '0';
        n /= 10;
    }
    while (i > 0)
    {
        OK_OR_PROPAGATE(_hd44780_send_byte(HD44780_RS_DATA, buffer[--i]));
    }

    return OK;
}

static Result _hd44780_send_nibble(const hd44780_rs_t rs, const uint8_t nibble)
{
    _S_INTERFACE_CHECK();

    const uint8_t data = nibble & UPPER_NIBBLE_Msk;
    return s_interface->send_nibbles(rs, &data, 1);
}

static Result _hd44780_send_byte(const hd44780_rs_t rs, const uint8_t byte)
{
    _S_INTERFACE_CHECK();

    uint8_t data[2];
    data[0] = byte & UPPER_NIBBLE_Msk; // Upper nibble mask
    data[1] = byte << 4U;              // Lower nibble shifted (no mask needed)
    return s_interface->send_nibbles(rs, data, 2);
}
