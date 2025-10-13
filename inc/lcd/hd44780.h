#ifndef HD44780_H_
#define HD44780_H_

#include "stm32f103xb.h"
#include "utils/result.h"

// -- Commands --
#define HD44780_CMD_CLEAR_DISPLAY 0x01
#define HD44780_CMD_RETURN_HOME 0x02

// -- Entry Mode Set --
#define HD44780_ENTRY_MODE_SET_CTRL 0x04
#define HD44780_ENTRY_MODE_SET_DECREMENT HD44780_ENTRY_MODE_SET_CTRL
#define HD44780_ENTRY_MODE_SET_INCREMENT (HD44780_ENTRY_MODE_SET_CTRL | 0x02)
#define HD44780_ENTRY_MODE_SET_SHIFT_OFF HD44780_ENTRY_MODE_SET_CTRL
#define HD44780_ENTRY_MODE_SET_SHIFT_ON (HD44780_ENTRY_MODE_SET_CTRL | 0x01)

// -- Display On/Off Control --
#define HD44780_CMD_DISPLAY_CTRL 0x08
#define HD44780_DISPLAY_OFF HD44780_CMD_DISPLAY_CTRL
#define HD44780_DISPLAY_ON (HD44780_CMD_DISPLAY_CTRL | 0x04)
#define HD44780_CURSOR_OFF HD44780_CMD_DISPLAY_CTRL
#define HD44780_CURSOR_ON (HD44780_CMD_DISPLAY_CTRL | 0x06)
#define HD44780_BLINK_OFF HD44780_CMD_DISPLAY_CTRL
#define HD44780_BLINK_ON (HD44780_CMD_DISPLAY_CTRL | 0x05)

// -- Function Set --
#define HD44780_FN_SET_CTRL 0x20
#define HD44780_FN_SET_4_BIT HD44780_FN_SET_CTRL
#define HD44780_FN_SET_8_BIT (HD44780_FN_SET_CTRL | 0x10)
#define HD44780_FN_SET_1_LINE HD44780_FN_SET_CTRL
#define HD44780_FN_SET_2_LINES (HD44780_FN_SET_CTRL | 0x08)
#define HD44780_FN_SET_5x8_DOTS HD44780_FN_SET_CTRL
#define HD44780_FN_SET_5x10_DOTS (HD44780_FN_SET_CTRL | 0x04)

typedef enum hd44780_rs_s
{
    HD44780_RS_CMD = 0,
    HD44780_RS_DATA = 1
} hd44780_rs_t;

typedef struct hd44780_display_control_s
{
    uint8_t display_on : 1;
    uint8_t cursor_on : 1;
    uint8_t blink_on : 1;
} hd44780_dc_t;

typedef struct hd44780_interface_s
{
    Result (*send_nibbles)(const hd44780_rs_t rs, const uint8_t *nibbles, const uint32_t size);
    void (*delay_ms)(const uint32_t ms);
} hd44780_interface_t;

Result hd44780_init(const hd44780_interface_t *interface);
Result hd44780_reset(void);
Result hd44780_display_control(const hd44780_dc_t dc);
Result hd44780_clear_display(void);
Result hd44780_return_home(void);
Result hd44780_set_cursor(const uint8_t row, const uint8_t col);
Result hd44780_putchar(const char c);
Result hd44780_putstr(const char *s);
Result hd44780_putint(int32_t n);

#endif // HD44780_H_
