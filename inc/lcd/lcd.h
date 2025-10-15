#ifndef LCD_H_
#define LCD_H_

#include "lcd/hd44780.h"
#include "lcd/pcf8574.h"

Result lcd_initialize(void);

#define lcd_set_i2c_instance(i2c) pcf8574_set_i2c_instance(i2c)
#define lcd_set_backlight(on) pcf8574_set_backlight(on)
#define lcd_set_cursor_pos(r, c) hd44780_set_cursor(r, c)
#define lcd_clear() hd44780_clear_display()
#define lcd_putchar(c) hd44780_putchar(c)
#define lcd_putstr(s) hd44780_putstr(s)
#define lcd_putint(n) hd44780_putint(n)
#define lcd_create_custom_char(pos, charmap) hd44780_add_custom_char(pos, charmap)

#endif // LCD_H_
