#include "lcd/lcd.h"

Result lcd_initialize(void)
{
    OK_OR_PROPAGATE(hd44780_init(pcf8574_get_hd44780_interface()));
    return hd44780_reset();
}
