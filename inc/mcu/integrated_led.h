
#ifndef INTEGRATED_LED_H
#define INTEGRATED_LED_H

#include "stm32f103xb.h"
#include "utils/result.h"

void integrated_led_init(void);
void integrated_led_switch(const uint8_t status);
void integrated_led_error_blink(const uint32_t err);

#endif // INTEGRATED_LED_H
