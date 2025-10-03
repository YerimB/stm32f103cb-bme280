#ifndef USER_BUTTON_H
#define USER_BUTTON_H

#include "stm32f103xb.h"

#define DEBOUNCE_US 20000U

typedef void (*EXTI_callback)(const uint16_t);

typedef struct _user_btn_trigger_config
{
    uint8_t enable_rising : 1;
    uint8_t enable_falling : 1;
    EXTI_callback callback;
} user_btn_trigger_config_t;

void enable_user_button(const user_btn_trigger_config_t);
void disable_user_button(void);

#endif // USER_BUTTON_H