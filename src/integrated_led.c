#include "integrated_led.h"

void integrated_led_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    GPIOB->CRL &= ~(0xF << GPIO_CRL_MODE2_Pos);
    GPIOB->CRL |= GPIO_CRL_MODE2_1;
}

void integrated_led_switch(const uint8_t status)
{
    GPIOB->BSRR |= status ? GPIO_BSRR_BS2 : GPIO_BSRR_BR2;
}

void integrated_led_error_blink(const uint32_t err)
{
    const uint32_t min_loops = 800000;

    while (1)
    {
        integrated_led_switch(0);
        for (volatile uint32_t i = 2 * min_loops; i > 0; --i)
            ;
        integrated_led_switch(1);
        for (volatile uint32_t i = 4 * min_loops; i > 0; --i)
            ;
        integrated_led_switch(0);
        for (volatile uint32_t i = 2 * min_loops; i > 0; --i)
            ;

        for (uint32_t i = err; i > 0; --i)
        {
            integrated_led_switch(1);
            for (volatile uint32_t i = min_loops; i > 0; --i)
                ;
            integrated_led_switch(0);
            for (volatile uint32_t i = min_loops; i > 0; --i)
                ;
        }
    }
}
