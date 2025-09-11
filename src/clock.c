#include "clock.h"

static volatile uint32_t systick_count = 0;

#ifdef __cplusplus
extern "C"
{
#endif
    void SysTick_Handler(void)
    {
        systick_count++;
    }
#ifdef __cplusplus
}
#endif

uint32_t get_systick_count(void)
{
    return systick_count;
}

static uint32_t min(const uint32_t a, const uint32_t b)
{
    return a > b ? b : a;
}

// See stm32f103c8 reference manual - chapter 8 (https://www.st.com/en/microcontrollers-microprocessors/stm32f103c8.html#documentation)
uint32_t sysclk_frequency(void)
{
    uint32_t sysclk_hz = HSI_hz; // Default is HSI frequency
    uint32_t hse_status = 0;

    // Check current clock source (SWS bits in CFGR, bits 3:2)
    uint32_t sws = (RCC->CFGR >> 2) & 0x3;

    if (sws == 0x2 /* PLL used as system clock */)
    {
        // Get PLL multiplication factor (PLLMUL, bits 21:18)
        uint32_t pllmul = min(((RCC->CFGR >> 18) & 0xF) + 2, 0x10); // value range is 2x to 16x
        // Check PLL source (PLLSRC, bit 16)
        uint32_t pllsrc = (RCC->CFGR >> 16) & 0x1;

        if (pllsrc == 0) /* PLL source is HSI/2 */
        {
            sysclk_hz = (HSI_hz / 2) * pllmul;
        }
        else /* PLL source is HSE */
        {
            hse_status = (RCC->CR >> 17) & 0x1; // HSE ready flag
            if (hse_status)
            {
                sysclk_hz = HSE_hz * pllmul;
            }
        }
    }
    else if (sws == 0x1 /* HSE used as system clock */)
    {
        hse_status = (RCC->CR >> 17) & 0x1;
        if (hse_status)
        {
            sysclk_hz = HSE_hz;
        }
    } // Else, Default is HSI oscillator

    return sysclk_hz;
}

uint32_t cpu_clock_frequency(void)
{
    const uint32_t ahb_prescaler = (RCC->CFGR >> 4) & 15;

    return sysclk_frequency() / HPRE_to_prescaler_div_factor(ahb_prescaler);
}

// See Arm Cortex M3 reference manual - chapter 4 (https://developer.arm.com/documentation/ddi0337/h?_ga=2.258143811.839925519.1629395464-2030874199.1629395464)
void systick_init()
{
    // Calculate reload value for 1 ms tick
    uint32_t reload_value = (sysclk_frequency() / 1000) - 1; // Ticks per ms

    SysTick->LOAD = reload_value;
    SysTick->VAL = 0; // Reset timer
    // SysTick_CTRL_ENABLE_Msk to start timer & SysTick_CTRL_CLKSOURCE_Msk to use the CPU main clock
    SysTick->CTRL = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk;
}

// Precise delay function
void delay_ms(const uint32_t ms)
{
    for (uint32_t i = 0; i < ms; i++)
    {
        // Wait until the COUNTFLAG is set (timer reached zero)
        while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk))
            ;
    }
}

uint32_t PPRE_to_prescaler_div_factor(const uint32_t ppre)
{
    switch (ppre)
    {
    case 4:
        return 2;
    case 5:
        return 4;
    case 6:
        return 8;
    case 7:
        return 16;
    default:
        return 1;
    }
}

uint32_t HPRE_to_prescaler_div_factor(const uint32_t hpre)
{
    switch (hpre)
    {
    case 8:
        return 2;
    case 9:
        return 4;
    case 10:
        return 8;
    case 11:
        return 16;
    case 12:
        return 64;
    case 13:
        return 128;
    case 14:
        return 256;
    case 15:
        return 512;
    default:
        return 1;
    }
}
