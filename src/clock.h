#ifndef CLOCK_H
#define CLOCK_H

#include "stm32f103xb.h"
#include "result.h"

/* WeAct Blue Pill Plus Clone (STM32F103CBT6) values (https://stm32-base.org/boards/STM32F103C8T6-WeAct-Blue-Pill-Plus-Clone) */
#define HSI_HZ 8000000U              // 8MHz
#define HSE_HZ 8000000U              // 8MHz
#define LSI_HZ 40000U                // 40kHz
#define LSE_HZ 32768U                // 32.768kHz
#define MCU_MAX_SYSCLK_HZ 72000000UL // MCU limit = 72MHz
#define APB1_MAX_HZ 36000000U        // APB1 limit = 36MHz

#define RCC_CFGR_SW_TIMEOUT 100U
#define RCC_CFGR_PLLMUL_MIN_FACTOR 2U
#define RCC_CFGR_PLLMUL_MAX_FACTOR 16U
#define FLASH_ACR_LATENCY_THRESHOLD_1 24000000
#define FLASH_ACR_LATENCY_THRESHOLD_2 48000000

typedef enum
{
    CLOCK_HSI_8MHz,      // Default HSI
    CLOCK_HSE_8MHz,      // HSE 8MHz
    CLOCK_PLL_HSI_32MHz, // HSI/2 * 8 = 32MHz
    CLOCK_PLL_HSI_64MHz, // HSI/2 * 16 = 64MHz (max for HSI)
    CLOCK_PLL_HSE_48MHz, // HSE * 6 = 48MHz (useful for USB)
    CLOCK_PLL_HSE_72MHz, // HSE * 9 = 72MHz (max)
    CLOCK_CUSTOM         // Use custom config struct
} ClockPreset;

typedef struct clock_custom_config
{
    uint8_t use_hse;
    uint8_t enable_pllxtpre;
    uint8_t pllmul_factor;
} clock_custom_config_t;

uint32_t get_systick_count(void);
Result sysclk_switch(ClockPreset preset, const clock_custom_config_t *custom);
const char *sysclk_used(void);
uint32_t sysclk_frequency(void);
uint32_t cpu_clock_frequency(void);
void systick_init(void);
void delay_ms(const uint32_t ms);
uint32_t PPRE_to_prescaler_div_factor(const uint32_t ppre);
uint32_t HPRE_to_prescaler_div_factor(const uint32_t hpre);

#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))

#endif // CLOCK_H
