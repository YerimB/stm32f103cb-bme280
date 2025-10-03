#include "clock.h"

#define HSI_STABILIZE_CYCLES 20000UL  // ~2.5ms at 8MHz
#define HSE_STABILIZE_CYCLES 200000UL // ~25ms at 8MHz
#define PLL_STABILIZE_CYCLES 20000UL  // ~2.5ms at 8MHz
#define SWS_POLL_CYCLES 20000UL       // ~2.5ms at 8MHz

static volatile uint32_t systick_count = 0;

static uint8_t get_pllmul_factor(void);
static uint32_t factor_to_pllmul(const uint8_t factor);
static uint8_t pllmul_factor_is_valid(const uint8_t pllmul);
static uint32_t frequency_to_flash_acr_latency(const uint32_t freq);
static Result hsi_enable_internal(void);
static Result hse_enable_internal(void);
static Result pll_enable_internal(const uint8_t use_hse, const uint8_t enable_pllxtpre, const uint8_t pllmul_factor);
static Result hsi_disable_internal(void);
static Result hse_disable_internal(void);
static Result pll_disable_internal(void);
static Result disable_unused_internal(void);
static uint32_t pll_estimate_frequency(const uint8_t use_hse, const uint8_t enable_pllxtpre, const uint8_t pllmul_factor);

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

// TODO: pause/resume interrupts
// TODO: peripherals reset with correct clocks
Result sysclk_switch(ClockPreset preset, const clock_custom_config_t *custom)
{
    uint8_t sw = RCC_CFGR_SW_PLL; // PLL if not changed
    uint8_t pll_use_hse = 0;      // HSI/2 if not changed
    uint8_t enable_pllxtpre = 0;  // Disabled by default
    uint8_t pllmul_factor = 2;    // Default to 2

    switch (preset)
    {
    case CLOCK_HSI_8MHz:
        sw = RCC_CFGR_SW_HSI;
        break;
    case CLOCK_HSE_8MHz:
        sw = RCC_CFGR_SW_HSE;
        break;
    case CLOCK_PLL_HSI_32MHz:
        pllmul_factor = 8; // 4MHz * 8 = 32MHz
        break;
    case CLOCK_PLL_HSI_64MHz:
        pllmul_factor = 16; // 4MHz * 16 = 64MHz
        break;
    case CLOCK_PLL_HSE_48MHz:
        pll_use_hse = 1;   // HSE
        pllmul_factor = 6; // 8MHz * 6 = 48MHz
        break;
    case CLOCK_PLL_HSE_72MHz:
        pll_use_hse = 1;   // HSE
        pllmul_factor = 9; // 8MHz * 9 = 72MHz
        break;
    case CLOCK_CUSTOM:
        if (!custom)
            return CLOCK_CONFIG_INVALID;
        pll_use_hse = custom->use_hse;
        enable_pllxtpre = custom->enable_pllxtpre;
        pllmul_factor = custom->pllmul_factor;
        break;
    }

    // Enable target clock
    if (sw == RCC_CFGR_SW_HSI) // HSI
        OK_OR_PROPAGATE(hsi_enable_internal());
    else if (sw == RCC_CFGR_SW_HSE) // HSE
        OK_OR_PROPAGATE(hse_enable_internal());
    else // PLL
        OK_OR_PROPAGATE(pll_enable_internal(pll_use_hse, enable_pllxtpre, pllmul_factor));

    // Set SW
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | sw;

    // Poll SWS to confirm switch (no timeout)
    for (volatile uint32_t i = SWS_POLL_CYCLES; i > 0; --i)
    {
        if (((RCC->CFGR & RCC_CFGR_SWS_Msk) >> RCC_CFGR_SWS_Pos) == sw)
        {
            disable_unused_internal(); // Safe to disable unused now
            return OK;
        }
    }

    return CLOCK_SW_FAILURE;
}

const char *sysclk_used(void)
{
    const uint32_t sws = RCC->CFGR & RCC_CFGR_SWS_Msk;
    const uint32_t hsi_ready = RCC->CR & RCC_CR_HSIRDY;
    const uint32_t hse_ready = RCC->CR & RCC_CR_HSERDY;
    const uint32_t pll_ready = RCC->CR & RCC_CR_PLLRDY;

    // Check current clock source (SWS bits in CFGR, bits 3:2)
    if (sws == RCC_CFGR_SWS_PLL && pll_ready) // PLL used as system clock
    {
        // Get PLL multiplication factor (PLLMUL, bits 21:18)
        if ((RCC->CFGR & RCC_CFGR_PLLSRC) && hse_ready) // PLLSRC is HSE
        {
            if (RCC->CFGR & RCC_CFGR_PLLXTPRE) // PLLXTPRE set => PLLSRC is HSE/2
                return "(HSE_hz / 2) * pllmul";
            // else
            return "HSE_hz * pllmul";
        }
        else if (hsi_ready) // PLLSRC is HSI/2
            return "(HSI_HZ / 2) * pllmul";
    }
    else if (sws == RCC_CFGR_SWS_HSE && hse_ready) // HSE used as system clock
        return "HSE_hz";
    else if (sws == RCC_CFGR_SWS_HSI && hsi_ready) // `RCC->CFGR & RCC_CFGR_SWS_HSI` => HSI used as system clock
        return "HSI_HZ";
    return "0"; // Selected clock source is not ready
}

// See stm32f103c8 reference manual - chapter 8 (https://www.st.com/en/microcontrollers-microprocessors/stm32f103c8.html#documentation)
uint32_t sysclk_frequency(void)
{
    const uint32_t sws = RCC->CFGR & RCC_CFGR_SWS_Msk;
    const uint32_t hsi_ready = RCC->CR & RCC_CR_HSIRDY;
    const uint32_t hse_ready = RCC->CR & RCC_CR_HSERDY;
    const uint32_t pll_ready = RCC->CR & RCC_CR_PLLRDY;

    // Check current clock source (SWS bits in CFGR, bits 3:2)
    if (sws == RCC_CFGR_SWS_PLL && pll_ready) // PLL used as system clock
    {

        // Get PLL multiplication factor (PLLMUL, bits 21:18)
        const uint8_t pllmul_factor = get_pllmul_factor();

        if ((RCC->CFGR & RCC_CFGR_PLLSRC) && hse_ready) // PLLSRC is HSE
        {
            if (RCC->CFGR & RCC_CFGR_PLLXTPRE) // PLLXTPRE set => PLLSRC is HSE/2
                return (HSE_HZ / 2) * pllmul_factor;
            // else
            return HSE_HZ * pllmul_factor;
        }
        else if (hsi_ready) // PLLSRC is HSI/2
            return (HSI_HZ / 2) * pllmul_factor;
    }
    else if (sws == RCC_CFGR_SWS_HSE && hse_ready) // HSE used as system clock
        return HSE_HZ;
    else if (sws == RCC_CFGR_SWS_HSI && hsi_ready) // `RCC->CFGR & RCC_CFGR_SWS_HSI` => HSI used as system clock
        return HSI_HZ;
    return 0; // Selected clock source is not ready
}

uint32_t cpu_clock_frequency(void)
{
    const uint32_t ahb_prescaler = (RCC->CFGR >> 4) & 15;

    return sysclk_frequency() / HPRE_to_prescaler_div_factor(ahb_prescaler);
}

// See ARMv7-M architecture reference manual - section B3.3 (https://developer.arm.com/documentation/ddi0403/latest)
void systick_init(void)
{
    SysTick->LOAD = (cpu_clock_frequency() / 1000) - 1; // Ticks per ms;
    SysTick->VAL = 0;                                   // Reset timer
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
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

static uint8_t get_pllmul_factor(void)
{
    const uint8_t pllmul_nibble = (RCC->CFGR & RCC_CFGR_PLLMULL_Msk) >> RCC_CFGR_PLLMULL_Pos;

    return MIN(pllmul_nibble + RCC_CFGR_PLLMUL_MIN_FACTOR, RCC_CFGR_PLLMUL_MAX_FACTOR);
}

static uint32_t factor_to_pllmul(const uint8_t factor)
{
    uint32_t pllmul_nibble = MAX(factor, RCC_CFGR_PLLMUL_MIN_FACTOR);

    pllmul_nibble = MIN(pllmul_nibble, RCC_CFGR_PLLMUL_MAX_FACTOR);
    return (pllmul_nibble - RCC_CFGR_PLLMUL_MIN_FACTOR) << RCC_CFGR_PLLMULL_Pos;
}

static uint8_t pllmul_factor_is_valid(const uint8_t pllmul)
{
    // Reserved values defined in RM0008 - 8.3.2
    return pllmul >= 2 && pllmul <= 16;
}

static Result hsi_enable_internal(void)
{
    if (RCC->CR & RCC_CR_HSIRDY)
        return OK; // Already enabled

    RCC->CR |= RCC_CR_HSION; // Enable
    for (volatile uint32_t i = HSI_STABILIZE_CYCLES; i > 0; --i)
        ;

    if (!(RCC->CR & RCC_CR_HSIRDY))
        return CLOCK_NOT_READY;
    return OK;
}

static Result hsi_disable_internal(void)
{
    if ((RCC->CFGR & RCC_CFGR_SWS_Msk) == RCC_CFGR_SWS_HSI ||                                   // HSI used as system clock
        ((RCC->CFGR & RCC_CFGR_SWS_Msk) == RCC_CFGR_SWS_PLL && !(RCC->CFGR & RCC_CFGR_PLLSRC))) // PLL uses HSI and is used as system clock
        return CLOCK_ILLEGAL_DISABLE;
    if (!(RCC->CR & RCC_CR_HSION))
        return OK;

    if (RCC->CR & RCC_CR_HSION)
        RCC->CR &= ~RCC_CR_HSION;
    if (!(RCC->CR & RCC_CR_HSIRDY))
    {
        for (volatile uint32_t i = HSI_STABILIZE_CYCLES; i > 0; --i)
            ;
    }

    return (RCC->CR & RCC_CR_HSIRDY) ? CLOCK_NOT_READY : OK;
}

static Result hse_enable_internal(void)
{
    if (RCC->CR & RCC_CR_HSERDY)
        return OK; // Already enabled

    RCC->CR |= RCC_CR_HSEON;
    for (volatile uint32_t i = HSE_STABILIZE_CYCLES; i > 0; --i)
        ;

    if (RCC->CIR & RCC_CIR_CSSF)
        return CLOCK_HSE_FAILURE;
    if (!(RCC->CR & RCC_CR_HSERDY))
        return CLOCK_NOT_READY;

    return OK;
}

static Result hse_disable_internal(void)
{
    if ((RCC->CFGR & RCC_CFGR_SWS_Msk) == RCC_CFGR_SWS_HSE ||                                   // HSE used as system clock
        ((RCC->CFGR & RCC_CFGR_SWS_Msk) == RCC_CFGR_SWS_PLL && !(RCC->CFGR & RCC_CFGR_PLLSRC))) // PLL uses HSE and is used as system clock
        return CLOCK_ILLEGAL_DISABLE;
    if (!(RCC->CR & RCC_CR_HSEON))
        return OK;

    if (RCC->CR & RCC_CR_HSEON)
        RCC->CR &= ~RCC_CR_HSEON;
    if (!(RCC->CR & RCC_CR_HSERDY))
    {
        for (volatile uint32_t i = HSE_STABILIZE_CYCLES; i > 0; --i)
            ;
    }

    return (RCC->CR & RCC_CR_HSERDY) ? CLOCK_NOT_READY : OK;
}

static uint32_t pll_estimate_frequency(const uint8_t use_hse, const uint8_t enable_pllxtpre, const uint8_t pllmul_factor)
{
    if (use_hse)
    {
        if (enable_pllxtpre)
            return (HSE_HZ / 2) * pllmul_factor;
        else
            return HSE_HZ * pllmul_factor;
    }
    return (HSI_HZ / 2) * pllmul_factor;
}

static uint32_t frequency_to_flash_acr_latency(const uint32_t freq)
{
    if (freq <= FLASH_ACR_LATENCY_THRESHOLD_1)
        return 0;
    if (freq <= FLASH_ACR_LATENCY_THRESHOLD_2)
        return FLASH_ACR_LATENCY_0; // 1
    // if <= 72M
    return FLASH_ACR_LATENCY_1; // 2
}

static Result pll_enable_internal(const uint8_t use_hse, const uint8_t enable_pllxtpre, const uint8_t pllmul_factor)
{
    if ((RCC->CFGR & RCC_CFGR_SWS_Msk) == RCC_CFGR_SWS_PLL)
        return OK;

    const uint32_t estimated_frequency = pll_estimate_frequency(use_hse, enable_pllxtpre, pllmul_factor);
    // Validate config
    if (!pllmul_factor_is_valid(pllmul_factor) || use_hse > 1 || enable_pllxtpre > 1 ||
        estimated_frequency > RCC_MAX_FREQUENCY)
        return CLOCK_CONFIG_INVALID;

    const uint32_t pllmul = factor_to_pllmul(pllmul_factor);

    // Enable PLL input
    if (use_hse)
        OK_OR_PROPAGATE(hse_enable_internal()); // PLLSRC = HSE
    else
        OK_OR_PROPAGATE(hsi_enable_internal()); // PLLSRC = HSI/2

    // Define config & Configure PLL
    const uint32_t config = (use_hse << RCC_CFGR_PLLSRC_Pos) |
                            (enable_pllxtpre << RCC_CFGR_PLLXTPRE_Pos) |
                            pllmul;

    OK_OR_PROPAGATE(pll_disable_internal()); // Disable before configuring as per datasheet
    if (estimated_frequency > APB1_MAX_HZ)   // APB1 runs at max 36MHz so if HCLK exceeds it, the prescaler must be >2.
        RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_PPRE1_Msk) | RCC_CFGR_PPRE1_DIV2;
    FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY_Msk) | frequency_to_flash_acr_latency(estimated_frequency);
    RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL)) | config;
    RCC->CR |= RCC_CR_PLLON;

    for (volatile uint32_t i = PLL_STABILIZE_CYCLES; i > 0; --i)
        ;

    return (RCC->CR & RCC_CR_PLLRDY) ? OK : CLOCK_NOT_READY;
}

static Result pll_disable_internal(void)
{
    if ((RCC->CFGR & RCC_CFGR_SWS_Msk) == RCC_CFGR_SWS_PLL)
        return CLOCK_ILLEGAL_DISABLE; // PLL used as system clock
    if (!(RCC->CR & RCC_CR_PLLON))
        return OK; // Already disabled

    if (RCC->CR & RCC_CR_PLLON)
        RCC->CR &= ~RCC_CR_PLLON;
    for (volatile uint32_t i = PLL_STABILIZE_CYCLES; i > 0; --i)
    {
        if (!(RCC->CR & RCC_CR_PLLRDY))
            return OK;
    }

    return CLOCK_NOT_READY; // Failed to disable
}

static Result disable_unused_internal(void)
{
    const uint32_t sws = RCC->CFGR & RCC_CFGR_SWS_Msk;

    if (sws == RCC_CFGR_SWS_HSI)
    {
        OK_OR_PROPAGATE(hse_disable_internal());
        OK_OR_PROPAGATE(pll_disable_internal());
    }
    else if (sws == RCC_CFGR_SWS_HSE)
    {
        OK_OR_PROPAGATE(hsi_disable_internal());
        OK_OR_PROPAGATE(pll_disable_internal());
    }
    else if (sws == RCC_CFGR_SWS_PLL)
    {
        if (RCC->CFGR & RCC_CFGR_PLLSRC_Msk)
            OK_OR_PROPAGATE(hsi_disable_internal());
        else
            OK_OR_PROPAGATE(hse_disable_internal());
    }

    return OK;
}
