#include "mcu/clock.h"
#include "mcu/user_button.h"

#define TIM_CR1_RESET 0x0000U
#define TIM_DIER_RESET 0x0000U
#define TIM_ARR_RESET 0xFFFFU
#define TIM_PSC_RESET 0x0000U
#define TIM_SR_RESET 0x0000U

static void debounce_timer_init(void);
static void debounce_timer_reset(void);
static void debounce_timer_start(void);

static uint8_t last_pin_stable_state = 0;
static EXTI_callback callback = 0x0;

void enable_user_button(const user_btn_trigger_config_t config)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN;
    GPIOA->CRL &= ~(GPIO_CRL_CNF0_Msk | GPIO_CRL_MODE0_Msk);                               // Set to 0b0000
    GPIOA->CRL |= GPIO_CRL_CNF0_1;                                                         // Input pull-up/pull-down
    GPIOA->ODR &= ~GPIO_ODR_ODR0;                                                          // Pull-down
    last_pin_stable_state = GPIOA->IDR & GPIO_IDR_IDR0;                                    // Get initial pin state
    AFIO->EXTICR[0] = (AFIO->EXTICR[0] & ~AFIO_EXTICR1_EXTI0_Msk) | AFIO_EXTICR1_EXTI0_PA; // Set source input for EXTI0 to PA0

    if (config.callback)
        callback = config.callback;
    if (config.enable_rising)
        EXTI->RTSR |= EXTI_RTSR_TR0;
    if (config.enable_falling)
        EXTI->FTSR |= EXTI_FTSR_TR0;

    EXTI->IMR |= EXTI_IMR_IM0;
    NVIC_EnableIRQ(EXTI0_IRQn);

    debounce_timer_init();
}

void disable_user_button(void)
{
    debounce_timer_reset();

    NVIC_DisableIRQ(EXTI0_IRQn);
    EXTI->IMR &= ~EXTI_IMR_IM0;
    EXTI->FTSR &= ~EXTI_FTSR_TR0;
    EXTI->RTSR &= ~EXTI_RTSR_TR0;
    EXTI->PR = EXTI_PR_PR0; // W1C register
    AFIO->EXTICR[0] &= ~AFIO_EXTICR1_EXTI0_Msk;
    // GPIOA->ODR &= ~GPIO_ODR_ODR0_Msk;
    GPIOA->CRL = (GPIOA->CRL & ~(GPIO_CRL_CNF0_Msk | GPIO_CRL_MODE0_Msk)) | GPIO_CRL_CNF0_0;
}

static void button_debounce_finished(void)
{
    const uint8_t pin_state = (GPIOA->IDR & GPIO_IDR_IDR0);
    const uint8_t ft_enabled = EXTI->FTSR & EXTI_FTSR_TR0;
    const uint8_t rt_enabled = EXTI->RTSR & EXTI_RTSR_TR0;

    if (callback &&
        ((rt_enabled && ft_enabled && pin_state != last_pin_stable_state) ||
         (rt_enabled && pin_state) ||
         (ft_enabled && !pin_state)))
    {
        callback(pin_state);
    }

    last_pin_stable_state = pin_state; // Update the last stable state for the next event
    NVIC_EnableIRQ(EXTI0_IRQn);
}

#ifdef __cplusplus
extern "C"
{
#endif

    void EXTI0_IRQHandler(void)
    {
        if (EXTI->PR & EXTI_PR_PR0)
        {
            NVIC_DisableIRQ(EXTI0_IRQn);
            EXTI->PR = EXTI_PR_PR0; // W1C register
            debounce_timer_start();
        }
    }

    void TIM2_IRQHandler(void)
    {
        if (TIM2->SR & TIM_SR_UIF)
        {
            TIM2->SR &= ~TIM_SR_UIF;
            button_debounce_finished();
        }
    }

#ifdef __cplusplus
}
#endif

static void debounce_timer_init(void)
{
    const uint32_t hclk = cpu_clock_frequency();
    const uint32_t apb1_prescaler = PPRE_to_prescaler_div_factor((RCC->CFGR & RCC_CFGR_PPRE1_Msk) >> RCC_CFGR_PPRE1_Pos);
    const uint32_t pclk1 = hclk / apb1_prescaler;
    const uint32_t ck_psc = apb1_prescaler > 1 ? (pclk1 * 2) : pclk1; // Per clock tree

    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = (ck_psc / 1000000) - 1; // µs
    TIM2->ARR = DEBOUNCE_US - 1;
    TIM2->CR1 |= TIM_CR1_OPM;   // One Pulse Mode
    TIM2->DIER |= TIM_DIER_UIE; // Enable interrupt trigger

    NVIC_SetPriority(TIM2_IRQn, 3);
    NVIC_EnableIRQ(TIM2_IRQn);
}

static void debounce_timer_reset(void)
{
    NVIC_DisableIRQ(TIM2_IRQn);
    TIM2->DIER = TIM_DIER_RESET;
    TIM2->CR1 = TIM_CR1_RESET;
    TIM2->ARR = TIM_ARR_RESET;
    TIM2->PSC = TIM_PSC_RESET;
    TIM2->SR = TIM_SR_RESET;
}

static void debounce_timer_start(void)
{
    TIM2->CR1 |= TIM_CR1_CEN;
}
