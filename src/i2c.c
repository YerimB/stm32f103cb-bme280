#include "i2c.h"
#include "clock.h"

Result i2c1_init(const int remap)
{
    // Enable clock for GPIOB & I2C1
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

    if (remap)
    {
        RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
        AFIO->MAPR |= AFIO_MAPR_I2C1_REMAP;          // Enable remap
        GPIOB->CRH &= ~(0xFF << GPIO_CRH_MODE8_Pos); // Reset values for GPIOB_CRL pin 8 & 9
        // Set GPIO 8 & 9 to Alternate Function Open Drain at 10MHz (Needed for I2C protocol)
        GPIOB->CRH |=
            (GPIO_CRH_CNF8 | GPIO_CRH_MODE8_0) |
            (GPIO_CRH_CNF9 | GPIO_CRH_MODE9_0);
    }
    else
    {
        AFIO->MAPR &= ~AFIO_MAPR_I2C1_REMAP;         // Disable remap (or ensure it is disabled)
        GPIOB->CRL &= ~(0xFF << GPIO_CRL_MODE6_Pos); // Reset values for GPIOB_CRL pin 6 & 7
        // Set GPIO 6 & 7 to Alternate Function Open Drain at 10MHz (Needed for I2C protocol)
        GPIOB->CRL |=
            (GPIO_CRL_CNF6 | GPIO_CRL_MODE6_0) |
            (GPIO_CRL_CNF7 | GPIO_CRL_MODE7_0);
    }

    const uint32_t hclk_frequency = cpu_clock_frequency();
    const uint32_t apb1_prescaler = PPRE_to_prescaler_div_factor((RCC->CFGR >> 8) & 0x7);
    const uint32_t apb1_clock_frequency_MHz = (hclk_frequency / apb1_prescaler) / 1000000;
    const uint32_t cr2_freq_MHz = apb1_clock_frequency_MHz < 50 ? apb1_clock_frequency_MHz : 50; // Max is 50Mhz

    // Reset I2C to ensure it is in a known state
    I2C1->CR1 = I2C_CR1_SWRST;
    I2C1->CR1 &= ~I2C_CR1_SWRST;

    I2C1->CR1 &= ~I2C_CR1_PE; // Disable to configure I2C

    I2C1->CR2 = cr2_freq_MHz;
    I2C1->CCR &= ~I2C_CCR_FS;                                           // Ensure Sm mode I2C
    I2C1->CCR |= (cr2_freq_MHz * 1000000) / (2 * I2C_MAX_FREQUENCY_Sm); // Set clock control register
    I2C1->TRISE = cr2_freq_MHz + 1;

    I2C1->CR1 |= I2C_CR1_PE; // Re-enable

    return OK;
}

Result i2c2_init(void)
{
    // Enable clock for GPIOB & I2C2
    RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
    RCC->APB2ENR |= (RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPBEN);

    GPIOB->CRH &= ~(0xFF << GPIO_CRH_MODE10_Pos); // Reset values for GPIOB_CRH pin 10 & 11
    // Set GPIO 10 & 11 to Alternate Function Open Drain at 10MHz (Needed for I2C protocol)
    GPIOB->CRH |=
        (GPIO_CRH_CNF10 | GPIO_CRH_MODE10_0) |
        (GPIO_CRH_CNF11 | GPIO_CRH_MODE11_0);

    const uint32_t hclk_frequency = cpu_clock_frequency();
    const uint32_t apb1_prescaler = PPRE_to_prescaler_div_factor((RCC->CFGR >> 8) & 0x7);
    const uint32_t apb1_clock_frequency_MHz = (hclk_frequency / apb1_prescaler) / 1000000;
    const uint32_t cr2_freq_MHz = apb1_clock_frequency_MHz < 50 ? apb1_clock_frequency_MHz : 50; // Max is 50Mhz

    // Reset I2C to ensure it is in a known state
    I2C2->CR1 = I2C_CR1_SWRST;
    I2C2->CR1 &= ~I2C_CR1_SWRST;

    I2C2->CR1 &= ~I2C_CR1_PE; // Disable to configure I2C

    I2C2->CR2 = cr2_freq_MHz;
    I2C2->CCR &= ~I2C_CCR_FS;                                           // Ensure Sm mode I2C
    I2C2->CCR |= (cr2_freq_MHz * 1000000) / (2 * I2C_MAX_FREQUENCY_Sm); // Set clock control register
    I2C2->TRISE = cr2_freq_MHz + 1;

    I2C2->CR1 |= I2C_CR1_PE; // Re-enable

    return OK;
}

// Waits for ongoing communication to end if any (Bus freed)
static Result i2c_wait_for_bus(I2C_TypeDef *i2c)
{
    if (!i2c)
        return INVALID_PARAMETER;

    const uint32_t tp = get_systick_count();

    while (i2c->SR2 & I2C_SR2_BUSY)
    {
        if ((get_systick_count() - tp) > I2C_TIMEOUT_MS)
            return I2C_TIMEOUT_ERR;
    }
    return OK;
}

Result i2c_write(I2C_TypeDef *i2c, const uint8_t slave_addr, const uint8_t *data, const uint32_t size, const I2C_ops_params_t params)
{
    if (!params.restart)
    {
        OK_OR_PROPAGATE(i2c_wait_for_bus(i2c));
    }

    uint32_t tp;

    i2c->CR1 |= I2C_CR1_START; // Start/Restart communication
    tp = get_systick_count();
    while (!(i2c->SR1 & I2C_SR1_SB)) // Wait for confirmation
    {
        if ((get_systick_count() - tp) > I2C_TIMEOUT_MS)
            return I2C_TIMEOUT_ERR;
    }

    i2c->DR = (slave_addr << 1); // Send slave address (7 bits) followed by 0 (write mode)
    tp = get_systick_count();
    while (!(i2c->SR1 & I2C_SR1_ADDR)) // Wait for end of address transmission
    {
        if (i2c->SR1 & I2C_SR1_AF)
        {
            i2c->SR1 &= ~I2C_SR1_AF;
            return I2C_NACK;
        }
        if ((get_systick_count() - tp) > I2C_TIMEOUT_MS)
            return I2C_TIMEOUT_ERR;
    }
    (void)i2c->SR2; // Clear ADDR by reading in sequence SR1 -> SR2

    for (uint32_t i = 0; i < size; ++i)
    {
        i2c->DR = data[i]; // Send data
        tp = get_systick_count();
        while (!(i2c->SR1 & I2C_SR1_TXE)) // Wait for data to be received by slave
        {
            if (i2c->SR1 & I2C_SR1_AF)
            {
                i2c->SR1 &= ~I2C_SR1_AF;
                return I2C_NACK;
            }
            if ((get_systick_count() - tp) > I2C_TIMEOUT_MS)
                return I2C_TIMEOUT_ERR;
        }
    }

    if (i2c->SR1 & I2C_SR1_BERR)
    {
        i2c->SR1 &= ~I2C_SR1_BERR;
        return I2C_BUS_ERROR;
    }
    if (params.generate_stop)
    {
        i2c->CR1 |= I2C_CR1_STOP; // Generate stop condition
    }

    return OK;
}

Result i2c_read(I2C_TypeDef *i2c, const uint8_t slave_addr, uint8_t *data, const uint32_t size, const I2C_ops_params_t params)
{
    // We do not want to wait if we are restarting
    if (!params.restart)
    {
        OK_OR_PROPAGATE(i2c_wait_for_bus(i2c));
    }

    uint32_t tp;

    i2c->CR1 |= I2C_CR1_START; // Start/Restart communication
    tp = get_systick_count();
    while (!(i2c->SR1 & I2C_SR1_SB)) // Wait for confirmation
    {
        if ((get_systick_count() - tp) > I2C_TIMEOUT_MS)
            return I2C_TIMEOUT_ERR;
    }

    i2c->DR = (slave_addr << 1) | 1; // Send slave address (7 bits) followed by 1 (read mode)
    tp = get_systick_count();
    while (!(i2c->SR1 & I2C_SR1_ADDR)) // Wait for end of address transmission
    {
        if (i2c->SR1 & I2C_SR1_AF)
        {
            i2c->SR1 &= ~I2C_SR1_AF;
            return I2C_NACK;
        }
        if (get_systick_count() - tp > I2C_TIMEOUT_MS)
            return I2C_TIMEOUT_ERR;
    }
    (void)i2c->SR2; // Clear ADDR by reading in sequence SR1 -> SR2

    if (size == 1)
    {
        i2c->CR1 &= ~I2C_CR1_ACK;
        tp = get_systick_count();
        while (!(i2c->SR1 & I2C_SR1_RXNE))
        {
            if (get_systick_count() - tp > I2C_TIMEOUT_MS)
                return I2C_TIMEOUT_ERR;
        }
        data[0] = i2c->DR;
    }
    else
    {
        i2c->CR1 |= I2C_CR1_ACK; // Enable ACK generation to receive several chunks
        for (uint32_t i = 0; i < size; ++i)
        {
            if (i == size - 1)
                i2c->CR1 &= ~I2C_CR1_ACK; // Disable ACK on last byte (generates NACK => stops waiting for data)
            tp = get_systick_count();
            while (!(i2c->SR1 & I2C_SR1_RXNE))
            {
                if (get_systick_count() - tp > I2C_TIMEOUT_MS)
                    return I2C_TIMEOUT_ERR;
            }
            data[i] = i2c->DR;
        }
    }

    if (i2c->SR1 & I2C_SR1_BERR)
    {
        i2c->SR1 &= ~I2C_SR1_BERR;
        return I2C_BUS_ERROR;
    }
    if (params.generate_stop)
    {
        i2c->CR1 |= I2C_CR1_STOP; // Generate stop condition
    }

    return OK;
}
