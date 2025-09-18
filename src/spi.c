#include "spi.h"

static uint8_t compute_SPI_CR1_BR(const uint32_t target_baud, SPI_TypeDef *spi_instance);
static Result spi_write_dummy_byte(SPI_TypeDef *spi_instance);
static Result spi_read_dummy_byte(SPI_TypeDef *spi_instance);

void spi1_init(const uint8_t remap, const uint32_t target_baudrate)
{
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; // Enable clock for SPI1

    // CS: Output mode, general purpose push-pull
    // SCK: Output mode, AF push-pull
    // MISO: Input mode, floating input
    // MOSI: Output mode, AF push-pull
    if (remap)
    {
        RCC->APB2ENR |= (RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN);
        AFIO->MAPR |= AFIO_MAPR_SPI1_REMAP;

        // Pins are PA15 (CS), PB3 (SCLK), PB4 (MISO), PB5 (MOSI)
        GPIOA->CRH &= ~(0xF << GPIO_CRH_MODE15_Pos);
        GPIOB->CRL &= ~(0xFFF << GPIO_CRL_MODE3_Pos);
        GPIOA->CRH |= GPIO_CRH_MODE15;                     // Output (50MHz), General purpose push-pull
        GPIOB->CRL |= (GPIO_CRL_MODE3 | GPIO_CRL_CNF3_1) | // Output (50MHz), Alternate function push-pull
                      GPIO_CRL_CNF4_0 |                    // Input, Floating
                      (GPIO_CRL_MODE5 | GPIO_CRL_CNF5_1);  // Output (50MHz), Alternate function push-pull

        GPIOA->BSRR = GPIO_BSRR_BR15; // CS low to force SPI mode
    }
    else
    {
        RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
        AFIO->MAPR &= ~AFIO_MAPR_SPI1_REMAP; // Ensure no remap

        // Pins are PA4 (CS), PA5 (SCLK), PA6 (MISO), PA7 (MOSI)
        GPIOA->CRL &= ~(0xFFFF << GPIO_CRL_MODE4_Pos);     // Reset values for GPIOA_CRL pin 4, 5, 6 & 7
        GPIOA->CRL |= GPIO_CRL_MODE4 |                     // Output (50MHz), General purpose push-pull
                      (GPIO_CRL_MODE5 | GPIO_CRL_CNF5_1) | // Output (50MHz), Alternate function push-pull
                      GPIO_CRL_CNF6_0 |                    // Input, Floating
                      (GPIO_CRL_MODE7 | GPIO_CRL_CNF7_1);  // Output (50MHz), Alternate function push-pull

        GPIOA->BSRR = GPIO_BSRR_BR4; // CS low to force SPI mode
    }

    SPI1->CR1 &= ~SPI_CR1_SPE; // Disable SPI1 before configuring
    SPI1->CR1 = 0x0;           // Reset value
    SPI1->CR1 |= SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | compute_SPI_CR1_BR(target_baudrate, SPI1);
    // SPI1->CR1 &= ~(SPI_CR1_CPOL | SPI_CR1_CPHA | SPI_CR1_LSBFIRST); // Ensure mode '00', MSB first
    SPI1->CR1 |= SPI_CR1_SPE; // Re-enable SPI1
}

Result spi_write_byte(SPI_TypeDef *spi, const uint8_t data)
{
    uint32_t tp = get_ms_count();

    while (!(spi->SR & SPI_SR_TXE)) // Wait for TX buffer empty
    {
        if (get_ms_count() - tp > SPI_TIMEOUT_MS)
            return SPI_TIMEOUT_ERR;
    }

    spi->DR = data; // Write data
    spi_read_dummy_byte(spi);

    tp = get_ms_count();
    while (spi->SR & SPI_SR_BSY) // Wait for not busy (end of transfer)
    {
        if (get_ms_count() - tp > SPI_TIMEOUT_MS)
            return SPI_TIMEOUT_ERR;
    }

    return OK;
}

Result spi_write_bytes(SPI_TypeDef *spi, const uint8_t *data, uint32_t count)
{
    while (count--)
        OK_OR_PROPAGATE(spi_write_byte(spi, *data++));

    return OK;
}

Result spi_read_byte(SPI_TypeDef *spi, uint8_t *data)
{
    OK_OR_PROPAGATE(spi_write_dummy_byte(spi));

    uint32_t tp = get_ms_count();
    while (!(spi->SR & SPI_SR_RXNE)) // Wait for RX not empty
    {
        if (get_ms_count() - tp > SPI_TIMEOUT_MS)
            return SPI_TIMEOUT_ERR;
    }

    *data = spi->DR; // Read data

    tp = get_ms_count();
    while (spi->SR & SPI_SR_BSY) // Wait for not busy
    {
        if (get_ms_count() - tp > SPI_TIMEOUT_MS)
            return SPI_TIMEOUT_ERR;
    }

    return OK;
}

Result spi_read_bytes(SPI_TypeDef *spi, uint8_t *data, uint32_t count)
{
    while (count--)
        OK_OR_PROPAGATE(spi_read_byte(spi, data++));

    return OK;
}

static Result spi_write_dummy_byte(SPI_TypeDef *spi_instance)
{
    uint32_t tp = get_ms_count();

    while (!(spi_instance->SR & SPI_SR_TXE)) // Wait for TX buffer empty
    {
        if (get_ms_count() - tp > SPI_TIMEOUT_MS)
            return SPI_TIMEOUT_ERR;
    }
    spi_instance->DR = 0xFF; // Dummy write to trigger SCK for slave to write

    tp = get_ms_count();
    while (SPI1->SR & SPI_SR_BSY) // Wait for not busy (end of transfer)
    {
        if (get_ms_count() - tp > SPI_TIMEOUT_MS)
            return SPI_TIMEOUT_ERR;
    }

    return OK;
}

static Result spi_read_dummy_byte(SPI_TypeDef *spi_instance)
{
    uint32_t tp = get_ms_count();

    while (!(spi_instance->SR & SPI_SR_RXNE)) // Wait for RX buffer not empty
    {
        if (get_ms_count() - tp > SPI_TIMEOUT_MS)
            return SPI_TIMEOUT_ERR;
    }
    spi_instance->DR; // Dummy read to clear data register

    tp = get_ms_count();
    while (SPI1->SR & SPI_SR_BSY) // Wait for not busy (end of transfer)
    {
        if (get_ms_count() - tp > SPI_TIMEOUT_MS)
            return SPI_TIMEOUT_ERR;
    }

    return OK;
}

static uint8_t compute_SPI_CR1_BR(const uint32_t target_baud, SPI_TypeDef *spi_instance)
{
    // SPI1 uses APB2 & SPI2 uses APB1
    const uint32_t hclk_frequency = cpu_clock_frequency();
    const uint32_t ppre = (RCC->CFGR >> (spi_instance == SPI1 ? RCC_CFGR_PPRE2_Pos : RCC_CFGR_PPRE1_Pos)) & 0x7;
    const uint32_t apb_prescaler = PPRE_to_prescaler_div_factor(ppre);
    const uint32_t pclk_frequency = hclk_frequency / apb_prescaler;
    // Find smallest prescaler >= pclk_frequency / target_baud
    const uint32_t presc = (pclk_frequency + target_baud - 1) / target_baud; // Ceiling div
    // Clamp to powers of 2 (2..256), find BR (0-7)
    uint8_t br = 0;

    // 0x7 (111) is the max BR value
    while ((1U << (br + 1)) < presc && br < 0x7)
        br++;

    return br << SPI_CR1_BR_Pos;
}
