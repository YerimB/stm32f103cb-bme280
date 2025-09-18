#include "uart.h"
#include "clock.h"

static const uint32_t UART_BAUDRATE = 115200;

static uint32_t div_to_BRR(uint32_t frequency, uint32_t baudrate)
{
    uint32_t DIV_mantissa = frequency / (16 * baudrate);
    uint32_t DIV_fraction = (((((uint64_t)frequency * 100) / (16 * baudrate)) % 100) * 16) / 100;

    return (DIV_mantissa << USART_BRR_DIV_Mantissa_Pos) | (DIV_fraction & USART_BRR_DIV_Fraction);
}

void uart_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_IOPAEN; // Enable clocks for UART1 and GPIOA

    // Set PA9 (TX) as Alternate function push-pull
    GPIOA->CRH &= ~(0xF << 4);                        // Clear config for PA9
    GPIOA->CRH |= GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9_1; // Alternate Function Push-Pull, 2MHz (bits 7:4 set to '1010')

    // Set PA10 (RX) as Input floating (commented since I am not receiving anything for now)
    // GPIOA->CRH &= ~(0xF << 8);      // Clear config for PA10
    // GPIOA->CRH |= GPIO_CRH_CNF10_0; // Input floating (bits 11:8 set to '0100')

    // Set the baud rate register value
    USART1->BRR = div_to_BRR(sysclk_frequency(), UART_BAUDRATE);

    // Enable UART (UE bit) & Transmitter (TE) (Not receiving for now, commenter)
    USART1->CR1 |= USART_CR1_UE | USART_CR1_TE; // | USART_CR1_RE;
}

void uart_send_char(const char c)
{
    // Wait until the Transmit Data Register is empty (TXE bit is set)
    while (!(USART1->SR & USART_SR_TXE))
        ;
    // Write the byte to the Data Register
    USART1->DR = c;
}

void uart_print_str(const char *str)
{
    while (*str)
    {
        uart_send_char(*str++);
    }
}

void uart_print_int(int32_t n)
{
    if (n == 0)
    {
        uart_send_char('0');
        return;
    }

    char buffer[11];
    uint8_t i = 0;

    if (n >> 31)
    {
        uart_send_char('-');
        n = -n;
    }

    while (n > 0)
    {
        buffer[i++] = (n % 10) + '0';
        n /= 10;
    }
    while (i > 0)
        uart_send_char(buffer[--i]);
}

void uart_print_hex(int32_t n)
{
    if (n == 0)
    {
        uart_print_str("0x0");
        return;
    }

    char buffer[8];
    uint8_t i = 0;
    uint32_t un = (uint32_t)n;

    while (un > 0)
    {
        uint8_t nibble = un & 0xF;

        buffer[i++] = nibble < 10 ? '0' + nibble : 'A' + (nibble - 10);
        un /= 16;
    }

    uart_print_str("0x");
    while (i > 0)
        uart_send_char(buffer[--i]);
}
