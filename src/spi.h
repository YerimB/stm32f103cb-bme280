#ifndef SPI_H
#define SPI_H

#include "stm32f103xb.h"
#include "clock.h"
#include "result.h"

#define SPI_TIMEOUT_MS 100

void spi1_init(const uint8_t remap, const uint32_t target_baudrate);
Result spi_write_byte(SPI_TypeDef *spi, const uint8_t data);
Result spi_write_bytes(SPI_TypeDef *spi, const uint8_t *data, uint32_t count);
Result spi_read_byte(SPI_TypeDef *spi, uint8_t *data);
Result spi_read_bytes(SPI_TypeDef *spi, uint8_t *data, uint32_t count);

#endif // SPI_H
