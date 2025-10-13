#include "utils/result.h"
#include "mcu/uart.h"

void uart_display_result(const Result status)
{
    switch (status)
    {
    case OK:
        uart_print_str("OK\r\n");
        break;
    case CLOCK_ILLEGAL_DISABLE:
        uart_print_str("Attempt to disable oscillator while in use\r\n");
        break;
    case CLOCK_HSE_FAILURE:
        uart_print_str("Failure detected in the external oscillator\r\n");
        break;
    case CLOCK_NOT_READY:
        uart_print_str("System clock switch timed out (NOT READY)\r\n");
        break;
    case CLOCK_SW_FAILURE:
        uart_print_str("System clock switch failed\r\n");
        break;
    case CLOCK_CONFIG_INVALID:
        uart_print_str("Invalid clock configuration\r\n");
        break;
    case I2C_INIT_FAILED:
        uart_print_str("I2C Initialization Failed\r\n");
        break;
    case I2C_TIMEOUT_ERR:
        uart_print_str("I2C Timeout Error\r\n");
        break;
    case I2C_NACK:
        uart_print_str("I2C NACK Received\r\n");
        break;
    case I2C_BUS_ERROR:
        uart_print_str("I2C Bus Error (BERR)\r\n");
        break;
    case SPI_TIMEOUT_ERR:
        uart_print_str("SPI Timeout Error\r\n");
        break;
    case SENSOR_NOT_DETECTED:
        uart_print_str("Sensor Not Detected\r\n");
        break;
    case SENSOR_INVALID_READ:
        uart_print_str("Sensor Invalid Read\r\n");
        break;
    case SENSOR_CALIB_FAILED:
        uart_print_str("Sensor Calibration Read Failed\r\n");
        break;
    case INVALID_PARAMETER:
        uart_print_str("Invalid Parameter was provided\r\n");
        break;
    default:
        uart_print_str("Unknown Error\r\n");
        break;
    }
}
