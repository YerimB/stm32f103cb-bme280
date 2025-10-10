#ifndef ERROR_H_
#define ERROR_H_

typedef enum Result
{
    OK = 0, // Success
    // Clock errors
    CLOCK_NOT_READY,
    CLOCK_CONFIG_INVALID,
    CLOCK_SW_FAILURE,
    CLOCK_HSE_FAILURE,
    CLOCK_ILLEGAL_DISABLE,
    // I2C Errors
    I2C_INIT_FAILED,      // I2C initialization error
    I2C_TIMEOUT_ERR,      // I2C operation timed out
    I2C_NACK,             // I2C address NACK received
    I2C_BUS_ERROR,        // I2C bus error (BERR)
    I2C_ARBITRATION_LOST, // I2C arbitration lost (ARLO)
    I2C_OVERRUN_UNDERRUN, // I2C overrun/underrun (OVR)
    // SPI Errors
    SPI_TIMEOUT_ERR,
    // Sensor Errors (here BME280)
    SENSOR_NOT_DETECTED, // Sensor ID read failed
    SENSOR_INVALID_READ, // Invalid sensor data read
    SENSOR_CALIB_FAILED, // Sensor calibration read failed
    // LCD1602 Errors
    LCD1602_INVALID_POS, // LCD1602 invalid cursor position
    // General Errors
    INVALID_PARAMETER, // Function parameter invalid
} Result;

#define OK_OR_PROPAGATE(expr)                    \
    do                                           \
    {                                            \
        Result _ok_or_propagate_result = (expr); \
        if (_ok_or_propagate_result != OK)       \
            return _ok_or_propagate_result;      \
    } while (0)

#define OK_OR(expr, fallback)          \
    do                                 \
    {                                  \
        Result _ok_or_result = (expr); \
        if (_ok_or_result != OK)       \
            fallback(_ok_or_result);   \
    } while (0)

#define OK_OR_RETURN(expr, fallback)               \
    do                                             \
    {                                              \
        Result _ok_or_return_result = (expr);      \
        if (_ok_or_return_result != OK)            \
            return fallback(_ok_or_return_result); \
    } while (0)

void uart_display_result(const Result status);

#endif // ERROR_H_
