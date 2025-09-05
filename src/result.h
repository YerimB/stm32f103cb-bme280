#ifndef ERROR_H_
#define ERROR_H_

typedef enum
{
    OK = 0, // Success
    // I2C Errors
    I2C_INIT_FAILED, // I2C initialization error
    I2C_TIMEOUT,     // I2C operation timed out
    I2C_NACK,        // I2C address NACK received
    I2C_BUS_ERROR,   // I2C bus error (BERR)
    // Sensor Errors (here BME280)
    SENSOR_NOT_DETECTED, // Sensor ID read failed
    SENSOR_INVALID_READ, // Invalid sensor data read
    SENSOR_CALIB_FAILED  // Sensor calibration read failed
} Result;

void uart_display_result(const Result status);

#endif // ERROR_H_
