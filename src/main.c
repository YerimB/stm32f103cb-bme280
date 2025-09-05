#include "stm32f103xb.h"
#include "bme280.h"

#include "clock.h"
#include "uart.h"
#include "i2c.h"

int display_error_and_halt(const Result r)
{
    uart_display_result(r);
    while (1)
        ;
    return -1;
}

void uart_display_measurements(int32_t temperature, uint32_t pressure, uint32_t humidity)
{
    uart_print_str("Temperature: ");
    uart_print_uint(temperature / 100);
    uart_print_str(".");
    uart_print_uint(temperature % 100);
    uart_print_str(" °C, ");

    const uint32_t pa = pressure / 256;
    uart_print_str("Pressure: ");
    uart_print_uint(pa / 100);
    uart_print_str(".");
    uart_print_uint(pa % 100);
    uart_print_str(" hPa, ");

    uart_print_str("Humidity: ");
    uart_print_uint(humidity / 1024);
    uart_print_str(".");
    uart_print_uint((humidity % 1024) * 1000 / 1024);
    uart_print_str(" %\r\n");
}

int main(void)
{
    Result r;

    uart_init();
    uart_print_str("UART initialized\r\n");
    systick_init();
    uart_print_str("SysTick initialized\r\n");
    r = i2c1_init(0);

    if (r != OK)
        return display_error_and_halt(r);
    uart_print_str("I2C1 initialized (no remap)\r\n");

    uart_print_str("\n-------------------------------------\r\nSTM32F103CB WeAct Blue Pill Plus & BME280 Sensor Example\r\n-------------------------------------\r\n");

    const uint8_t bme280_addr = bme280_i2c_address(0); // SDO is grounded, change to 1 if SDO is connected to VDDIO

    uint8_t bme280_id;
    uart_print_str("Reading BME280 ID...\r\n");
    r = bme280_read_registers(BME280_REG_ID, &bme280_id, 1, bme280_addr);
    uart_print_str("BME280 ID: ");
    if (r != OK)
        return display_error_and_halt(r);
    if (bme280_id != 0x60)
        return display_error_and_halt(SENSOR_NOT_DETECTED);
    uart_print_uint(bme280_id); // Expected value is 0x60 (or 96 in decimal notation)
    uart_print_str("\r\n");

    uart_print_str("Resetting BME280...");
    r = bme280_soft_reset(bme280_addr);
    if (r != OK)
        return display_error_and_halt(r);
    delay_ms(5); // Wait for reset to complete
    uart_print_str("BME280 Reset Complete\r\n");

    bme280_calib_data_t calib_data;
    uart_print_str("Reading BME280 Calibration Data...");
    r = bme280_get_calib(&calib_data, bme280_addr);
    if (r != OK)
        return display_error_and_halt(r);
    uart_print_str("Done\r\n");

    uart_print_str("Configuring BME280...");

    int32_t temperature;
    uint32_t pressure, humidity;
#ifdef BME280_NORMAL_MODE
    r = bme280_set_ctrl_hum(BME280_CTRL_HUM_OSRS_H_x4, bme280_addr);
    if (r != OK)
        return display_error_and_halt(r);
    r = bme280_set_ctrl_meas(BME280_CTRL_MEAS_MODE_NORMAL | BME280_CTRL_MEAS_OSRS_P_x4 | BME280_CTRL_MEAS_OSRS_T_x4, bme280_addr);
    if (r != OK)
        return display_error_and_halt(r);
    r = bme280_set_config(BME280_CONFIG_TIME_STANDBY_1000MS | BME280_CONFIG_FILTER_x4, bme280_addr);
    if (r != OK)
        return display_error_and_halt(r);
    uart_print_str("Normal Mode Configuration Complete\r\n");

    while (1)
    {
        r = bme280_read_measurements(&temperature, &pressure, &humidity, &calib_data, bme280_addr);
        if (r != OK)
            return display_error_and_halt(r);
        uart_display_measurements(temperature, pressure, humidity);
        delay_ms(5000);
    }
#else
    uint8_t ctrl_meas_osrs = BME280_CTRL_MEAS_OSRS_P_x16 | BME280_CTRL_MEAS_OSRS_T_x16;

    r = bme280_set_ctrl_hum(BME280_CTRL_HUM_OSRS_H_x16, bme280_addr);
    if (r != OK)
        return display_error_and_halt(r);
    r = bme280_set_ctrl_meas(BME280_CTRL_MEAS_MODE_SLEEP | ctrl_meas_osrs, bme280_addr); // Ensure sleep mode even if it is the default on power-up
    if (r != OK)
        return display_error_and_halt(r);
    r = bme280_set_config(BME280_CONFIG_FILTER_OFF, bme280_addr);
    if (r != OK)
        return display_error_and_halt(r);
    uart_print_str("Forced Mode Configuration Complete\r\n");

    while (1)
    {
        r = bme280_trigger_forced_mode(&ctrl_meas_osrs, bme280_addr); // Trigger measurement
        if (r != OK)
            return display_error_and_halt(r);
        r = bme280_read_measurements(&temperature, &pressure, &humidity, &calib_data, bme280_addr);
        if (r != OK)
            return display_error_and_halt(r);
        uart_display_measurements(temperature, pressure, humidity);
        delay_ms(30000); // Could be any value, as sensor is in forced mode
    }
#endif

    return 0;
}
