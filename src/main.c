#include "stm32f103xb.h"
#include "clock.h"
#include "uart.h"
#include "i2c.h"
#include "bme280.h"
#include "lcd1602.h"

static int display_error_and_halt(const Result r)
{
    uart_display_result(r);
    while (1)
        ;
    return -1;
}

static Result display_welcome_message()
{
    Result r;

    uart_print_str("\n-------------------------------------\r\nSTM32F103CB WeAct Blue Pill Plus & BME280 Sensor Example\r\n-------------------------------------\r\n\n");

    if ((r = lcd1602_put_cursor(0, 0)) != OK)
        return r;
    if ((r = lcd1602_put_str("STM32F103CB")) != OK)
        return r;
    if ((r = lcd1602_put_cursor(1, 0)) != OK)
        return r;
    if ((r = lcd1602_put_str("BME280 Sensor")) != OK)
        return r;

    delay_ms(3000);
    if ((r = lcd1602_clear()) != OK)
        return r;
    delay_ms(100);

    return OK;
}

static void uart_display_measurements(int32_t temperature, uint32_t pressure, uint32_t humidity)
{
    const uint32_t pa = pressure / 256;
    const int32_t t_int = temperature / 100;
    const int32_t t_frac = temperature % 100;

    uart_print_str("Temperature: ");
    uart_print_int(t_int);
    t_frac < 10 ? uart_print_str(".0") : uart_print_str(".");
    uart_print_int(t_frac < 0 ? -t_frac : t_frac);
    uart_print_str(" °C, Pressure: ");
    uart_print_int(pa / 100);
    uart_print_str(".");
    uart_print_int(pa % 100);
    uart_print_str(" hPa, Humidity: ");
    uart_print_int(humidity / 1024);
    uart_print_str(".");
    uart_print_int((humidity % 1024) * 1000 / 1024);
    uart_print_str(" %\r\n");
}

static void lcd1602_display_measurements(int32_t temperature, uint32_t pressure, uint32_t humidity)
{
    const uint32_t pa = pressure / 256;
    const int32_t t_int = temperature / 100;
    const int32_t t_frac = (temperature % 100) / 10;

    // First line: T:xx.xC H:xxx%
    lcd1602_put_cursor(0, 0);
    lcd1602_put_str("T:");
    lcd1602_put_int(t_int);
    lcd1602_put_str(".");
    lcd1602_put_int(t_frac < 0 ? -t_frac : t_frac);
    lcd1602_put_str("C H:");
    lcd1602_put_int(humidity / 1024);
    lcd1602_put_str("%");
    // Second line: P:xxxx.xx hPa
    lcd1602_put_cursor(1, 0);
    lcd1602_put_str("P:");
    lcd1602_put_int(pa / 100);
    lcd1602_put_str(".");
    lcd1602_put_int(pa % 100);
    lcd1602_put_str(" hPa");
}

static Result init_peripherals(uint8_t *bme280_addr, bme280_calib_data_t *calib_data)
{
    Result r;

    uart_init();
    // The following ANSI escape code clears the terminal screen.
    // Note: Not all UART terminal programs support ANSI escape codes.
    // You may disable this line if your terminal does not clear as expected.
    uart_print_str("\033[2J");
    uart_print_str("UART initialized\r\n");
    systick_init();
    uart_print_str("SysTick initialized\r\n");

    // Init I2C1 without remap (SCL: PB6, SDA: PB7)
    r = i2c1_init(0);
    if (r != OK)
        return r;
    uart_print_str("I2C1 initialized (no remap)\r\n");

    *bme280_addr = bme280_i2c_address(0); // SDO is grounded, change to 1 if SDO is connected to VDDIO
    r = bme280_init(*bme280_addr);
    if (r != OK)
        return r;
    uart_print_str("--- BME280 Initialized\r\n");
    uart_print_str("--- Reading Calibration Data...");
    r = bme280_get_calib(calib_data, *bme280_addr);
    if (r != OK)
        return r;
    uart_print_str("Done.\r\n");

    // Init I2C2
    r = i2c2_init();
    if (r != OK)
        return r;
    uart_print_str("I2C2 initialized\r\n");

    // Init LCD1602
    r = lcd1602_init();
    if (r != OK)
        return r;
    uart_print_str("--- LCD1602 initialized\r\n");

    return OK;
}

#ifdef BME280_NORMAL_MODE
static void bme280_main_loop(const bme280_calib_data_t *calib_data, uint8_t bme280_addr)
{
    Result r;
    int32_t temperature;
    uint32_t pressure, humidity;

    while (1)
    {
        r = bme280_read_measurements(&temperature, &pressure, &humidity, calib_data, bme280_addr);
        if (r != OK)
        {
            display_error_and_halt(r);
            return;
        }
        uart_display_measurements(temperature, pressure, humidity);
        delay_ms(5000);
    }
}
#else
static void bme280_main_loop(const bme280_calib_data_t *calib_data, uint8_t bme280_addr)
{
    Result r;
    int32_t temperature;
    uint32_t pressure, humidity;
    uint8_t ctrl_meas = BME280_CTRL_MEAS_OSRS_P_x16 | BME280_CTRL_MEAS_OSRS_T_x16;

    while (1)
    {
        r = bme280_trigger_forced_mode(bme280_addr, &ctrl_meas); // Trigger measurement
        if (r != OK)
        {
            display_error_and_halt(r);
            return;
        }
        r = bme280_read_measurements(&temperature, &pressure, &humidity, calib_data, bme280_addr);
        if (r != OK)
        {
            display_error_and_halt(r);
            return;
        }
        uart_display_measurements(temperature, pressure, humidity);
        lcd1602_set_backlight(1);
        lcd1602_display_measurements(temperature, pressure, humidity);
        delay_ms(10000); // Keep backlight for 10 seconds when a new measurement is displayed
        lcd1602_set_backlight(0);
        delay_ms(50000); // Dim screen
    }
}
#endif

int main(void)
{
    Result r;
    uint8_t bme280_addr;
    bme280_calib_data_t calib_data;

    r = init_peripherals(&bme280_addr, &calib_data);
    if (r != OK)
        return display_error_and_halt(r);
    r = display_welcome_message();
    if (r != OK)
        return display_error_and_halt(r);

    bme280_main_loop(&calib_data, bme280_addr);

    return 0;
}
