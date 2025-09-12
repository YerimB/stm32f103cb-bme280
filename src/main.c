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
    uart_print_str("\n-------------------------------------\r\nSTM32F103CB WeAct Blue Pill Plus & BME280 Sensor Example\r\n-------------------------------------\r\n\n");

    OK_OR_PROPAGATE(lcd1602_put_cursor(0, 0));
    OK_OR_PROPAGATE(lcd1602_put_str("STM32F103CB"));
    OK_OR_PROPAGATE(lcd1602_put_cursor(1, 0));
    OK_OR_PROPAGATE(lcd1602_put_str("BME280 Sensor"));

    delay_ms(3000);
    OK_OR_PROPAGATE(lcd1602_clear());
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

static Result lcd1602_display_measurements(int32_t temperature, uint32_t pressure, uint32_t humidity)
{
    const uint32_t pa_int = pressure / (256 * 100);
    const uint32_t pa_frac = (pressure / 256) % 100;
    const int32_t t_int = temperature / 100;
    const int32_t t_frac = (temperature % 100) / 10;

    // First line: T:xx.xC H:xxx%
    OK_OR_PROPAGATE(lcd1602_put_cursor(0, 0));
    OK_OR_PROPAGATE(lcd1602_put_str("T:"));
    OK_OR_PROPAGATE(lcd1602_put_int(t_int));
    OK_OR_PROPAGATE(lcd1602_put_str("."));
    OK_OR_PROPAGATE(lcd1602_put_int(t_frac < 0 ? -t_frac : t_frac));
    OK_OR_PROPAGATE(lcd1602_put_str("C H:"));
    OK_OR_PROPAGATE(lcd1602_put_int(humidity / 1024));
    OK_OR_PROPAGATE(lcd1602_put_str("%"));
    // Second line: P:xxxx.xx hPa
    OK_OR_PROPAGATE(lcd1602_put_cursor(1, 0));
    OK_OR_PROPAGATE(lcd1602_put_str("P:"));
    OK_OR_PROPAGATE(lcd1602_put_int(pa_int));
    OK_OR_PROPAGATE(pa_frac < 10 ? lcd1602_put_str(".0") : lcd1602_put_str("."));
    OK_OR_PROPAGATE(lcd1602_put_int(pa_frac));
    OK_OR_PROPAGATE(lcd1602_put_str(" hPa"));

    return OK;
}

static Result init_peripherals(uint8_t *bme280_addr, bme280_calib_data_t *calib_data)
{
    uart_init();
    // The following ANSI escape code clears the terminal screen.
    // Note: Not all UART terminal programs support ANSI escape codes.
    // You may disable this line if your terminal does not clear as expected.
    uart_print_str("\033[2J");
    uart_print_str("UART initialized\r\n");
    systick_init();
    uart_print_str("SysTick initialized\r\n");

    // Init I2C1 without remap (SCL: PB6, SDA: PB7)
    OK_OR_PROPAGATE(i2c1_init(1));
    uart_print_str("I2C1 initialized (no remap)\r\n");

    *bme280_addr = bme280_i2c_address(0); // SDO is grounded, change to 1 if SDO is connected to VDDIO
    bme280_set_i2c_instance(I2C1);
    OK_OR_PROPAGATE(bme280_init(*bme280_addr));
    uart_print_str("--- BME280 Initialized\r\n");
    uart_print_str("--- Reading Calibration Data...");
    OK_OR_PROPAGATE(bme280_get_calib(calib_data, *bme280_addr));
    uart_print_str("Done.\r\n");

    // Init LCD1602
    lcd1602_set_i2c_instance(I2C1); // If using I2C2, be sure to initialize it using the `i2c2_init` function
    OK_OR_PROPAGATE(lcd1602_init());
    uart_print_str("--- LCD1602 initialized\r\n");

    return OK;
}

#ifdef BME280_NORMAL_MODE
static void bme280_main_loop(const bme280_calib_data_t *calib_data, uint8_t bme280_addr)
{
    int32_t temperature;
    uint32_t pressure, humidity;

    while (1)
    {
        OK_OR(bme280_read_measurements(&temperature, &pressure, &humidity, calib_data, bme280_addr), display_error_and_halt);
        uart_display_measurements(temperature, pressure, humidity);
        OK_OR(lcd1602_set_backlight(1), display_error_and_halt);
        OK_OR(lcd1602_display_measurements(temperature, pressure, humidity), display_error_and_halt);
        delay_ms(5000);
        OK_OR(lcd1602_set_backlight(0), display_error_and_halt); // Turn off backlight
        delay_ms(25000);
    }
}
#else
static void bme280_main_loop(const bme280_calib_data_t *calib_data, uint8_t bme280_addr)
{
    int32_t temperature;
    uint32_t pressure, humidity;
    uint8_t ctrl_meas = BME280_CTRL_MEAS_OSRS_P_x16 | BME280_CTRL_MEAS_OSRS_T_x16;

    while (1)
    {
        OK_OR(bme280_trigger_forced_mode(bme280_addr, &ctrl_meas), display_error_and_halt); // Trigger measurement
        OK_OR(bme280_read_measurements(&temperature, &pressure, &humidity, calib_data, bme280_addr), display_error_and_halt);
        uart_display_measurements(temperature, pressure, humidity);
        OK_OR(lcd1602_set_backlight(1), display_error_and_halt);
        OK_OR(lcd1602_display_measurements(temperature, pressure, humidity), display_error_and_halt);
        delay_ms(10000); // Keep backlight for 10 seconds when a new measurement is displayed
        OK_OR(lcd1602_set_backlight(0), display_error_and_halt);
        delay_ms(50000); // Dim screen
    }
}
#endif

int main(void)
{
    uint8_t bme280_addr;
    bme280_calib_data_t calib_data;

    OK_OR(init_peripherals(&bme280_addr, &calib_data), display_error_and_halt);
    OK_OR(display_welcome_message(), display_error_and_halt);

    bme280_main_loop(&calib_data, bme280_addr);

    return 0;
}
