#include "stm32f103xb.h"
#include "mcu/clock.h"
#include "mcu/uart.h"
#include "mcu/integrated_led.h"
#include "mcu/user_button.h"
#include "bme280/bme280.h"
#include "lcd/lcd.h"

static bme280_calib_data_t calib_data;

static int error_blink_halt(const Result r);
static Result display_welcome_message();
static void uart_display_measurements(int32_t temperature, uint32_t pressure, uint32_t humidity);
static Result lcd_display_measurements(int32_t temperature, uint32_t pressure, uint32_t humidity);
static Result init_sysclk();
static Result init_peripherals(bme280_calib_data_t *calib_data);
static Result setup_custom_characters(void);
static void bme280_main_loop(void);
#ifdef BME280_FORCED_MODE
static void bme280_trigger_routine(const uint16_t input);
#endif

int main(void)
{
    integrated_led_init();
    OK_OR(init_sysclk(), error_blink_halt);
    OK_OR(init_peripherals(&calib_data), error_blink_halt);
    OK_OR(display_welcome_message(), error_blink_halt);

    // Define '°' as custom character at position 1
    OK_OR(setup_custom_characters(), error_blink_halt);

#ifdef BME280_FORCED_MODE
    OK_OR(lcd_set_backlight(0), error_blink_halt);
    enable_user_button((user_btn_trigger_config_t){
        .enable_rising = 0,
        .enable_falling = 1, // Triggers measurement on button release only
        .callback = bme280_trigger_routine,
    });
#endif
    bme280_main_loop();

    return 0;
}

static int error_blink_halt(const Result r)
{
    // uart_display_result(r);
    integrated_led_error_blink(r);
    return -1;
}

static Result display_welcome_message()
{
    uart_print_str("\n-------------------------------------\r\nSTM32F103CB WeAct Blue Pill Plus & BME280 Sensor Example\r\n-------------------------------------\r\n\n");

    OK_OR_PROPAGATE(lcd_set_backlight(1));
    OK_OR_PROPAGATE(lcd_set_cursor_pos(0, 0));
    OK_OR_PROPAGATE(lcd_putstr("STM32F103CB"));
    OK_OR_PROPAGATE(lcd_set_cursor_pos(1, 0));
    OK_OR_PROPAGATE(lcd_putstr("BME280 Sensor"));

    delay_ms(3000);
    OK_OR_PROPAGATE(lcd_clear());
    delay_ms(100);

    return OK;
}

static void uart_display_measurements(int32_t temperature, uint32_t pressure, uint32_t humidity)
{
    const uint32_t pa_int = pressure / (256 * 100);
    const uint32_t pa_frac = (pressure / 256) % 100;
    const int32_t t_int = temperature / 100;
    const int32_t t_frac = temperature % 100;

    uart_print_str("Temperature: ");
    uart_print_int(t_int);
    t_frac < 10 ? uart_print_str(".0") : uart_print_str(".");
    uart_print_int(t_frac < 0 ? -t_frac : t_frac);
    uart_print_str(" °C, Pressure: ");
    uart_print_int(pa_int);
    uart_print_str(pa_frac < 10 ? ".0" : ".");
    uart_print_int(pa_frac);
    uart_print_str(" hPa, Humidity: ");
    uart_print_int(humidity / 1024);
    uart_print_str(".");
    uart_print_int((humidity % 1024) * 1000 / 1024);
    uart_print_str(" %\r\n");
}

static Result lcd_display_measurements(int32_t temperature, uint32_t pressure, uint32_t humidity)
{
    const uint32_t pa_int = pressure / (256 * 100);
    const uint32_t pa_frac = (pressure / 256) % 100;
    const int32_t t_int = temperature / 100;
    const int32_t t_frac = (temperature % 100) / 10;

    // First line: T:xx.xC H:xxx%
    OK_OR_PROPAGATE(lcd_set_cursor_pos(0, 0));
    OK_OR_PROPAGATE(lcd_putstr("T:"));
    OK_OR_PROPAGATE(lcd_putint(t_int));
    OK_OR_PROPAGATE(lcd_putstr("."));
    OK_OR_PROPAGATE(lcd_putint(t_frac < 0 ? -t_frac : t_frac));
    OK_OR_PROPAGATE(lcd_putstr("\1C H:")); // '°' is custom character at pos 1, sending 0x01 as char to display means displaying '°'
    OK_OR_PROPAGATE(lcd_putint(humidity / 1024));
    OK_OR_PROPAGATE(lcd_putstr("%"));
    // Second line: P:xxxx.xx hPa
    OK_OR_PROPAGATE(lcd_set_cursor_pos(1, 0));
    OK_OR_PROPAGATE(lcd_putstr("P:"));
    OK_OR_PROPAGATE(lcd_putint(pa_int));
    OK_OR_PROPAGATE(lcd_putstr(pa_frac < 10 ? ".0" : "."));
    OK_OR_PROPAGATE(lcd_putint(pa_frac));
    OK_OR_PROPAGATE(lcd_putstr(" hPa"));

    return OK;
}

static Result init_sysclk()
{
    // clock_custom_config_t conf = {
    //     .use_hse = 1,
    //     .enable_pllxtpre = 0,
    //     .pllmul_factor = 9,
    // };
    // OK_OR_PROPAGATE(sysclk_switch(CLOCK_CUSTOM, &conf));
    OK_OR_PROPAGATE(sysclk_switch(CLOCK_PLL_HSE_72MHz, 0x0));
    systick_init();

    return OK;
}

static Result init_peripherals(bme280_calib_data_t *calib_data)
{
    uart_init();

    // The following ANSI escape code clears the terminal screen.
    // Note: Not all UART terminal programs support ANSI escape codes.
    // You may disable this line if your terminal does not clear as expected.
    uart_print_str("\033[2J");
    uart_print_str("UART initialized\r\n");
    uart_print_str("SysClk Hz = ");
    uart_print_int(sysclk_frequency());
    uart_print_str(" using [");
    uart_print_str(sysclk_used());
    uart_print_str("]\r\n");

    OK_OR_PROPAGATE(i2c1_init(0)); // Init I2C1 without remap (SCL: PB6, SDA: PB7)
    uart_print_str("I2C1 initialized (no remap)\r\n");

    // Init BME280
#ifdef BME280_USE_I2C
    bme280_set_i2c_instance(I2C1);
#else
    spi1_init(0, BME280_SPI_TARGET_BAUDRATE);
    uart_print_str("SPI1 initialized (no remap)\r\n");
#endif
    OK_OR_PROPAGATE(bme280_init());
    uart_print_str("--- BME280 Initialized. Reading Calibration Data...");
    OK_OR_PROPAGATE(bme280_get_calib(calib_data));
    uart_print_str("Done.\r\n");

    lcd_set_i2c_instance(I2C1);
    OK_OR_PROPAGATE(lcd_initialize());
    uart_print_str("--- LCD initialized\r\n");

    return OK;
}

#ifdef BME280_NORMAL_MODE
static void bme280_main_loop(void)
{
    int32_t temperature;
    uint32_t pressure, humidity;

    while (1)
    {
        integrated_led_switch(1);
        OK_OR(bme280_read_measurements(&temperature, &pressure, &humidity, &calib_data), error_blink_halt);
        uart_display_measurements(temperature, pressure, humidity);
        integrated_led_switch(0);
        OK_OR(lcd_set_backlight(1), error_blink_halt);
        OK_OR(lcd_display_measurements(temperature, pressure, humidity), error_blink_halt);
        delay_ms(5000);
        OK_OR(lcd_set_backlight(0), error_blink_halt);
        delay_ms(25000);
    }
}
#else
static volatile uint8_t g_forced_measurement_triggered = 0;

static void bme280_main_loop(void)
{
    int32_t temperature;
    uint32_t pressure, humidity;
    uint8_t ctrl_meas = BME280_CTRL_MEAS_OSRS_P_x16 | BME280_CTRL_MEAS_OSRS_T_x16;

    while (1)
    {
        if (g_forced_measurement_triggered)
        {
            integrated_led_switch(1);
            OK_OR(bme280_trigger_forced_mode(&ctrl_meas), error_blink_halt); // Trigger measurement
            OK_OR(bme280_read_measurements(&temperature, &pressure, &humidity, &calib_data), error_blink_halt);
            integrated_led_switch(0);
            uart_display_measurements(temperature, pressure, humidity);
            OK_OR(lcd_set_backlight(1), error_blink_halt);
            OK_OR(lcd_display_measurements(temperature, pressure, humidity), error_blink_halt);

            delay_ms(500); // Prevents measurement 0.5ms after one ended
            g_forced_measurement_triggered = 0;
            for (uint8_t k = 0; k < 25; ++k)
            {
                delay_ms(100);
                if (g_forced_measurement_triggered)
                    break;
            }
            if (!g_forced_measurement_triggered)
                OK_OR(lcd_set_backlight(0), error_blink_halt);
        }
        delay_ms(1);
    }
}

static void bme280_trigger_routine(const uint16_t input)
{
    if (!input)
        g_forced_measurement_triggered = 1;
}
#endif

static Result setup_custom_characters(void)
{
    const unsigned char degree_char_map[8] = {
        0b00110,
        0b01001,
        0b01001,
        0b00110,
        0b00000,
        0b00000,
        0b00000,
        0b00000,
    };
    // Define '°' as custom character at position 1
    OK_OR_PROPAGATE(lcd_create_custom_char(1, degree_char_map));

    return OK;
}
