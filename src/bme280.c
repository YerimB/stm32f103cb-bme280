#include "bme280.h"
#include "clock.h"

// BME280 reference: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf

static Result bme280_set_ctrl_hum(const uint8_t ctrl_hum);
static Result bme280_set_ctrl_meas(const uint8_t ctrl_meas);
static Result bme280_set_config(const uint8_t config);
static Result bme280_configure(void);
static Result bme280_wait_for_measurement(void);
static int32_t bme280_compensate_T(int32_t adc_T, const bme280_calib_data_t *calib);
static uint32_t bme280_compensate_P(int32_t adc_P, const bme280_calib_data_t *calib);
static uint32_t bme280_compensate_H(int32_t adc_H, const bme280_calib_data_t *calib);

#ifdef BME280_USE_I2C
static I2C_TypeDef *bme280_i2c_instance = I2C1; // Default value

/**
 * Sets the I2C peripheral instance used by the BME280 driver.
 *
 * This function allows the user to specify which I2C peripheral instance
 * the BME280 driver should use for communication. If a valid (non-NULL)
 * pointer is provided, the internal reference will be updated to use the
 * specified I2C instance. By default, the driver uses I2C1.
 *
 * @param i2c_instance Pointer to the I2C peripheral instance to use (e.g., I2C1, I2C2).
 */
void bme280_set_i2c_instance(I2C_TypeDef *i2c_instance)
{
    if (i2c_instance)
        bme280_i2c_instance = i2c_instance;
}
#endif

Result bme280_init(void)
{
    uint8_t bme280_id;

    OK_OR_PROPAGATE(bme280_soft_reset());                                 // Soft reset BME280
    OK_OR_PROPAGATE(bme280_read_registers(BME280_REG_ID, &bme280_id, 1)); // Read ID register

    if (bme280_id != BME280_ID)
        return SENSOR_NOT_DETECTED;

    OK_OR_PROPAGATE(bme280_configure());

    return OK;
}

Result bme280_write_register(const uint8_t reg, const uint8_t value)
{
#ifdef BME280_USE_I2C
    return i2c_write(bme280_i2c_instance, BME280_I2C_ADDR, (uint8_t[2]){reg, value}, 2, (I2C_ops_params_t){0, 1});
#else
    const uint8_t reg_cmd = reg & BME280_SPI_WRITE_MASK;

    GPIOA->BSRR = GPIO_BSRR_BR4; // Select BME280
    OK_OR_PROPAGATE(spi_write_bytes(SPI1, (uint8_t[2]){reg_cmd, value}, 2));
    GPIOA->BSRR = GPIO_BSRR_BS4; // Unselect BME280
    return OK;
#endif
}

Result bme280_read_registers(const uint8_t reg, uint8_t *data, uint32_t len)
{
#ifdef BME280_USE_I2C
    OK_OR_PROPAGATE(i2c_write(bme280_i2c_instance, BME280_I2C_ADDR, &reg, 1, (I2C_ops_params_t){0, 0})); // Write register address to read from
    return i2c_read(bme280_i2c_instance, BME280_I2C_ADDR, data, len, (I2C_ops_params_t){1, 1});          // Read data
#else
    const uint8_t reg_cmd = reg | BME280_SPI_READ_MASK;

    GPIOA->BSRR = GPIO_BSRR_BR4;                      // Select BME280
    OK_OR_PROPAGATE(spi_write_byte(SPI1, reg_cmd));   // Write address to read
    OK_OR_PROPAGATE(spi_read_bytes(SPI1, data, len)); // Batch read
    GPIOA->BSRR = GPIO_BSRR_BS4;                      // Unselect BME280
    return OK;
#endif
}

Result bme280_soft_reset()
{
#ifdef BME280_USE_I2C
    OK_OR_PROPAGATE(i2c_write(bme280_i2c_instance, BME280_I2C_ADDR, (uint8_t[2]){BME280_REG_RESET, BME280_RESET_CMD}, 2, (I2C_ops_params_t){0, 1}));
    delay_ms(BME280_SOFT_RESET_DELAY_MS);
#else
    GPIOA->BSRR = GPIO_BSRR_BR4;
    OK_OR_PROPAGATE(spi_write_bytes(SPI1, (uint8_t[2]){BME280_REG_RESET, BME280_RESET_CMD}, 2));
    delay_ms(BME280_SOFT_RESET_DELAY_MS);
    GPIOA->BSRR = GPIO_BSRR_BS4;
#endif
    return OK;
}

static Result calib_failure(const Result r)
{
    uart_display_result(r);
    return SENSOR_CALIB_FAILED;
}

// Reference: Section 4.2.2 of the datasheet (Table 16)
Result bme280_get_calib(bme280_calib_data_t *calib)
{
    uint8_t data[24];
    OK_OR_RETURN(bme280_read_registers(0x88, data, 24), calib_failure);
    calib->dig_T1 = (uint16_t)(data[0] | (data[1] << 8));
    calib->dig_T2 = (int16_t)(data[2] | (data[3] << 8));
    calib->dig_T3 = (int16_t)(data[4] | (data[5] << 8));
    calib->dig_P1 = (uint16_t)(data[6] | (data[7] << 8));
    calib->dig_P2 = (int16_t)(data[8] | (data[9] << 8));
    calib->dig_P3 = (int16_t)(data[10] | (data[11] << 8));
    calib->dig_P4 = (int16_t)(data[12] | (data[13] << 8));
    calib->dig_P5 = (int16_t)(data[14] | (data[15] << 8));
    calib->dig_P6 = (int16_t)(data[16] | (data[17] << 8));
    calib->dig_P7 = (int16_t)(data[18] | (data[19] << 8));
    calib->dig_P8 = (int16_t)(data[20] | (data[21] << 8));
    calib->dig_P9 = (int16_t)(data[22] | (data[23] << 8));

    uint8_t h1;
    OK_OR_RETURN(bme280_read_registers(0xA1, &h1, 1), calib_failure);
    calib->dig_H1 = h1;

    uint8_t hdata[7];
    OK_OR_RETURN(bme280_read_registers(0xE1, hdata, 7), calib_failure);
    calib->dig_H2 = (int16_t)(hdata[0] | (hdata[1] << 8));
    calib->dig_H3 = hdata[2];
    // H4 and H5 share a byte, see datasheet for details
    calib->dig_H4 = (int16_t)((hdata[3] << 4) | (hdata[4] & 0x0F));
    calib->dig_H5 = (int16_t)(((hdata[4] >> 4) & 0x0F) | (hdata[5] << 4));
    calib->dig_H6 = (int8_t)hdata[6];

    if (calib->dig_T1 == 0 || calib->dig_P1 == 0 || calib->dig_H1 == 0)
        return SENSOR_CALIB_FAILED;

    return OK;
}

/*
 * Triggers a forced mode measurement on the BME280 sensor by setting the ctrl_meas
 * register (0xF4) to forced mode while preserving the existing temperature and
 * pressure oversampling settings. If ctrl_meas is NULL, the current register value
 * is read; otherwise, the provided value is used.
 *
 * Parameters:
 *   ctrl_meas - Pointer to the ctrl_meas value (NULL to read current value) or
 *               the desired value with oversampling settings.
 */
Result bme280_trigger_forced_mode(uint8_t *ctrl_meas)
{
    uint8_t ctrl_meas_forced;

    if (!ctrl_meas)
    {
        const uint8_t reg = BME280_REG_CTRL_MEAS;

#ifdef BME280_USE_I2C
        OK_OR_PROPAGATE(i2c_write(bme280_i2c_instance, BME280_I2C_ADDR, &reg, 1, (I2C_ops_params_t){0, 0}));
        OK_OR_PROPAGATE(i2c_read(bme280_i2c_instance, BME280_I2C_ADDR, &ctrl_meas_forced, 1, (I2C_ops_params_t){1, 0}));
#else
        OK_OR_PROPAGATE(bme280_read_registers(reg, &ctrl_meas_forced, 1));
#endif
        // Mask out the 2 LSBs and set to forced mode
        ctrl_meas_forced = (ctrl_meas_forced & ~0x3) | BME280_CTRL_MEAS_MODE_FORCED;
    }
    else
    {
        ctrl_meas_forced = (*ctrl_meas & ~0x3) | BME280_CTRL_MEAS_MODE_FORCED;
    }

#ifdef BME280_USE_I2C
    OK_OR_PROPAGATE(i2c_write(bme280_i2c_instance, BME280_I2C_ADDR, (uint8_t[2]){BME280_REG_CTRL_MEAS, ctrl_meas_forced}, 2, (I2C_ops_params_t){1, 1}));
#else
    OK_OR_PROPAGATE(bme280_write_register(BME280_REG_CTRL_MEAS, ctrl_meas_forced));
#endif

    delay_ms(5); // Wait a short time to ensure the command is processed

    return OK;
}

/**
 * Reads temperature, pressure, and humidity measurements from the BME280 sensor.
 *
 * @param temp  Pointer to store temperature in 0.01 °C units.
 * @param press Pointer to store pressure in Q24.8 format (value/256 = Pa).
 * @param hum   Pointer to store humidity in Q22.10 format (value/1024 = %RH).
 * @param calib Pointer to calibration data.
 * @return Result code.
 */
Result bme280_read_measurements(int32_t *temp, uint32_t *press, uint32_t *hum, const bme280_calib_data_t *calib)
{
    uint8_t raw[8];

    // Wait until measurement is complete
    OK_OR_PROPAGATE(bme280_wait_for_measurement());
    // Read raw data burst (8 bytes) as per datasheet
    OK_OR_PROPAGATE(bme280_read_registers(0xF7, raw, 8));

    // Assemble 20-bit unsigned for press/temp, 16-bit for hum
    int32_t adc_P = ((uint32_t)raw[0] << 12) | ((uint32_t)raw[1] << 4) | (raw[2] >> 4);
    int32_t adc_T = ((uint32_t)raw[3] << 12) | ((uint32_t)raw[4] << 4) | (raw[5] >> 4);
    int32_t adc_H = ((uint32_t)raw[6] << 8) | raw[7];

    *temp = bme280_compensate_T(adc_T, calib);
    *press = bme280_compensate_P(adc_P, calib);
    *hum = bme280_compensate_H(adc_H, calib);

    return OK;
}

// note: ctrl_hum must be written before ctrl_meas for it to take effect
static Result bme280_set_ctrl_hum(const uint8_t ctrl_hum)
{
    // 0x7 is a mask for the 3 LSBs
    return bme280_write_register(BME280_REG_CTRL_HUM, ctrl_hum & 0x7);
}

static Result bme280_set_ctrl_meas(const uint8_t ctrl_meas)
{
    return bme280_write_register(BME280_REG_CTRL_MEAS, ctrl_meas);
}

static Result bme280_set_config(const uint8_t config)
{
    return bme280_write_register(BME280_REG_CONFIG, config);
}

static Result bme280_configure(void)
{
#ifdef BME280_NORMAL_MODE
    OK_OR_PROPAGATE(bme280_set_ctrl_hum(BME280_CTRL_HUM_OSRS_H_x4));
    OK_OR_PROPAGATE(bme280_set_ctrl_meas(BME280_CTRL_MEAS_MODE_NORMAL | BME280_CTRL_MEAS_OSRS_P_x4 | BME280_CTRL_MEAS_OSRS_T_x4));
    OK_OR_PROPAGATE(bme280_set_config(BME280_CONFIG_TIME_STANDBY_1000MS | BME280_CONFIG_FILTER_x4));
#else
    uint8_t ctrl_meas_osrs = BME280_CTRL_MEAS_OSRS_P_x16 | BME280_CTRL_MEAS_OSRS_T_x16;

    OK_OR_PROPAGATE(bme280_set_ctrl_hum(BME280_CTRL_HUM_OSRS_H_x16));
    OK_OR_PROPAGATE(bme280_set_ctrl_meas(BME280_CTRL_MEAS_MODE_SLEEP | ctrl_meas_osrs)); // Ensure sleep mode even if it is the default on power-up
    OK_OR_PROPAGATE(bme280_set_config(BME280_CONFIG_FILTER_OFF));
#endif

    return OK;
}

// Waits until occuring measurement is complete if any
static Result bme280_wait_for_measurement(void)
{
    uint8_t status;
    uint32_t tp = get_systick_count();

    do
    {
        if ((get_systick_count() - tp) > 100)
        {
            return SENSOR_INVALID_READ;
        }

        OK_OR_PROPAGATE(bme280_read_registers(BME280_REG_STATUS, &status, 1));

        if (!(status & 0x08)) // Bit 3: 1=measuring, 0=done
            break;
        delay_ms(1);
    } while (1);

    return OK;
}

/****************************************************************************/
/* THE FOLLOWING COMPENSATION FUNCTIONS ARE DEFINED IN THE BME280 DATASHEET */
/****************************************************************************/

static int32_t t_fine;

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
// t_fine carries fine temperature as global value
static int32_t bme280_compensate_T(int32_t adc_T, const bme280_calib_data_t *calib)
{
    int32_t var1, var2, T;

    var1 = ((((adc_T >> 3) - ((int32_t)calib->dig_T1 << 1))) * ((int32_t)calib->dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)calib->dig_T1)) * ((adc_T >> 4) - ((int32_t)calib->dig_T1))) >> 12) *
            ((int32_t)calib->dig_T3)) >>
           14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;

    return T;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
static uint32_t bme280_compensate_P(int32_t adc_P, const bme280_calib_data_t *calib)
{
    int64_t var1, var2, p;

    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)calib->dig_P6;
    var2 = var2 + ((var1 * (int64_t)calib->dig_P5) << 17);
    var2 = var2 + (((int64_t)calib->dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)calib->dig_P3) >> 8) + ((var1 * (int64_t)calib->dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calib->dig_P1) >> 33;
    if (var1 == 0)
    {
        return 0; // aResult exception caused by division by zero
    }
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)calib->dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)calib->dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)calib->dig_P7) << 4);

    return (uint32_t)p;
}

// Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits).
// Output value of “47445” represents 47445/1024 = 46.333 %RH
static uint32_t bme280_compensate_H(int32_t adc_H, const bme280_calib_data_t *calib)
{
    int32_t v_x1_u32r;

    v_x1_u32r = (t_fine - ((int32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)calib->dig_H4) << 20) - (((int32_t)calib->dig_H5) * v_x1_u32r)) +
                   ((int32_t)16384)) >>
                  15) *
                 (((((((v_x1_u32r *
                        ((int32_t)calib->dig_H6)) >>
                       10) *
                      (((v_x1_u32r * ((int32_t)calib->dig_H3)) >> 11) +
                       ((int32_t)32768))) >>
                     10) +
                    ((int32_t)2097152)) *
                       ((int32_t)calib->dig_H2) +
                   8192) >>
                  14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
                               ((int32_t)calib->dig_H1)) >>
                              4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);

    return (uint32_t)(v_x1_u32r >> 12);
}
