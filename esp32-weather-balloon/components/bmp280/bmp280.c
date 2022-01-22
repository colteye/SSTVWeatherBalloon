#include <math.h>
#include "bmp280.h"
#include "error_handling.h"
#include "sensor_i2c.h"
#include "convert.h"

// These global BMP280 variables necessary for generating measurements.
int32_t t_fine;
bmp280_coefficients_t coeffs;

esp_err_t bmp280_read_coeffs(bmp280_coefficients_t *coeffs)
{
    // Init error.
    esp_err_t err = ESP_OK;

    // Read all of the coefficient calibration registers at once.
    uint8_t data[24];
    ESP_ERROR_VALIDATE("BMP 280 Coefficient Register Read",
                       err,
                       i2c_sensor_register_read(BMP280_ADDR, BMP280_DIG_T1_REG, data, 24))

    // Write them all in a useful form for downstream calculations.
    coeffs->dig_T1 = U16_2_U16_LE(U8S_2_U16(data[0], data[1]));
    coeffs->dig_T2 = U16_2_U16_LE(U8S_2_U16(data[2], data[3]));
    coeffs->dig_T3 = U16_2_U16_LE(U8S_2_U16(data[4], data[5]));

    coeffs->dig_P1 = U16_2_U16_LE(U8S_2_U16(data[6], data[7]));
    coeffs->dig_P2 = U16_2_U16_LE(U8S_2_U16(data[8], data[9]));
    coeffs->dig_P3 = U16_2_U16_LE(U8S_2_U16(data[10], data[11]));
    coeffs->dig_P4 = U16_2_U16_LE(U8S_2_U16(data[12], data[13]));
    coeffs->dig_P5 = U16_2_U16_LE(U8S_2_U16(data[14], data[15]));
    coeffs->dig_P6 = U16_2_U16_LE(U8S_2_U16(data[16], data[17]));
    coeffs->dig_P7 = U16_2_U16_LE(U8S_2_U16(data[18], data[19]));
    coeffs->dig_P8 = U16_2_U16_LE(U8S_2_U16(data[20], data[21]));
    coeffs->dig_P9 = U16_2_U16_LE(U8S_2_U16(data[22], data[23]));

    return ESP_OK;
}

esp_err_t bmp280_init(void)
{
    // Init error.
    esp_err_t err = ESP_OK;

    // Validate that the chip ID is correct.
    uint8_t chip_id = 0;
    ESP_ERROR_VALIDATE("BMP 280 Chip ID",
                       err,
                       i2c_sensor_register_read(BMP280_ADDR, BMP280_CHIPID_REG, &chip_id, 1))

    ESP_ERROR_VALIDATE("BMP 280 Chip ID Validation",
                       err,
                       ((chip_id == BMP280_CHIPID) - 1))

    // Print the chip ID.
    ESP_LOGI("BMP 280 Initializion", "Chip ID %X", chip_id);

    // Read BMP280 Coefficients for processing.
    ESP_ERROR_VALIDATE("BMP 280 Coefficients Read",
                       err,
                       bmp280_read_coeffs(&coeffs))

    // Set configuration register.
    uint8_t config = (STANDBY_MS_1 << 5) | (FILTER_OFF << 2);
    ESP_ERROR_VALIDATE("BMP 280 Configuration Write",
                       err,
                       i2c_sensor_register_write_byte(BMP280_ADDR, BMP280_CONFIG_REG, config))

    // Set control register.
    uint8_t control = (SAMPLING_X16 << 5) | (SAMPLING_X16 << 2) | MODE_NORMAL;
    ESP_ERROR_VALIDATE("BMP 280 Control Write",
                       err,
                       i2c_sensor_register_write_byte(BMP280_ADDR, BMP280_CONTROL_REG, control))

    // Do some delay to wait for init.
    vTaskDelay(100 / portTICK_PERIOD_MS);

    return ESP_OK;
}

esp_err_t bmp280_reset(void)
{
    // Init error.
    esp_err_t err = ESP_OK;

    // Reset the BMP 280.
    ESP_ERROR_VALIDATE("BMP 280 Reset Write",
                       err,
                       i2c_sensor_register_write_byte(BMP280_ADDR, BMP280_SOFTRESET_REG, MODE_SOFT_RESET_CODE))

    // Delay until reset is complete.
    vTaskDelay(100 / portTICK_PERIOD_MS);

    ESP_LOGI("BMP 280 Reset", "Reset Complete!");
    return ESP_OK;
}

esp_err_t bmp280_read_temperature(bmp280_data_out_t *output)
{
    // Init error.
    esp_err_t err = ESP_OK;

    // Get temperature data from register.
    uint8_t data[3];
    ESP_ERROR_VALIDATE("BMP 280 Temperature Register Read",
                       err,
                       i2c_sensor_register_read(BMP280_ADDR, BMP280_TEMPDATA_REG, data, 3))

    int32_t raw_temp = U8S_2_U32(0, data[0], data[1], data[2]);
    int32_t var1, var2;

    // Convert Raw temperature to valid temperature.
    raw_temp >>= 4;

    var1 = ((((raw_temp >> 3) - ((int32_t)coeffs.dig_T1 << 1))) *
            ((int32_t)coeffs.dig_T2)) >>
           11;

    var2 = (((((raw_temp >> 4) - ((int32_t)coeffs.dig_T1)) *
              ((raw_temp >> 4) - ((int32_t)coeffs.dig_T1))) >>
             12) *
            ((int32_t)coeffs.dig_T3)) >>
           14;

    t_fine = var1 + var2;

    // Save to output.
    output->temperature = (float)((t_fine * 5 + 128) >> 8) / 100.0;

    return ESP_OK;
}

esp_err_t bmp280_read_pressure(bmp280_data_out_t *output)
{
    // Init error.
    esp_err_t err = ESP_OK;

    // Get pressure data from register.
    uint8_t data[3];
    ESP_ERROR_VALIDATE("BMP 280 Pressure Register Read",
                       err,
                       i2c_sensor_register_read(BMP280_ADDR, BMP280_PRESSUREDATA_REG, data, 3))

    int32_t raw_pressure = U8S_2_U32(0, data[0], data[1], data[2]);
    int64_t var1, var2, p;

    // Convert Raw pressure to valid pressure.
    raw_pressure >>= 4;

    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)coeffs.dig_P6;
    var2 = var2 + ((var1 * (int64_t)coeffs.dig_P5) << 17);
    var2 = var2 + (((int64_t)coeffs.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)coeffs.dig_P3) >> 8) +
           ((var1 * (int64_t)coeffs.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)coeffs.dig_P1) >> 33;

    // avoid exception caused by division by zero
    if (var1 == 0)
    {
        output->pressure = 0;
    }

    p = 1048576 - raw_pressure;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)coeffs.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)coeffs.dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)coeffs.dig_P7) << 4);

    // Save to output.
    output->pressure = (float)p / 256.0;

    return ESP_OK;
}

void bmp280_read_altitude(bmp280_data_out_t *output)
{
    // Convert from pressure to altitude using Sea level HPa in Pa.
    float altitude;

    float pressure = output->pressure;
    pressure /= 100;

    altitude = 44330 * (1.0 - pow(pressure / SEALEVEL_HPA, 0.1903));

    output->altitude = altitude;
}

esp_err_t bmp280_read(bmp280_data_out_t *output)
{
    // Init error.
    esp_err_t err = ESP_OK;

    // Read temperature first to set t_fine variable!
    ESP_ERROR_VALIDATE("BMP 280 Read Temperature",
                       err,
                       bmp280_read_temperature(output))

    // Read pressure next to create conversion for altitude!
    ESP_ERROR_VALIDATE("BMP 280 Read Pressure",
                       err,
                       bmp280_read_pressure(output))

    // Read altitude last as it only requires existing values.
    bmp280_read_altitude(output);

    return ESP_OK;
}