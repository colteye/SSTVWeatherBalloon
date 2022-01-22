#include "mpu6050.h"
#include "error_handling.h"
#include "sensor_i2c.h"
#include "convert.h"

esp_err_t mpu6050_init(void)
{
    // Init error.
    esp_err_t err = ESP_OK;

    // Validate that the Who am I is correct.
    uint8_t who_am_i = 0;
    ESP_ERROR_VALIDATE("MPU 6050 Who am I",
                       err,
                       i2c_sensor_register_read(MPU6050_ADDR, MPU6050_WHO_AM_I_REG, &who_am_i, 1))

    ESP_ERROR_VALIDATE("MPU 6050 Who am I Validation",
                       err,
                       ((who_am_i == MPU6050_ADDR) - 1))

    // Print the chip ID.
    ESP_LOGI("MPU 6050 Initializion", "Who am I %X", who_am_i);

    // Wake up MPU6050.
    ESP_ERROR_VALIDATE("MPU 6050 Wake Up",
                       err,
                       i2c_sensor_register_write_byte(MPU6050_ADDR, MPU6050_PWR_MGMT_1_REG, 0))

    // Set sample rate divisor.
    ESP_ERROR_VALIDATE("MPU 6050 sample rate divisor Write",
                       err,
                       i2c_sensor_register_write_byte(MPU6050_ADDR, MPU6050_SMPRT_DIV_REG, 0))

    // Set filter bandwidth and FSYNC pin.
    ESP_ERROR_VALIDATE("MPU 6050 Bandwidth and FSYNC Write",
                       err,
                       i2c_sensor_register_write_byte(MPU6050_ADDR, MPU6050_CONFIG_REG, MPU6050_BAND_21_HZ))

    // Set gyroscope range.
    ESP_ERROR_VALIDATE("MPU 6050 Gyroscope Range Write",
                       err,
                       i2c_sensor_register_write_byte(MPU6050_ADDR, MPU6050_GYRO_CONFIG_REG, (MPU6050_RANGE_500_DEG << 3)))

    // Set accelerometer range.
    ESP_ERROR_VALIDATE("MPU 6050 Accelerometer Range Write",
                       err,
                       i2c_sensor_register_write_byte(MPU6050_ADDR, MPU6050_ACCEL_CONFIG_REG, (MPU6050_RANGE_8_G << 3)))

    // Set Clock to PLL with X axis gyro reference.
    ESP_ERROR_VALIDATE("MPU 6050 Clock Configuration Write",
                       err,
                       i2c_sensor_register_write_byte(MPU6050_ADDR, MPU6050_PWR_MGMT_1_REG, MPU6050_PLL_GYROX))

    // Do some delay to wait for init.
    vTaskDelay(100 / portTICK_PERIOD_MS);

    return ESP_OK;
}

esp_err_t mpu6050_reset(void)
{
    // Init error.
    esp_err_t err = ESP_OK;

    // Write reset bit.
    uint8_t data;
    ESP_ERROR_VALIDATE("MPU 6050 Reset Write",
                       err,
                       i2c_sensor_register_write_byte(MPU6050_ADDR, MPU6050_PWR_MGMT_1_REG, 1 << MPU6050_RESET_BIT))

    // Iterate until reset fully complete.
    for (;;)
    {
        ESP_ERROR_VALIDATE("MPU 6050 Reset Read",
                           err,
                           i2c_sensor_register_read(MPU6050_ADDR, MPU6050_PWR_MGMT_1_REG, &data, 1))
        // Check if reset bit is 0.
        if ((data & 0b10000000) == 0)
            break;
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    // Finish reset routine with a few delays and resetting all signal paths.
    vTaskDelay(100 / portTICK_PERIOD_MS);

    ESP_ERROR_VALIDATE("MPU 6050 Reset Finalize",
                       err,
                       i2c_sensor_register_write_byte(MPU6050_ADDR, MPU6050_SIGNAL_PATH_RESET_REG, 0x7))

    vTaskDelay(100 / portTICK_PERIOD_MS);

    ESP_LOGI("MPU 6050 Reset", "Reset Complete!");
    return ESP_OK;
}

esp_err_t mpu6050_read(mpu6050_data_out_t *output)
{
    // Init error.
    esp_err_t err = ESP_OK;

    // Collect all acceleration, gyroscope, and temperature data.
    uint8_t data[14];
    ESP_ERROR_VALIDATE("MPU 6050 Data Read",
                       err,
                       i2c_sensor_register_read(MPU6050_ADDR, MPU6050_ACCEL_OUT_REG, data, 14))

    int16_t raw_acc_x = U8S_2_U16(data[0], data[1]);
    int16_t raw_acc_y = U8S_2_U16(data[2], data[3]);
    int16_t raw_acc_z = U8S_2_U16(data[4], data[5]);

    int16_t raw_temp = U8S_2_U16(data[6], data[7]);

    int16_t raw_gyro_x = U8S_2_U16(data[8], data[9]);
    int16_t raw_gyro_y = U8S_2_U16(data[10], data[11]);
    int16_t raw_gyro_z = U8S_2_U16(data[12], data[13]);

    // Convert from raw to Celsius temperature.
    output->temperature = ((float)raw_temp / 340.0) + 36.53;

    // Access accelerometer range.
    uint8_t accel_range;
    ESP_ERROR_VALIDATE("MPU 6050 Accelerometer Range Read",
                       err,
                       i2c_sensor_register_read(MPU6050_ADDR, MPU6050_ACCEL_CONFIG_REG, &accel_range, 1))
    accel_range = (accel_range & 0b00011000) >> 3;

    // Set scaling factor based on accelerometer range.
    float accel_scale = 1.0;
    if (accel_range == MPU6050_RANGE_16_G)
        accel_scale = 2048.0;
    if (accel_range == MPU6050_RANGE_8_G)
        accel_scale = 4096.0;
    if (accel_range == MPU6050_RANGE_4_G)
        accel_scale = 8192.0;
    if (accel_range == MPU6050_RANGE_2_G)
        accel_scale = 16384.0;

    // Setup range dependant scaling.
    // Output final accelerometer values.
    // Output is meters per second ^ 2.
    output->acc_x = (((float)raw_acc_x) / accel_scale) * SENSORS_GRAVITY_EARTH;
    output->acc_y = (((float)raw_acc_y) / accel_scale) * SENSORS_GRAVITY_EARTH;
    output->acc_z = (((float)raw_acc_z) / accel_scale) * SENSORS_GRAVITY_EARTH;

    // Access gyroscope range.
    uint8_t gyro_range;
    ESP_ERROR_VALIDATE("MPU 6050 Gyroscope Range Read",
                       err,
                       i2c_sensor_register_read(MPU6050_ADDR, MPU6050_GYRO_CONFIG_REG, &gyro_range, 1))
    gyro_range = (gyro_range & 0b00011000) >> 3;

    // Set gyroscope factor based on accelerometer range.
    float gyro_scale = 1.0;
    if (gyro_range == MPU6050_RANGE_250_DEG)
        gyro_scale = 131.0;
    if (gyro_range == MPU6050_RANGE_500_DEG)
        gyro_scale = 65.5;
    if (gyro_range == MPU6050_RANGE_1000_DEG)
        gyro_scale = 32.8;
    if (gyro_range == MPU6050_RANGE_2000_DEG)
        gyro_scale = 16.4;

    // Setup range dependant scaling.
    // Output final gyroscope values.
    // Output is in radians per second.
    output->gyro_x = (((float)raw_gyro_x) / gyro_scale) * SENSORS_DPS_TO_RADS;
    output->gyro_y = (((float)raw_gyro_y) / gyro_scale) * SENSORS_DPS_TO_RADS;
    output->gyro_z = (((float)raw_gyro_z) / gyro_scale) * SENSORS_DPS_TO_RADS;

    return ESP_OK;
}