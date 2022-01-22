#include "mpu6050.h"
#include "error_handling.h"
#include "sensor_i2c.h"

esp_err_t mpu6050_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, MPU6050_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
}

esp_err_t mpu6050_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
}

esp_err_t mpu6050_init(void)
{
    // Init error.
    esp_err_t err = ESP_OK;

    // Wake up MPU6050.
    ESP_ERROR_VALIDATE("MPU 6050 Wake Up",
                       err,
                       mpu6050_register_write_byte(MPU6050_PWR_MGMT_1_REG, 0))

    // Set sample rate divisor.
    ESP_ERROR_VALIDATE("MPU 6050 sample rate divisor Write",
                       err,
                       mpu6050_register_write_byte(MPU6050_SMPRT_DIV_REG, 0))

    // Set filter bandwidth and FSYNC pin.
    ESP_ERROR_VALIDATE("MPU 6050 Bandwidth and FSYNC Write",
                       err,
                       mpu6050_register_write_byte(MPU6050_CONFIG_REG, MPU6050_BAND_21_HZ))

    // Set gyroscope range.
    ESP_ERROR_VALIDATE("MPU 6050 Gyroscope Range Write",
                       err,
                       mpu6050_register_write_byte(MPU6050_GYRO_CONFIG_REG, (MPU6050_RANGE_500_DEG << 3)))

    // Set accelerometer range.
    ESP_ERROR_VALIDATE("MPU 6050 Accelerometer Range Write",
                       err,
                       mpu6050_register_write_byte(MPU6050_ACCEL_CONFIG_REG, (MPU6050_RANGE_8_G << 3)))

    // Set Clock to PLL with X axis gyro reference.
    ESP_ERROR_VALIDATE("MPU 6050 Clock Configuration Write",
                       err,
                       mpu6050_register_write_byte(MPU6050_PWR_MGMT_1_REG, MPU6050_PLL_GYROX))

    vTaskDelay(100 / portTICK_PERIOD_MS);

    uint8_t who_am_i = 0;
    ESP_ERROR_VALIDATE("MPU 6050 Who am I", err, mpu6050_register_read(MPU6050_WHO_AM_I_REG, &who_am_i, 1))
    ESP_LOGI("Init", "Who am I? %X", who_am_i);
    return ESP_OK;
}

esp_err_t mpu6050_reset(void)
{
    // Init error.
    esp_err_t err = ESP_OK;

    uint8_t data;
    ESP_ERROR_VALIDATE("MPU 6050 Reset Write",
                       err,
                       mpu6050_register_write_byte(MPU6050_PWR_MGMT_1_REG, 1 << MPU6050_RESET_BIT))
    for (;;)
    {
        ESP_ERROR_VALIDATE("MPU 6050 Reset Read",
                           err,
                           mpu6050_register_read(MPU6050_PWR_MGMT_1_REG, &data, 1))
        if ((data & 0b10000000) == 0)
            break;
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);

    ESP_ERROR_VALIDATE("MPU 6050 Reset Finalize",
                       err,
                       mpu6050_register_write_byte(MPU6050_SIGNAL_PATH_RESET_REG, 0x7))

    vTaskDelay(100 / portTICK_PERIOD_MS);

    ESP_LOGI("MPU 6050 Reset", "Reset Complete!");
    return ESP_OK;
}

esp_err_t mpu6050_read(mpu6050_data_out_t *output)
{
    // Init error.
    esp_err_t err = ESP_OK;

    // Wake up MPU6050.
    uint8_t data[14];
    ESP_ERROR_VALIDATE("MPU 6050 Data Read",
                       err,
                       mpu6050_register_read(MPU6050_ACCEL_OUT_REG, data, 14))

    int16_t raw_acc_x = ((int16_t)data[0] << 8) | data[1];
    int16_t raw_acc_y = ((int16_t)data[2] << 8) | data[3];
    int16_t raw_acc_z = ((int16_t)data[4] << 8) | data[5];

    int16_t raw_temp = ((int16_t)data[6] << 8) | data[7];

    int16_t raw_gyro_x = ((int16_t)data[8] << 8) | data[9];
    int16_t raw_gyro_y = ((int16_t)data[10] << 8) | data[11];
    int16_t raw_gyro_z = ((int16_t)data[12] << 8) | data[13];

    output->temperature = ((float)raw_temp / 340.0) + 36.53;
    // Access accelerometer range.
    uint8_t accel_range;
    ESP_ERROR_VALIDATE("MPU 6050 Accelerometer Range Read",
                       err,
                       mpu6050_register_read(MPU6050_ACCEL_CONFIG_REG, &accel_range, 1))
    accel_range = (accel_range & 0b00011000) >> 3;

    float accel_scale = 1.0;
    if (accel_range == MPU6050_RANGE_16_G)
        accel_scale = 2048.0;
    if (accel_range == MPU6050_RANGE_8_G)
        accel_scale = 4096.0;
    if (accel_range == MPU6050_RANGE_4_G)
        accel_scale = 8192.0;
    if (accel_range == MPU6050_RANGE_2_G)
        accel_scale = 16384.0;

    // setup range dependant scaling
    output->acc_x = (((float)raw_acc_x) / accel_scale) * SENSORS_GRAVITY_EARTH;
    output->acc_y = (((float)raw_acc_y) / accel_scale) * SENSORS_GRAVITY_EARTH;
    output->acc_z = (((float)raw_acc_z) / accel_scale) * SENSORS_GRAVITY_EARTH;

    // Access gyroscope range.
    uint8_t gyro_range;
    ESP_ERROR_VALIDATE("MPU 6050 Gyroscope Range Read",
                       err,
                       mpu6050_register_read(MPU6050_GYRO_CONFIG_REG, &gyro_range, 1))
    gyro_range = (gyro_range & 0b00011000) >> 3;

    float gyro_scale = 1.0;
    if (gyro_range == MPU6050_RANGE_250_DEG)
        gyro_scale = 131.0;
    if (gyro_range == MPU6050_RANGE_500_DEG)
        gyro_scale = 65.5;
    if (gyro_range == MPU6050_RANGE_1000_DEG)
        gyro_scale = 32.8;
    if (gyro_range == MPU6050_RANGE_2000_DEG)
        gyro_scale = 16.4;

    output->gyro_x = (((float)raw_gyro_x) / gyro_scale) * SENSORS_DPS_TO_RADS;
    output->gyro_y = (((float)raw_gyro_y) / gyro_scale) * SENSORS_DPS_TO_RADS;
    output->gyro_z = (((float)raw_gyro_z) / gyro_scale) * SENSORS_DPS_TO_RADS;

    return ESP_OK;
}