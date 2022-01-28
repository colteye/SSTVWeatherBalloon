#include "sensor_i2c.h"
#include "error_handling.h"
#include "driver/i2c.h"
#include "hal/gpio_types.h"

#define I2C_MASTER_SCL_IO (GPIO_NUM_15) // GPIO number used for I2C master clock
#define I2C_MASTER_SDA_IO (GPIO_NUM_14) // GPIO number used for I2C master data
#define I2C_MASTER_NUM 0                // I2C master i2c port number
#define I2C_MASTER_FREQ_HZ 400000       // I2C master clock frequency
#define I2C_MASTER_TX_BUF_DISABLE 0     // I2C master doesn't need buffer
#define I2C_MASTER_RX_BUF_DISABLE 0     // I2C master doesn't need buffer
#define I2C_MASTER_TIMEOUT_MS 1000

#define I2C_TAG ("I2C")

esp_err_t i2c_master_init(void)
{
    // Init error.
    esp_err_t err = ESP_OK;

    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    // Configure I2C.
    ESP_ERROR_VALIDATE("I2C Parameter Config",
                       err,
                       i2c_param_config(i2c_master_port, &conf));

    // Install I2C for ESP32 Master.
    ESP_ERROR_VALIDATE("I2C Driver Install",
                       err,
                       i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0))

    ESP_LOGI(I2C_TAG, "I2C initialized successfully!");
    return ESP_OK;
}

esp_err_t i2c_master_deinit(void)
{
    // Init error.
    esp_err_t err = ESP_OK;

    // Delete I2C driver to deinitialize.
    ESP_ERROR_VALIDATE("I2C Driver Delete",
                       err,
                       i2c_driver_delete(I2C_MASTER_NUM))

    ESP_LOGI(I2C_TAG, "I2C deinitialized successfully!");
    return ESP_OK;
}

esp_err_t i2c_sensor_register_read(uint8_t sensor_addr, uint8_t reg_addr, uint8_t *data, size_t len)
{
    // Init error.
    esp_err_t err = ESP_OK;

    ESP_ERROR_VALIDATE("I2C Master Sensor Read",
                       err,
                       i2c_master_write_read_device(I2C_MASTER_NUM, sensor_addr, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS))
    return ESP_OK;
}

esp_err_t i2c_sensor_register_write_byte(uint8_t sensor_addr, uint8_t reg_addr, uint8_t data)
{
    // Init error.
    esp_err_t err = ESP_OK;

    uint8_t write_buf[2] = {reg_addr, data};
    ESP_ERROR_VALIDATE("I2C Master Sensor Write",
                       err,
                       i2c_master_write_to_device(I2C_MASTER_NUM, sensor_addr, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS))
    return ESP_OK;
}
