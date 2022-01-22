#include "sensor_i2c.h"
#include "error_handling.h"

const char *TAG = "Sensor I2C";

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

    ESP_ERROR_VALIDATE("I2C Parameter Config",
                       err,
                       i2c_param_config(i2c_master_port, &conf));

    ESP_ERROR_VALIDATE("I2C Driver Install",
                       err,
                       i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0))

    ESP_LOGI(TAG, "I2C initialized successfully!");
    return ESP_OK;
}

esp_err_t i2c_master_deinit(void)
{
    // Init error.
    esp_err_t err = ESP_OK;

    ESP_ERROR_VALIDATE("I2C Driver Delete",
                       err,
                       i2c_driver_delete(I2C_MASTER_NUM))

    ESP_LOGI(TAG, "I2C deinitialized successfully!");
    return ESP_OK;
}