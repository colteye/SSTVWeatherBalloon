#ifndef _SENSOR_I2C_H
#define _SENSOR_I2C_H

#include "driver/i2c.h"
#include <esp_system.h>

#define I2C_MASTER_SCL_IO GPIO_NUM_15 // GPIO number used for I2C master clock
#define I2C_MASTER_SDA_IO GPIO_NUM_14 // GPIO number used for I2C master data
#define I2C_MASTER_NUM 0              // I2C master i2c port number
#define I2C_MASTER_FREQ_HZ 400000     // I2C master clock frequency
#define I2C_MASTER_TX_BUF_DISABLE 0   // I2C master doesn't need buffer
#define I2C_MASTER_RX_BUF_DISABLE 0   // I2C master doesn't need buffer
#define I2C_MASTER_TIMEOUT_MS 1000

// Initialize ESP32 as an I2C Master.
esp_err_t i2c_master_init(void);

// Deinitialize ESP32 as an I2C Master.
esp_err_t i2c_master_deinit(void);

// Read an arbitrary number of values at address from a sensor.
esp_err_t i2c_sensor_register_read(uint8_t sensor_addr, uint8_t reg_addr, uint8_t *data, size_t len);

// Write a byte of data at address from a sensor.
esp_err_t i2c_sensor_register_write_byte(uint8_t sensor_addr, uint8_t reg_addr, uint8_t data);

#endif // _SENSOR_I2C_H