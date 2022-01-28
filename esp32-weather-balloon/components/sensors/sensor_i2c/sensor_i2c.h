#ifndef _SENSOR_I2C_H
#define _SENSOR_I2C_H

#include <esp_system.h>

// Initialize ESP32 as an I2C Master.
esp_err_t i2c_master_init(void);

// Deinitialize ESP32 as an I2C Master.
esp_err_t i2c_master_deinit(void);

// Read an arbitrary number of values at address from a sensor.
esp_err_t i2c_sensor_register_read(uint8_t sensor_addr, uint8_t reg_addr, uint8_t *data, size_t len);

// Write a byte of data at address from a sensor.
esp_err_t i2c_sensor_register_write_byte(uint8_t sensor_addr, uint8_t reg_addr, uint8_t data);

#endif // _SENSOR_I2C_H