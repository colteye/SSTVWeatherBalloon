#ifndef _MPU6050_H
#define _MPU6050_H

#include <esp_system.h>

// Data collected by MPU6050.
typedef struct
{
    float temperature; // Temperature (Celsius)
    float acc_x;       // X Axis Acceleration (meters/(second ^ 2))
    float acc_y;       // Y Axis Acceleration (meters/(second ^ 2))
    float acc_z;       // Z Axis Acceleration (meters/(second ^ 2))
    float gyro_x;      // X Axis Gyroscope (radians/second)
    float gyro_y;      // Y Axis Gyroscope (radians/second)
    float gyro_z;      // Z Axis Gyroscope (radians/second)
} mpu6050_data_out_t;

// Initialize the MPU6050 chip.
esp_err_t mpu6050_init(void);

// Reset the MPU6050 chip.
esp_err_t mpu6050_reset(void);

// Reset all of the data from the MPU6050 chip.
esp_err_t mpu6050_read(mpu6050_data_out_t *data);

#endif // _MPU6050_H