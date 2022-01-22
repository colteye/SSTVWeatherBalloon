
// Copyright (c) 2022-2022 Erik Coltey
//
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to
// the following conditions:
//
// The above copyright notice and this permission notice shall be
// included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
// NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
// LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
// OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
// WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

// ------------------------------------------------------------------------- //
// Entry point for ESP32 weather balloon code.                               //
// ------------------------------------------------------------------------- //

#include <esp_log.h>
#include <esp_system.h>
#include <sys/param.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "sstv.h"
#include "bmp280.h"
#include "mpu6050.h"
#include "sensor_i2c.h"

void app_main()
{
    i2c_master_init();
    mpu6050_init();
    bmp280_init();
    bmp280_data_out_t bmp280_data;
    mpu6050_data_out_t mpu6050_data;
    for (int i = 0; i < 100; ++i)
    {
        mpu6050_read(&mpu6050_data);
        ESP_LOGI("MPU6050 Read", "TEMP: %.6f", mpu6050_data.temperature);

        ESP_LOGI("MPU6050 Read", "ROTATION X: %.6f", mpu6050_data.gyro_x);
        ESP_LOGI("MPU6050 Read", "ROTATION Y: %.6f", mpu6050_data.gyro_y);
        ESP_LOGI("MPU6050 Read", "ROTATION Z: %.6f\n", mpu6050_data.gyro_z);

        ESP_LOGI("MPU6050 Read", "ACCEL X: %.6f", mpu6050_data.acc_x);
        ESP_LOGI("MPU6050 Read", "ACCEL Y: %.6f", mpu6050_data.acc_y);
        ESP_LOGI("MPU6050 Read", "ACCEL Z: %.6f\n", mpu6050_data.acc_z);

        bmp280_read(&bmp280_data);
        ESP_LOGI("BMP280 Read", "TEMP: %.6f", bmp280_data.temperature);
        ESP_LOGI("BMP280 Read", "PRESSURE: %.6f", bmp280_data.pressure);
        ESP_LOGI("BMP280 Read", "ALTITUDE: %.6f", bmp280_data.altitude);

        vTaskDelay(1000);
    }
    i2c_master_deinit();

    // Create task for taking SSTV pictures.
    xTaskCreate(SSTVCameraServiceTask, "SSTV Camera Service", 4096, NULL, tskIDLE_PRIORITY, NULL);
}
