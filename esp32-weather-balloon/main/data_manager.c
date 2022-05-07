#include "ax25.h"
#include "sstv.h"
#include "sensor_i2c.h"
#include "camera.h"
#include "gps.h"
#include "bmp280.h"
#include "mpu6050.h"
#include "radio_transmitter.h"
#include "error_handling.h"

#define SSTV_TAG ("SSTV")
#define AX25_TAG ("AX.25")

#define DATA_MANAGER_STACK_SIZE 4096

typedef struct
{
    gps_data_out_t gps_data;
    bmp280_data_out_t bmp280_data;
    mpu6050_data_out_t mpu6050_data;

} sensor_data_t;

uint8_t camera_transmit_status;
xSemaphoreHandle camera_transmit_mutex;

// Initialize radio waveform buffer.
esp_err_t waveform_buffer_init(radio_waveform_data_t *waveform, uint32_t len)
{
    // Use calloc to start everything at 0.
    waveform->buffer = (uint8_t *)calloc(len, sizeof(uint8_t));

    if (!waveform->buffer)
        return ESP_ERR_NO_MEM;

    return ESP_OK;
}

// Set all waveform buffer values to 0.
esp_err_t waveform_buffer_clear(radio_waveform_data_t *waveform, uint32_t len)
{
    if (!waveform->buffer)
        return ESP_ERR_NO_MEM;

    for (uint32_t i = 0; i < len; ++i)
    {
        waveform->buffer[i] = 0;
    }
    return ESP_OK;
}

// Free data used by buffer.
void waveform_buffer_deinit(radio_waveform_data_t *waveform)
{
    // Free buffer pointer.
    free(waveform->buffer);
}

void data_manager_task_entry(void *arg)
{
    // Init error.
    esp_err_t err = ESP_OK;

    radio_transmitter_config_t *radio_config = (radio_transmitter_config_t *)arg;

    // Camera data waveform (SSTV)!
    // SSTV messages take about 110 seconds to run completely.
    uint8_t sstv_seconds = 110;
    uint32_t sstv_buf_len = (radio_config->sample_rate * radio_config->bits_per_sample * sstv_seconds) / BYTE_SIZE;

    radio_waveform_data_t sstv_buf = {.current_osc = 0.25,
                                      .current_transmit_bit = 0xFFFFFFFF,
                                      .current_offset_bit = 0,

                                      .clip_seconds = sstv_seconds,
                                      .buffer_bits_len = sstv_buf_len * BYTE_SIZE,
                                      .buffer = NULL};

    // Sensor data waveform (AX.25)!
    // AX.25 messages typically only take 1 second with current data.
    uint8_t ax25_seconds = 1;
    uint32_t ax25_buf_len = (radio_config->sample_rate * radio_config->bits_per_sample * ax25_seconds) / BYTE_SIZE;

    radio_waveform_data_t ax25_buf = {.current_osc = 0.25,
                                      .current_transmit_bit = 0xFFFFFFFF,
                                      .current_offset_bit = 0,

                                      .clip_seconds = ax25_seconds,
                                      .buffer_bits_len = ax25_buf_len * BYTE_SIZE,
                                      .buffer = NULL};

    // Try to initialize different sensing/comms systems!

    /*i2c_master_init();
    camera_init();
    gps_init();
    mpu6050_init();
    bmp280_init();*/

    ESP_ERROR_REDO_TASK_VALIDATE("I2C Initialize", i2c_master_init, 100, 16);

    ESP_ERROR_REDO_TASK_VALIDATE("Camera Initialize", camera_init, 100, 16);

    ESP_ERROR_REDO_TASK_VALIDATE("GPS Initialize", gps_init, 100, 16);
    ESP_ERROR_REDO_TASK_VALIDATE("MPU6050 Initialize", mpu6050_init, 100, 16);
    ESP_ERROR_REDO_TASK_VALIDATE("BMP280 Initialize", bmp280_init, 100, 16);
    sensor_data_t sensors_data;

    // Initialize buffers.
    ESP_ERROR_TASK_VALIDATE("AX25 Buffer Initialize", err, waveform_buffer_init(&ax25_buf, ax25_buf_len))
    ESP_ERROR_TASK_VALIDATE("SSTV Buffer Initialize", err, waveform_buffer_init(&sstv_buf, sstv_buf_len))

    // Get waveform queue and mutex from radio transmitter.
    xSemaphoreHandle radio_busy_mutex = radio_transmitter_get_mutex();
    xQueueHandle waveform_queue = radio_transmitter_get_waveform_queue();

    for (;;)
    {
        // Take radio mutex once radio is no long busy!
        // In this achitecture, the radio is the one that notifies tasks that it is free.
        while (xSemaphoreTake(radio_busy_mutex, pdMS_TO_TICKS(1000)) == pdFALSE)
            ;

        // -------------------------------- //
        // CREATE SENSOR AX25 WAVEFORM DATA //
        // -------------------------------- //

        // Reset SSTV buffer variables and take a picture!
        ax25_buf.current_osc = 0.25;
        ax25_buf.current_transmit_bit = 0xFFFFFFFF;
        ax25_buf.current_offset_bit = 0;
        ESP_ERROR_TASK_VALIDATE("AX25 Buffer Clear", err, waveform_buffer_clear(&ax25_buf, ax25_buf_len))

        mpu6050_read(&sensors_data.mpu6050_data);
        ESP_LOGI("MPU6050 Read", "TEMP: %.6f", sensors_data.mpu6050_data.temperature);
        ESP_LOGI("MPU6050 Read", "ROTATION X: %.6f", sensors_data.mpu6050_data.gyro_x);
        ESP_LOGI("MPU6050 Read", "ROTATION Y: %.6f", sensors_data.mpu6050_data.gyro_y);
        ESP_LOGI("MPU6050 Read", "ROTATION Z: %.6f\n", sensors_data.mpu6050_data.gyro_z);
        ESP_LOGI("MPU6050 Read", "ACCEL X: %.6f", sensors_data.mpu6050_data.acc_x);
        ESP_LOGI("MPU6050 Read", "ACCEL Y: %.6f", sensors_data.mpu6050_data.acc_y);
        ESP_LOGI("MPU6050 Read", "ACCEL Z: %.6f\n", sensors_data.mpu6050_data.acc_z);

        bmp280_read(&sensors_data.bmp280_data);
        ESP_LOGI("BMP280 Read", "TEMP: %.6f", sensors_data.bmp280_data.temperature);
        ESP_LOGI("BMP280 Read", "PRESSURE: %.6f", sensors_data.bmp280_data.pressure);
        ESP_LOGI("BMP280 Read", "ALTITUDE: %.6f\n", sensors_data.bmp280_data.altitude);

        gps_read(&sensors_data.gps_data);
        ESP_LOGI("GPS Read", "LATITUDE: %.6f", sensors_data.gps_data.latitude);
        ESP_LOGI("GPS Read", "LONGITUDE: %.6f", sensors_data.gps_data.longitude);
        ESP_LOGI("GPS Read", "ALTITUDE: %.6f", sensors_data.gps_data.altitude);
        ESP_LOGI("GPS Read", "SPEED: %.6f\n", sensors_data.gps_data.speed);

        // Generate wave data.
        // Uses sizeof, need to make sure struct packing is correct for the reciever!
        // (ideally cast to C struct)
        ESP_LOGI(AX25_TAG, "Generating Sensor AX.25 Waveform...");
        ESP_ERROR_CONTINUE_VALIDATE("SSTV Generate Waveform",
                                    err,
                                    0,
                                    ax25_generate_waveform((uint8_t *)&sensors_data,
                                                           sizeof(sensor_data_t),
                                                           &ax25_buf))

        // -------------------------------- //
        // CREATE CAMERA SSTV WAVEFORM DATA //
        // -------------------------------- //

        // Reset SSTV buffer variables and take a picture!
        sstv_buf.current_osc = 0.25;
        sstv_buf.current_transmit_bit = 0xFFFFFFFF;
        sstv_buf.current_offset_bit = 0;
        ESP_ERROR_TASK_VALIDATE("SSTV Buffer Clear", err, waveform_buffer_clear(&sstv_buf, sstv_buf_len))

        camera_fb_t *pic = esp_camera_fb_get();
        ESP_LOGI(SSTV_TAG, "Picture taken! Its size was: %zu bytes", pic->len);
        esp_camera_fb_return(pic);

        // Generate wave data.
        ESP_LOGI(SSTV_TAG, "Generating SCOTTIE 1 SSTV Waveform...");
        ESP_ERROR_CONTINUE_VALIDATE("SSTV Generate Waveform",
                                    err,
                                    0,
                                    sstv_generate_waveform(pic, &sstv_buf))

        // ---------------------- //
        // TRANSMIT WAVEFORM DATA //
        // ---------------------- //

        // Send waveforms to radio queue!
        ESP_LOGI(AX25_TAG, "Transmitting Sensor AX.25 signal...");
        xQueueSend(waveform_queue, (void *)&ax25_buf, pdMS_TO_TICKS(200));

        ESP_LOGI(SSTV_TAG, "Transmitting Camera SSTV signal...");
        xQueueSend(waveform_queue, (void *)&sstv_buf, pdMS_TO_TICKS(200));

        // Infinite loop delay.
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }

    // If finished, delete buffers and delete task.
    waveform_buffer_deinit(&sstv_buf);
    waveform_buffer_deinit(&ax25_buf);

    vTaskDelete(NULL);
}

esp_err_t data_manager_init()
{
    // Init error.
    esp_err_t err = ESP_OK;

    // Initialize radio system.
    radio_transmitter_config_t radio_config = {.pwm_range_addr = R_PWM_RANGE_ADDR,
                                               .num_pwm_levels = R_NUM_PWM_LEVELS,
                                               .bits_per_sample = R_BITS_PER_SAMPLE,
                                               .sample_rate = R_SAMPLE_RATE,
                                               .timer = R_WAVEFORM_TIMER,
                                               .timer_divider = R_WAVEFORM_TIMER_DIVIDER,
                                               .gpio = R_GPIO};

    ESP_ERROR_VALIDATE("Data Manager Radio Init", err, radio_transmitter_init(radio_config))

    // Create tasks to collect data from all device sensors.
    BaseType_t xReturned;
    xTaskHandle xDataHandle = NULL;

    // Create data collection task.
    xReturned = xTaskCreatePinnedToCore(
        data_manager_task_entry,                // Function that implements the task.
        "Data Management",                      // Text name for the task.
        DATA_MANAGER_STACK_SIZE,                // Stack size in words, not bytes.
        (void *)radio_transmitter_get_config(), // Parameter passed into the task.
        tskIDLE_PRIORITY,                       // Priority at which the task is created.
        &xDataHandle,                           // Used to pass out the created task's handle.
        1);

    if (xReturned != pdPASS)
        return ESP_ERR_NO_MEM;

    return ESP_OK;
}

esp_err_t data_manager_deinit()
{
    return ESP_OK;
}