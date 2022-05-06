#include "gps.h"
#include "camera.h"
#include "sstv.h"
#include "bmp280.h"
#include "mpu6050.h"
#include "sensor_i2c.h"
#include "radio_transmitter.h"
#include "error_handling.h"

#define SSTV_TAG ("SSTV")
#define AX25_TAG ("AX.25")

#define DATA_MANAGER_STACK_SIZE 4096

uint8_t sstv_transmit_status;
xSemaphoreHandle sstv_transmit_mutex;

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

void data_manager_ax25_task_entry(void *arg)
{
    // Init error.
    esp_err_t err = ESP_OK;

    // Create full AX25 buffer struct.
    radio_transmitter_config_t *radio_config = (radio_transmitter_config_t *)arg;
    uint8_t ax25_seconds = 5;
    uint32_t ax25_buf_len = (radio_config->sample_rate * radio_config->bits_per_sample * ax25_seconds) / BYTE_SIZE;

    radio_waveform_data_t ax25_buf = {.current_osc = 0.25,
                                      .current_transmit_bit = 0xFFFFFFFF,
                                      .current_offset_bit = 0,

                                      .clip_seconds = ax25_seconds,
                                      .buffer_bits_len = ax25_buf_len * BYTE_SIZE,
                                      .buffer = NULL};

    // Initialize buffer.
    ESP_ERROR_TASK_VALIDATE("AX25 Buffer Initialize", err, waveform_buffer_init(&ax25_buf, ax25_buf_len))

    i2c_master_init();
    gps_init();
    mpu6050_init();
    bmp280_init();
    gps_data_out_t gps_data;
    bmp280_data_out_t bmp280_data;
    mpu6050_data_out_t mpu6050_data;

    // Get waveform queue and mutex from radio transmitter.
    xSemaphoreHandle radio_busy_mutex = radio_transmitter_get_mutex();
    xQueueHandle waveform_queue = radio_transmitter_get_waveform_queue();

    uint8_t current_sstv_transmit_status = 1;
    for (;;)
    {
        // Read SSTV transmit status controlled by semaphore.
        while (xSemaphoreTake(sstv_transmit_mutex, pdMS_TO_TICKS(1000)) == pdFALSE)
            ;
        current_sstv_transmit_status = sstv_transmit_status;
        xSemaphoreGive(sstv_transmit_mutex);

        // If not transmitting, read data!
        if (!current_sstv_transmit_status)
        {
            ESP_LOGI("MPU6050 Read", "TEMP");
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
            ESP_LOGI("BMP280 Read", "ALTITUDE: %.6f\n", bmp280_data.altitude);

            gps_read(&gps_data);
            ESP_LOGI("GPS Read", "LATITUDE: %.6f", gps_data.latitude);
            ESP_LOGI("GPS Read", "LONGITUDE: %.6f", gps_data.longitude);
            ESP_LOGI("GPS Read", "ALTITUDE: %.6f", gps_data.altitude);
            ESP_LOGI("GPS Read", "SPEED: %.6f\n", gps_data.speed);
        }

        // Add a bit of delay between each measurement.
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void data_manager_sstv_task_entry(void *arg)
{
    // Init error.
    esp_err_t err = ESP_OK;

    // Create full SSTV buffer struct.
    radio_transmitter_config_t *radio_config = (radio_transmitter_config_t *)arg;
    uint8_t sstv_seconds = 110;
    uint32_t sstv_buf_len = (radio_config->sample_rate * radio_config->bits_per_sample * sstv_seconds) / BYTE_SIZE;

    radio_waveform_data_t sstv_buf = {.current_osc = 0.25,
                                      .current_transmit_bit = 0xFFFFFFFF,
                                      .current_offset_bit = 0,

                                      .clip_seconds = sstv_seconds,
                                      .buffer_bits_len = sstv_buf_len * BYTE_SIZE,
                                      .buffer = NULL};

    // Initialize buffer and camera.
    // Try to redo but if it takes too long, delete task.
    ESP_ERROR_REDO_TASK_VALIDATE("SSTV Camera Initialize", camera_init, 1000, 16);
    ESP_ERROR_TASK_VALIDATE("SSTV Buffer Initialize", err, waveform_buffer_init(&sstv_buf, sstv_buf_len))

    // Get waveform queue and mutex from radio transmitter.
    xSemaphoreHandle radio_busy_mutex = radio_transmitter_get_mutex();
    xQueueHandle waveform_queue = radio_transmitter_get_waveform_queue();

    for (;;)
    {
        // Send to other tasks that sstv is not transmitting.
        // Set SSTV transmit status to 0, notify other tasks.
        while (xSemaphoreTake(sstv_transmit_mutex, pdMS_TO_TICKS(1000)) == pdFALSE)
            ;
        sstv_transmit_status = 0;
        xSemaphoreGive(sstv_transmit_mutex);

        // Reset SSTV buffer variables and take a picture!
        sstv_buf.current_osc = 0.25;
        sstv_buf.current_transmit_bit = 0xFFFFFFFF;
        sstv_buf.current_offset_bit = 0;
        ESP_ERROR_TASK_VALIDATE("SSTV Buffer Clear", err, waveform_buffer_clear(&sstv_buf, sstv_buf_len))

        camera_fb_t *pic = esp_camera_fb_get();
        ESP_LOGI(SSTV_TAG, "Picture taken! Its size was: %zu bytes", pic->len);
        esp_camera_fb_return(pic);

        // Add a bit of delay before to space out signals.
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        // Generate wave data.
        ESP_LOGI(SSTV_TAG, "Generating SCOTTIE 1 SSTV Waveform...");
        ESP_ERROR_CONTINUE_VALIDATE("SSTV Generate Waveform", err, 0, sstv_generate_waveform(pic, &sstv_buf))

        // Set SSTV transmit status to 1, notify other tasks.
        // Set SSTV transmit status to 0, notify other tasks.

        while (xSemaphoreTake(sstv_transmit_mutex, pdMS_TO_TICKS(1000)) == pdFALSE)
            ;
        sstv_transmit_status = 1;
        xSemaphoreGive(sstv_transmit_mutex);

        // Wait until semaphore can be taken.
        // Nothing else should be playing!
        while (xSemaphoreTake(radio_busy_mutex, pdMS_TO_TICKS(1000)) == pdFALSE)
            ;

        // Send data to radio queue to be transmitted.
        ESP_LOGI(SSTV_TAG, "Transmitting SSTV signal...");
        xQueueSend(waveform_queue, (void *)&sstv_buf, pdMS_TO_TICKS(200));

        // Dont process new SSTV image until previous is done transmitting.
        vTaskDelay(sstv_seconds * 1000 / portTICK_PERIOD_MS);

        // Give back the mutex after delay, transmission should be done.
        // If not, this provides a hard stop to continue the program.
        xSemaphoreGive(radio_busy_mutex);
    }

    // If finished, delete buffer and delete task.
    waveform_buffer_deinit(&sstv_buf);
    vTaskDelete(NULL);
}

esp_err_t data_manager_init()
{
    // Init error.
    esp_err_t err = ESP_OK;

    // Mutex to read or modify SSTV transmit state.
    sstv_transmit_status = 0;
    sstv_transmit_mutex = xSemaphoreCreateMutex();
    xSemaphoreGive(sstv_transmit_mutex);

    // Initialize radio system.
    radio_transmitter_config_t radio_config = {.pwm_range_addr = PWM_RANGE_ADDR,
                                               .num_pwm_levels = NUM_PWM_LEVELS,
                                               .bits_per_sample = BITS_PER_SAMPLE,
                                               .sample_rate = SAMPLE_RATE,
                                               .timer = WAVEFORM_TIMER,
                                               .timer_divider = WAVEFORM_TIMER_DIVIDER,
                                               .gpio = RADIO_GPIO};

    ESP_ERROR_VALIDATE("Data Manager Radio Init", err, radio_transmitter_init(radio_config))

    // Create tasks to collect data from all device sensors.
    BaseType_t xReturned;
    xTaskHandle xSSTVHandle = NULL;
    xTaskHandle xAX25Handle = NULL;

    // Create data collection tasks.
    xReturned = xTaskCreatePinnedToCore(
        data_manager_ax25_task_entry,
        "AX25 Sensor Data Management",
        DATA_MANAGER_STACK_SIZE,
        (void *)radio_transmitter_get_config(),
        tskIDLE_PRIORITY,
        &xAX25Handle,
        1);

    if (xReturned != pdPASS)
        return ESP_ERR_NO_MEM;

    xReturned = xTaskCreatePinnedToCore(
        data_manager_sstv_task_entry,           // Function that implements the task.
        "SSTV Camera Data Management",          // Text name for the task.
        DATA_MANAGER_STACK_SIZE,                // Stack size in words, not bytes.
        (void *)radio_transmitter_get_config(), // Parameter passed into the task.
        tskIDLE_PRIORITY,                       // Priority at which the task is created.
        &xSSTVHandle,                           // Used to pass out the created task's handle.
        1);

    if (xReturned != pdPASS)
        return ESP_ERR_NO_MEM;

    return ESP_OK;
}

esp_err_t data_manager_deinit()
{
    return ESP_OK;
}