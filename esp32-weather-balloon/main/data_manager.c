#include "gps.h"
#include "camera.h"
#include "sstv.h"
#include "bmp280.h"
#include "mpu6050.h"
#include "sensor_i2c.h"
#include "radio_transmitter.h"
#include "error_handling.h"

// Waveform related defines
#define BITS_PER_SAMPLE (2)
#define SAMPLE_RATE (40000)
#define WAVEFORM_GPIO (GPIO_NUM_2)

#define NUM_PWM_LEVELS (BITS_PER_SAMPLE + 1)
#define PWM_RANGE_ADDR (SINE_MAX - SINE_MIN) / (float)NUM_PWM_LEVELS

#define WAVEFORM_TIMER_DIVIDER (APB_CLK_FREQ / (SAMPLE_RATE * BITS_PER_SAMPLE)) //Subtract to try to compensate for extra operations being done?
#define WAVEFORM_TIMER (1)

#define SSTV_TAG ("SSTV")
#define AX25_TAG ("AX.25")

#define DATA_MANAGER_STACK_SIZE 4096

xQueueHandle sstv_transmit_queue;

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

/*static void data_manager_ax25_task_entry(void *arg)
{
    radio_transmitter_config_t *radio_config = (radio_transmitter_config_t *)arg;
    uint8_t ax25_seconds = 5;

    uint8_t *ax25_buf_ptr;
    uint8_t ax25_buf_len = (radio_config->sample_rate * radio_config->bits_per_sample * ax25_seconds) / BYTE_SIZE;

    buffer_init(ax25_buf_ptr, ax25_buf_len);

    radio_waveform_data_t ax25_buf = {current_osc = 0.25,
                                      current_transmit_bit = 0xFFFFFFFF,
                                      current_offset_bit = 0,

                                      clip_seconds = ax25_seconds,
                                      buffer_bits_len = radio_config->sample_rate *
                                                        radio_config->bits_per_sample *
                                                        ax25_seconds,
                                      buffer = ax25_buf_ptr};

    QueueHandle_t *waveform_queue = radio_transmitter_get_waveform_queue();

    // Initialize buffer and camera!
    // Try to redo but if it takes too long, abort.
    ESP_ERROR_REDO_VALIDATE("SSTV Buffer Initialize", buffer_init(sstv_buf_ptr, sstv_buf_len), 1000, 16);
    ESP_ERROR_REDO_VALIDATE("SSTV Camera Initialize", camera_init, 1000, 16);

    i2c_master_init();
    gps_init();
    mpu6050_init();
    bmp280_init();

    for (;;)
    {
    }
}*/

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
        //xQueueSend(sstv_transmit_queue, (void *)0, pdMS_TO_TICKS(200));

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

        // Send to other tasks that sstv is transmitting.
        //xQueueSend(sstv_transmit_queue, (void *)1, pdMS_TO_TICKS(200));

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

    // Queue to send SSTV transmit state changes to other data writing tasks.
    sstv_transmit_queue = xQueueCreate(1, sizeof(uint8_t));

    // Initialize radio system.
    radio_transmitter_config_t radio_config = {.pwm_range_addr = PWM_RANGE_ADDR,
                                               .num_pwm_levels = NUM_PWM_LEVELS,
                                               .bits_per_sample = BITS_PER_SAMPLE,
                                               .sample_rate = SAMPLE_RATE,
                                               .timer = WAVEFORM_TIMER,
                                               .timer_divider = WAVEFORM_TIMER_DIVIDER,
                                               .gpio = WAVEFORM_GPIO};

    ESP_ERROR_VALIDATE("Data Manager Radio Init", err, radio_transmitter_init(radio_config))

    // Create tasks to collect data from all device sensors.
    BaseType_t xReturned;
    xTaskHandle xSSTVHandle = NULL;
    xTaskHandle xAX25Handle = NULL;

    // Create data collection tasks.
    xReturned = xTaskCreatePinnedToCore(
        data_manager_sstv_task_entry,           // Function that implements the task.
        "SSTV Camera Data Management",          // Text name for the task.
        DATA_MANAGER_STACK_SIZE,                // Stack size in words, not bytes.
        (void *)radio_transmitter_get_config(), // Parameter passed into the task.
        tskIDLE_PRIORITY,                       // Priority at which the task is created.
        &xSSTVHandle,
        1); // Used to pass out the created task's handle.

    if (xReturned != pdPASS)
        return ESP_ERR_NO_MEM;

    /* Create the task, storing the handle. 
    xReturned = xTaskCreate(
        data_manager_ax25_task_entry,
        "AX25 Sensor Data Management",
        DATA_MANAGER_STACK_SIZE,
        (void *)radio_transmitter_get_config(),
        tskIDLE_PRIORITY,
        &xAX25Handle);

        if (xReturned != pdPASS)
            return ESP_ERR_NO_MEM;
            */
    return ESP_OK;
}

esp_err_t data_manager_deinit()
{
    return ESP_OK;
}