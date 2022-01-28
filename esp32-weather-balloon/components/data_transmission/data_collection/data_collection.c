void sstv_data_task_entry(void *arg)
{
    // Init error.
    esp_err_t err = ESP_OK;

    // Initialize SSTV transmission and camera recording.
    // Make sure errors do not arise!
    ESP_ERROR_REDO_VALIDATE("SSTV Initialize", sstv_init, 1000, 16);
    ESP_ERROR_REDO_VALIDATE("SSTV Camera Initialize", camera_init, 1000, 16);

    // Infinite loop for taking pictures.
    static const char *TAG = "SSTV Camera";
    for (;;)
    {
        // Clear buffer before transmitting a new message.
        // This is done first as there is no need to error check.
        sstv_buffer_clear();

        // Create buffer.
        ESP_ERROR_CONTINUE_VALIDATE("SSTV Buffer Initialize", err, 1000, sstv_buffer_init())

        // Take a picture!
        camera_fb_t *pic;
        camera_read(pic);

        ESP_LOGI(TAG, "Processing picture with SCOTTIE 1 SSTV...");

        // Generate wave data.
        ESP_ERROR_CONTINUE_VALIDATE("SSTV Generate Waveform", err, 1000, sstv_generate_waveform(pic))

        ESP_LOGI(TAG, "Finished Processing!");
        ESP_LOGI(TAG, "Transmitting SSTV signal...");

        // Start the timer to waveform data.
        current_waveform_bit = 0;

        // Delay while timer writes transmission to GPIO.
        vTaskDelay(CLIP_LENGTH * 1000 / portTICK_PERIOD_MS);

        ESP_LOGI(TAG, "Finished Transmitting!");

        // Delay before next image is captured.
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }