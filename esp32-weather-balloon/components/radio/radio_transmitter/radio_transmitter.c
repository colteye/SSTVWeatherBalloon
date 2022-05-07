#include "radio_transmitter.h"
#include "error_handling.h"

#include "driver/gpio.h"
#include "driver/timer.h"

#define RADIO_TRANSMITTER_STACK_SIZE (4096)
#define RADIO_TRANSMITTER_TAG ("Radio Transmitter")

typedef struct
{
    radio_transmitter_config_t config;
    radio_waveform_data_t waveform;
    xQueueHandle waveform_queue; // Radio waveform queue handle
    xSemaphoreHandle radio_busy_mutex;
    uint8_t transmitting;

} radio_state_t;

static radio_state_t *radio_state = NULL;

// Check range of a value between 2 values
#define IN_RANGE(low, high, x) (low <= x && x <= high)

radio_transmitter_config_t *radio_transmitter_get_config(void)
{
    return &radio_state->config;
}

xSemaphoreHandle radio_transmitter_get_mutex(void)
{
    return radio_state->radio_busy_mutex;
}

xQueueHandle radio_transmitter_get_waveform_queue(void)
{
    return radio_state->waveform_queue;
}

static void radio_transmitter_task_entry(void *arg)
{

    radio_waveform_data_t current_waveform;
    for (;;)
    {
        // if there is an item in the queue, transmit it!
        if (xQueueReceive(radio_state->waveform_queue, &current_waveform, pdMS_TO_TICKS(200)))
        {
            ESP_LOGI(RADIO_TRANSMITTER_TAG, "Transmitting data!");

            // Begin transmission!
            radio_state->waveform = current_waveform;
            radio_state->transmitting = 1;
            radio_state->waveform.current_transmit_bit = 0;
        }
        else
        {
            if (!uxQueueMessagesWaiting(radio_state->waveform_queue) && !radio_state->transmitting)
            {
                if (xSemaphoreTake(radio_state->radio_busy_mutex, pdMS_TO_TICKS(1000)) == pdFALSE)
                {
                    // Tell other processes that radio is no longer busy.
                    xSemaphoreGive(radio_state->radio_busy_mutex);
                }
            }
        }
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

// Write a frequency for a specific number of time into the Radio buffer as PWM.
// Uses code from https://github.com/brainwagon/sstv-encoders/blob/master/martin.c
esp_err_t radio_transmitter_write_pulse(float freq, float ms, radio_waveform_data_t *waveform)
{
    // Convert ms to samples.
    uint32_t nsamp_bits = (uint32_t)roundf(radio_state->config.sample_rate * ms / 1000.) * radio_state->config.bits_per_sample;

    // Error check.
    if (waveform->current_offset_bit + nsamp_bits >= waveform->buffer_bits_len)
    {
        return ESP_ERR_NO_MEM;
    }

    float complex m = cexp(I * 2.0 * M_PI * freq / (float)radio_state->config.sample_rate);

    for (uint32_t i = 0; i < nsamp_bits; i += radio_state->config.bits_per_sample)
    {
        waveform->current_osc *= m;
        float r = crealf(waveform->current_osc);

        // Convert continuous sine wave into PWM signals.
        float min = R_SINE_MIN;
        float max = R_SINE_MIN + radio_state->config.pwm_range_addr;
        for (uint8_t j = 0; j < radio_state->config.num_pwm_levels; ++j)
        {
            if (IN_RANGE(min, max, r))
            {
                uint32_t byte_idx = (waveform->current_offset_bit + i) / BYTE_SIZE;
                uint8_t bit_idx = (waveform->current_offset_bit + i) % BYTE_SIZE;

                // Set either 0 bits, 1 bit... n bits depending on PWM range.
                // j tells us how many bits will be 1.
                for (uint8_t k = 0; k < j; ++k)
                {
                    waveform->buffer[byte_idx] |= 1UL << (bit_idx + k);
                }
                // Break because we found the right range.
                break;
            }

            // Iterate to next range.
            min += radio_state->config.pwm_range_addr;
            max += radio_state->config.pwm_range_addr;
        }
    }

    // Increment offset
    // Allows next write pulse to automatically write to next part of buffer
    // Similar to file pointer
    waveform->current_offset_bit += nsamp_bits;
    return ESP_OK;
}

// Timer interrupt for radio transmission.
// If interrupt is during transmission, play current transmission bit.
void IRAM_ATTR waveform_isr(void *para)
{
    int timer_idx = (int)para;

    // Retrieve the interrupt status and the counter value.
    // from the timer that reported the interrupt.
    uint32_t intr_status = TIMERG0.int_st_timers.val;
    TIMERG0.hw_timer[timer_idx].update = 1;

    // Clear the interrupt.
    if ((intr_status & BIT(timer_idx)) && timer_idx == radio_state->config.timer)
    {
        // If fully transmitted, just consume interrupt.
        // Otherwise, play correct bit.
        if (radio_state->waveform.current_transmit_bit < radio_state->waveform.buffer_bits_len)
        {
            // Convert transmission bit into gpio level.
            uint32_t byte_idx = radio_state->waveform.current_transmit_bit / BYTE_SIZE;
            uint8_t bit_idx = radio_state->waveform.current_transmit_bit % BYTE_SIZE;
            uint8_t level = (radio_state->waveform.buffer[byte_idx] & BIT(bit_idx)) > 0;

            // Don't error check.
            // Even if it fails, it's worth to just continue as each interrupt is so short.
            if (level)
                GPIO.out_w1ts = ((uint32_t)1 << radio_state->config.gpio);
            else
                GPIO.out_w1tc = ((uint32_t)1 << radio_state->config.gpio);

            // Keep track of transmit bit.
            ++radio_state->waveform.current_transmit_bit;
        }
        else
            radio_state->transmitting = 0;

        TIMERG0.int_clr_timers.t1 = 1;
    }

    // After the alarm has been triggered.
    // We need enable it again, so it is triggered the next time.
    TIMERG0.hw_timer[timer_idx].config.alarm_en = TIMER_ALARM_EN;
}

// Initialize timer with radio options.
esp_err_t radio_transmitter_timer_init(void)
{
    // Init error.
    esp_err_t err = ESP_OK;

    // Select and initialize basic parameters of the timer.
    timer_config_t config = {
        .divider = radio_state->config.timer_divider,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = 1,
    }; // default clock source is APB
    ESP_ERROR_VALIDATE("Radio Initialize Timer",
                       err,
                       timer_init(TIMER_GROUP_0, radio_state->config.timer, &config))

    // Timer's counter will initially start from value below.
    // Also, if auto_reload is set, this value will be automatically reload on alarm.
    ESP_ERROR_VALIDATE("Radio Timer Counter",
                       err,
                       timer_set_counter_value(TIMER_GROUP_0, radio_state->config.timer, 0))

    // Configure the alarm value and the interrupt on alarm.
    ESP_ERROR_VALIDATE("Radio Alarm Value",
                       err,
                       timer_set_alarm_value(TIMER_GROUP_0, radio_state->config.timer, 1))

    ESP_ERROR_VALIDATE("Radio Enable Interrupt",
                       err,
                       timer_enable_intr(TIMER_GROUP_0, radio_state->config.timer))

    ESP_ERROR_VALIDATE("Radio Register Interrupt",
                       err,
                       timer_isr_register(TIMER_GROUP_0, radio_state->config.timer, waveform_isr, (void *)radio_state->config.timer, ESP_INTR_FLAG_IRAM, NULL))

    // Start timer from initialization.
    ESP_ERROR_VALIDATE("Radio Start Timer",
                       err,
                       timer_start(TIMER_GROUP_0, radio_state->config.timer))

    return ESP_OK;
}

// Initialize GPIO and Timer for Radio.
esp_err_t radio_transmitter_init(radio_transmitter_config_t radio_config)
{
    // Init error.
    esp_err_t err = ESP_OK;

    radio_state = (radio_state_t *)calloc(sizeof(radio_state_t), 1);
    radio_state->config = radio_config;
    radio_state->waveform_queue = xQueueCreate(2, sizeof(radio_waveform_data_t));

    // Initialize semaphore.
    radio_state->radio_busy_mutex = xSemaphoreCreateMutex();
    xSemaphoreGive(radio_state->radio_busy_mutex);

    // GPIO configuration for transmission pin.
    gpio_config_t io_conf = {
        .pin_bit_mask = BIT(radio_state->config.gpio),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_PIN_INTR_DISABLE,
    };

    // Configure and set direction.
    ESP_ERROR_VALIDATE("Radio Configure GPIO",
                       err,
                       gpio_config(&io_conf))
    ESP_ERROR_VALIDATE("Radio Set GPIO Direction",
                       err,
                       gpio_set_direction(radio_state->config.gpio, GPIO_MODE_OUTPUT))

    // Initialize Radio timer.
    ESP_ERROR_VALIDATE("Radio Initialize Timer",
                       err,
                       radio_transmitter_timer_init())

    // Create data collection tasks.
    BaseType_t xReturned;
    xReturned = xTaskCreatePinnedToCore(
        radio_transmitter_task_entry, // Function that implements the task.
        "Radio Transmission System",  // Text name for the task.
        RADIO_TRANSMITTER_STACK_SIZE, // Stack size in words, not bytes.
        NULL,                         // Parameter passed into the task.
        tskIDLE_PRIORITY,             // Priority at which the task is created.
        NULL,
        0); // Used to pass out the created task's handle.

    if (xReturned != pdPASS)
        return ESP_ERR_NO_MEM;

    return ESP_OK;
}
