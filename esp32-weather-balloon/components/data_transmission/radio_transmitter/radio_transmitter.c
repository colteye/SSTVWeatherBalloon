#include "radio_transmitter.h"
#include "camera.h"

#include "convert.h"
#include "error_handling.h"

#include "driver/timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

typedef struct
{
    radio_config_t config;
    radio_waveform_data_t waveform;
    QueueHandle_t waveform_queue; // Radio waveform queue handle
    SemaphoreHandle_t radio_busy_mutex;

} radio_state_t;

static radio_state_t *radio_state = NULL;

// Check range of a value between 2 values
#define IN_RANGE(low, high, x) (low <= x && x <= high)

// Write a frequency for a specific number of time into the Radio buffer as PWM.
// Uses code from https://github.com/brainwagon/radio-encoders/blob/master/martin.c
esp_err_t write_pulse(float freq, float ms, radio_waveform_data_t *waveform)
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
        waveform_osc *= m;
        float r = crealf(waveform->current_osc);

        // Convert continuous sine wave into PWM signals.
        float min = SINE_MIN;
        float max = SINE_MIN + radio_state->config.pwm_range_addr;
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
        if (radio_state->waveform.current_transmit_bit < radio_state->buffer_bits_len)
        {
            // Convert transmission bit into gpio level.
            uint32_t byte_idx = radio_state->waveform.current_transmit_bit / BYTE_SIZE;
            uint8_t bit_idx = radio_state->waveform.current_transmit_bit % BYTE_SIZE;
            uint8_t level = (radio_state->waveform.buffer[byte_idx] & BIT(bit_idx)) > 0;

            // Don't error check.
            //Even if it fails, it's worth to just continue as each interrupt is so short.
            gpio_set_level(radio_state->config.gpio, level);

            ++radio_state->waveform.current_transmit_bit;
        }
        else
        {
            // Give semaphore back, radio no longer busy.
            xSemaphoreGiveFromISR(radio_state->radio_busy_mutex, NULL);
        }

        TIMERG0.int_clr_timers.t1 = 1;
    }

    // After the alarm has been triggered.
    // We need enable it again, so it is triggered the next time.
    TIMERG0.hw_timer[timer_idx].config.alarm_en = TIMER_ALARM_EN;
}

// Initialize timer with Radio options.
esp_err_t radio_timer_init(void)
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
                       timer_isr_register(TIMER_GROUP_0, radio_state->config.timer, waveform_isr, (void *)radio_state->timer, ESP_INTR_FLAG_IRAM, NULL))

    // Start timer from initialization.
    ESP_ERROR_VALIDATE("Radio Start Timer",
                       err,
                       timer_start(TIMER_GROUP_0, radio_state->config.timer))

    return ESP_OK;
}

// Initialize GPIO and Timer for Radio.
esp_err_t radio_init(radio_config_t radio_config)
{
    // Init error.
    esp_err_t err = ESP_OK;

    radio_state = (radio_state_t *)calloc(sizeof(radio_state_t), 1);
    radio_state->config = radio_config;
    radio_state->waveform_queue = xQueueCreate(4, sizeof(radio_waveform_data_t));
    radio_state->radio_busy_mutex = xSemaphoreCreateBinary();

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

    // Initialize RadioE timer.
    ESP_ERROR_VALIDATE("Radio Initialize Timer",
                       err,
                       radio_timer_init())

    return ESP_OK;
}

/*// Initialize Radio waveform buffer.
esp_err_t radio_waveform_buffer_init()
{
    // Use calloc to start everything at 0.
    radio_state->data.buffer = (uint8_t *)calloc(radio_state->data.buffer_len, sizeof(uint8_t));
    if (!radio_state->data.buffer)
    {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

// Clear Radio waveform buffer.
void radio_waveform_buffer_clear()
{
    // Free buffer pointer.
    free(radio_state->data.buffer);
}*/

void radio_transmit(radio_state_t *radio_state)
{
    radio_state->waveform.current_transmit_bit = 0;
}

void radio_task_entry(void *arg)
{
    radio_state_t *radio_state = (radio_state_t *)arg;
    radio_init(radio_state);

    radio_waveform_data_t current_waveform;
    for (;;)
    {
        if (xQueueReceive(radio_state->waveform_queue, &current_waveform, pdMS_TO_TICKS(200)))
        {
            // Transmission is now busy
            if (xSemaphoreTake(radio_state->mutex, pdMS_TO_TICKS(50)) == pdTRUE)
            {
                ESP_LOGI(RADIO_TAG, "Transmitting data!");

                radio_state->data = current_waveform;
                radio_transmit(radio_state);
            }
            xQueueReset(radio_state->data_queue);
            break;
        }
    }
    vTaskDelete(NULL);
}
