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
// Define different functions and variables for SSTV transmission of images. //
// SSTV Protocol: Scottie 1                                                  //
// ------------------------------------------------------------------------- //

#include "sstv.h"
#include "sstv_luts.h"
#include "camera.h"

#include <complex.h>
#include <math.h>

#include "convert.h"
#include "error_handling.h"

#include "driver/timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

// Check range of a value between 2 values
#define IN_RANGE(low, high, x) (low <= x && x <= high)

// Pick color channel for SSTV scanlines
#define RED (0)
#define GREEN (1)
#define BLUE (2)

// Output buffer data
uint8_t *WAVEFORM_BUFFER;
#define BITS_LEN (SAMPLE_RATE * CLIP_LENGTH * BITS_PER_SAMPLE)
#define WAVEFORM_BUFFER_LEN (BITS_LEN / BYTE_SIZE)

// SSTV waveform related defines
#define BYTE_SIZE (8)
#define CLIP_LENGTH (110) // in seconds
#define BITS_PER_SAMPLE (2)
#define SAMPLE_RATE (40000)
#define WAVEFORM_GPIO (GPIO_NUM_2)

#define SSTV_NUM_PWM_LEVELS (BITS_PER_SAMPLE + 1)
#define SSTV_SINE_MIN (-0.25f)
#define SSTV_SINE_MAX (0.25f)
#define SSTV_PWM_RANGE_ADDR (SSTV_SINE_MAX - SSTV_SINE_MIN) / (float)SSTV_NUM_PWM_LEVELS

// Final image size should be 320 x 256.
#define CALLSIGN_HEIGHT (16)

#define SSTV_TIMER_DIVIDER (APB_CLK_FREQ / (SAMPLE_RATE * BITS_PER_SAMPLE)) //Subtract to try to compensate for extra operations being done?
#define SSTV_WAVEFORM_TIMER (1)                                             // done with auto reload

// Scottie 1 SSTV defines
#define SSTV_SCOTTIE_1_VIS_CODE (60)
#define SSTV_SCOTTIE_1_PULSE_MS (9.0f)
#define SSTV_SCOTTIE_1_PORCH_MS (1.5f)
#define SSTV_SCOTTIE_1_SEPARATOR_MS (1.5f)
#define SSTV_SCOTTIE_1_SCAN_MS (0.4320f)

// These are just to avoid confusion about the values
#define SSTV_300MS (300.0f)
#define SSTV_100MS (100.0f)
#define SSTV_10MS (10.0f)
#define SSTV_30MS (30.0f)

#define SSTV_1100HZ (1100.0f)
#define SSTV_1200HZ (1200.0f)
#define SSTV_1300HZ (1300.0f)
#define SSTV_1500HZ (1500.0f)
#define SSTV_1900HZ (1900.0f)
#define SSTV_2300HZ (2300.0f)

// Start value to help generate sine wave oscillations.
float complex waveform_osc = 0.25;

// Used to keep track of waveform bit while transmitting.
uint32_t current_waveform_bit = 0xFFFFFFFF;

// Write a frequency for a specific number of time into the SSTV buffer as PWM.
// Uses code from https://github.com/brainwagon/sstv-encoders/blob/master/martin.c
esp_err_t write_pulse(float freq, float ms, uint32_t *offset)
{
    // Convert ms to samples.
    uint32_t nsamp_bits = (uint32_t)roundf(SAMPLE_RATE * ms / 1000.) * BITS_PER_SAMPLE;

    // Error check.
    if (*offset + nsamp_bits >= BITS_LEN)
    {
        return ESP_ERR_NO_MEM;
    }

    float complex m = cexp(I * 2.0 * M_PI * freq / (float)SAMPLE_RATE);

    for (uint32_t i = 0; i < nsamp_bits; i += BITS_PER_SAMPLE)
    {
        waveform_osc *= m;
        float r = crealf(waveform_osc);

        // Convert continuous sine wave into PWM signals.
        float min = SSTV_SINE_MIN;
        float max = SSTV_SINE_MIN + SSTV_PWM_RANGE_ADDR;
        for (uint8_t j = 0; j < SSTV_NUM_PWM_LEVELS; ++j)
        {
            if (IN_RANGE(min, max, r))
            {
                uint32_t byte_idx = (*offset + i) / BYTE_SIZE;
                uint8_t bit_idx = (*offset + i) % BYTE_SIZE;

                // Set either 0 bits, 1 bit... n bits depending on PWM range.
                // j tells us how many bits will be 1.
                for (uint8_t k = 0; k < j; ++k)
                {
                    WAVEFORM_BUFFER[byte_idx] |= 1UL << (bit_idx + k);
                }

                // Break because we found the right range.
                break;
            }

            // Iterate to next range.
            min += SSTV_PWM_RANGE_ADDR;
            max += SSTV_PWM_RANGE_ADDR;
        }
    }

    // Increment offset
    // Allows next write pulse to automatically write to next part of buffer
    // Similar to file pointer
    *offset += nsamp_bits;
    return ESP_OK;
}

// Convert pixel from specific color channel into a frequency.
float sstv_pixel_to_freq(uint16_t pixel, uint8_t color_channel)
{
    // Init frequency float.
    float freq = 0.0;

    // Convert into index to turn into freq.
    // Based on RGB565 for bitmasking.
    if (color_channel == RED)
    {
        pixel = (pixel & 0b1111100000000000) >> 11;
        freq = RB_FREQ_LUT[pixel];
    }
    else if (color_channel == GREEN)
    {
        pixel = (pixel & 0b0000011111100000) >> 5;
        freq = G_FREQ_LUT[pixel];
    }
    else // BLUE
    {
        pixel = pixel & 0b0000000000011111;
        freq = RB_FREQ_LUT[pixel];
    }
    return freq;
}

// Generate SSTV waveform for a single line of the camera.
esp_err_t sstv_camera_line(camera_fb_t *pic, uint16_t line, uint8_t color_channel, uint32_t *offset)
{
    // Init error.
    esp_err_t err = ESP_OK;

    // Start at current line.
    uint32_t l_idx = pic->width * 2 * line;

    // Iterate over length of width * 2 to account for 2 uint8 -> uint16_t.
    for (uint16_t i = 0; i < pic->width * 2; i += 2)
    {
        // Go to correct pixel, convert to frequency and write to the buffer.

        uint16_t pixel = U8S_2_U16(pic->buf[l_idx + i], pic->buf[l_idx + (i + 1)]);
        float freq = sstv_pixel_to_freq(pixel, color_channel);
        ESP_ERROR_VALIDATE("SSTV Camera Line",
                           err,
                           write_pulse(freq, SSTV_SCOTTIE_1_SCAN_MS, offset))
    }
    return ESP_OK;
}

// Generate SSTV waveform for a single line of the callsign.
esp_err_t sstv_callsign_line(uint16_t pic_width, uint16_t line, uint8_t color_channel, uint32_t *offset)
{
    // Init error.
    esp_err_t err = ESP_OK;

    // Start at current line.
    uint32_t l_idx = pic_width * line;

    // Iterate over length of width.
    for (uint16_t i = 0; i < pic_width; ++i)
    {
        // Go to correct pixel, convert to frequency and write to the buffer.
        float freq = sstv_pixel_to_freq(SSTV_CALLSIGN[l_idx + i], color_channel);
        ESP_ERROR_VALIDATE("SSTV Callsign Line",
                           err,
                           write_pulse(freq, SSTV_SCOTTIE_1_SCAN_MS, offset))
    }
    return ESP_OK;
}

// Generate SSTV waveform of entire image.
esp_err_t sstv_generate_waveform(camera_fb_t *pic)
{
    // Init error.
    esp_err_t err = ESP_OK;

    // First part of message.
    // Offset automatically incremented to correct location.
    uint32_t offset = 0;

    // Initial optional vox tone.
    ESP_ERROR_VALIDATE("VOX Tone",
                       err,
                       write_pulse(SSTV_1900HZ, SSTV_100MS, &offset))
    ESP_ERROR_VALIDATE("VOX Tone",
                       err,
                       write_pulse(SSTV_1500HZ, SSTV_100MS, &offset))
    ESP_ERROR_VALIDATE("VOX Tone",
                       err,
                       write_pulse(SSTV_1900HZ, SSTV_100MS, &offset))
    ESP_ERROR_VALIDATE("VOX Tone",
                       err,
                       write_pulse(SSTV_1500HZ, SSTV_100MS, &offset))
    ESP_ERROR_VALIDATE("VOX Tone",
                       err,
                       write_pulse(SSTV_2300HZ, SSTV_100MS, &offset))
    ESP_ERROR_VALIDATE("VOX Tone",
                       err,
                       write_pulse(SSTV_1500HZ, SSTV_100MS, &offset))
    ESP_ERROR_VALIDATE("VOX Tone",
                       err,
                       write_pulse(SSTV_2300HZ, SSTV_100MS, &offset))
    ESP_ERROR_VALIDATE("VOX Tone",
                       err,
                       write_pulse(SSTV_1500HZ, SSTV_100MS, &offset))

    // Start tone
    ESP_ERROR_VALIDATE("Start Tone",
                       err,
                       write_pulse(SSTV_1900HZ, SSTV_300MS, &offset))
    ESP_ERROR_VALIDATE("Start Tone",
                       err,
                       write_pulse(SSTV_1200HZ, SSTV_10MS, &offset))
    ESP_ERROR_VALIDATE("Start Tone",
                       err,
                       write_pulse(SSTV_1900HZ, SSTV_300MS, &offset))
    ESP_ERROR_VALIDATE("Start Tone",
                       err,
                       write_pulse(SSTV_1200HZ, SSTV_30MS, &offset))

    // Playing VIS code.
    uint8_t code = SSTV_SCOTTIE_1_VIS_CODE;
    uint8_t parity = 0;
    for (uint8_t i = 0; i < 7; i++)
    {
        if (code & 1)
        {
            ESP_ERROR_VALIDATE("VIS Code",
                               err,
                               write_pulse(SSTV_1100HZ, SSTV_30MS, &offset))
            ++parity;
        }
        else
        {
            ESP_ERROR_VALIDATE("VIS Code",
                               err,
                               write_pulse(SSTV_1300HZ, SSTV_30MS, &offset))
        }
        code = code >> 1;
    }

    // Add parity!
    if (parity & 1)
    {
        ESP_ERROR_VALIDATE("VIS Parity",
                           err,
                           write_pulse(SSTV_1100HZ, SSTV_30MS, &offset)) /* Output a 1 */
    }
    else
    {
        ESP_ERROR_VALIDATE("VIS Parity",
                           err,
                           write_pulse(SSTV_1300HZ, SSTV_30MS, &offset)) /* Output a 0 */
    }

    // Stop bit.
    ESP_ERROR_VALIDATE("Stop Bit",
                       err,
                       write_pulse(SSTV_1200HZ, SSTV_30MS, &offset))

    // First line pulse.
    ESP_ERROR_VALIDATE("First Pulse",
                       err,
                       write_pulse(SSTV_1200HZ, SSTV_SCOTTIE_1_PULSE_MS, &offset))

    // Transmit callsign (320x16).
    for (uint16_t i = 0; i < CALLSIGN_HEIGHT; ++i)
    {
        ESP_ERROR_VALIDATE("Callsign Line Separator",
                           err,
                           write_pulse(SSTV_1500HZ, SSTV_SCOTTIE_1_SEPARATOR_MS, &offset))

        ESP_ERROR_VALIDATE("Callsign Line Green",
                           err,
                           sstv_callsign_line(pic->width, i, GREEN, &offset))

        ESP_ERROR_VALIDATE("Callsign Line Separator",
                           err,
                           write_pulse(SSTV_1500HZ, SSTV_SCOTTIE_1_SEPARATOR_MS, &offset))

        ESP_ERROR_VALIDATE("Callsign Line Blue",
                           err,
                           sstv_callsign_line(pic->width, i, BLUE, &offset))

        ESP_ERROR_VALIDATE("Callsign Pulse",
                           err,
                           write_pulse(SSTV_1200HZ, SSTV_SCOTTIE_1_PULSE_MS, &offset))
        ESP_ERROR_VALIDATE("Callsign Porch",
                           err,
                           write_pulse(SSTV_1500HZ, SSTV_SCOTTIE_1_PORCH_MS, &offset))

        ESP_ERROR_VALIDATE("Callsign Line Red",
                           err,
                           sstv_callsign_line(pic->width, i, RED, &offset))
    }

    // Transmit image (320x240).
    // Using QVGA from camera
    for (uint16_t i = 0; i < pic->height; ++i)
    {

        ESP_ERROR_VALIDATE("Camera Line Separator",
                           err,
                           write_pulse(SSTV_1500HZ, SSTV_SCOTTIE_1_SEPARATOR_MS, &offset))

        ESP_ERROR_VALIDATE("Camera Line Green",
                           err,
                           sstv_camera_line(pic, i, GREEN, &offset))

        ESP_ERROR_VALIDATE("Camera Line Separator",
                           err,
                           write_pulse(SSTV_1500HZ, SSTV_SCOTTIE_1_SEPARATOR_MS, &offset))

        ESP_ERROR_VALIDATE("Camera Line Blue",
                           err,
                           sstv_camera_line(pic, i, BLUE, &offset))

        ESP_ERROR_VALIDATE("Camera Pulse",
                           err,
                           write_pulse(SSTV_1200HZ, SSTV_SCOTTIE_1_PULSE_MS, &offset))
        ESP_ERROR_VALIDATE("Camera Porch",
                           err,
                           write_pulse(SSTV_1500HZ, SSTV_SCOTTIE_1_PORCH_MS, &offset))

        ESP_ERROR_VALIDATE("Camera Line Red",
                           err,
                           sstv_camera_line(pic, i, RED, &offset))
    }
    return ESP_OK;
}

// Timer interrupt for SSTV transmission.
// If interrupt is during transmission, play current transmission bit.
void IRAM_ATTR sstv_transmit_waveform_isr(void *para)
{
    int timer_idx = (int)para;

    // Retrieve the interrupt status and the counter value.
    // from the timer that reported the interrupt.
    uint32_t intr_status = TIMERG0.int_st_timers.val;
    TIMERG0.hw_timer[timer_idx].update = 1;

    // Clear the interrupt.
    if ((intr_status & BIT(timer_idx)) && timer_idx == TIMER_1)
    {
        // If fully transmitted, just consume interrupt.
        // Otherwise, play correct bit.
        if (current_waveform_bit < (SAMPLE_RATE * CLIP_LENGTH * BITS_PER_SAMPLE))
        {
            // Convert transmission bit into gpio level.
            uint32_t byte_idx = current_waveform_bit / BYTE_SIZE;
            uint8_t bit_idx = current_waveform_bit % BYTE_SIZE;
            uint8_t level = (WAVEFORM_BUFFER[byte_idx] & BIT(bit_idx)) > 0;

            // Don't error check.
            //Even if it fails, it's worth to just continue as each interrupt is so short.
            gpio_set_level(WAVEFORM_GPIO, level);

            ++current_waveform_bit;
        }

        TIMERG0.int_clr_timers.t1 = 1;
    }

    // After the alarm has been triggered.
    // We need enable it again, so it is triggered the next time.
    TIMERG0.hw_timer[timer_idx].config.alarm_en = TIMER_ALARM_EN;
}

// Initialize timer with SSTV options.
esp_err_t sstv_timer_init(void)
{
    // Init error.
    esp_err_t err = ESP_OK;

    // Select and initialize basic parameters of the timer.
    timer_config_t config = {
        .divider = SSTV_TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = 1,
    }; // default clock source is APB
    ESP_ERROR_VALIDATE("SSTV Initialize Timer",
                       err,
                       timer_init(TIMER_GROUP_0, SSTV_WAVEFORM_TIMER, &config))

    // Timer's counter will initially start from value below.
    // Also, if auto_reload is set, this value will be automatically reload on alarm.
    ESP_ERROR_VALIDATE("SSTV Timer Counter",
                       err,
                       timer_set_counter_value(TIMER_GROUP_0, SSTV_WAVEFORM_TIMER, 0))

    // Configure the alarm value and the interrupt on alarm.
    ESP_ERROR_VALIDATE("SSTV Alarm Value",
                       err,
                       timer_set_alarm_value(TIMER_GROUP_0, SSTV_WAVEFORM_TIMER, 1))

    ESP_ERROR_VALIDATE("SSTV Enable Interrupt",
                       err,
                       timer_enable_intr(TIMER_GROUP_0, SSTV_WAVEFORM_TIMER))

    ESP_ERROR_VALIDATE("SSTV Register Interrupt",
                       err,
                       timer_isr_register(TIMER_GROUP_0, SSTV_WAVEFORM_TIMER, sstv_transmit_waveform_isr, (void *)SSTV_WAVEFORM_TIMER, ESP_INTR_FLAG_IRAM, NULL))

    // Start timer from initialization.
    ESP_ERROR_VALIDATE("SSTV Start Timer",
                       err,
                       timer_start(TIMER_GROUP_0, SSTV_WAVEFORM_TIMER))

    return ESP_OK;
}

// Initialize GPIO and Timer for SSTV.
esp_err_t sstv_init(void)
{
    // Init error.
    esp_err_t err = ESP_OK;

    // GPIO configuration for transmission pin.
    gpio_config_t io_conf = {
        .pin_bit_mask = BIT(WAVEFORM_GPIO),
        .mode = GPIO_MODE_WAVEFORM,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_PIN_INTR_DISABLE,
    };

    // Configure and set direction.
    ESP_ERROR_VALIDATE("SSTV Configure GPIO",
                       err,
                       gpio_config(&io_conf))
    ESP_ERROR_VALIDATE("SSTV Set GPIO Direction",
                       err,
                       gpio_set_direction(WAVEFORM_GPIO, GPIO_MODE_WAVEFORM))

    // Initialize SSTVE timer.
    ESP_ERROR_VALIDATE("SSTV Initialize Timer",
                       err,
                       sstv_timer_init())

    return ESP_OK;
}

// Initialize SSTV waveform buffer.
esp_err_t sstv_buffer_init(void)
{
    // Use calloc to start everything at 0.
    WAVEFORM_BUFFER = (uint8_t *)calloc(WAVEFORM_BUFFER_LEN, sizeof(uint8_t));
    if (!WAVEFORM_BUFFER)
    {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

// Clear SSTV waveform buffer.
void sstv_buffer_clear(void)
{
    // Free buffer pointer.
    free(WAVEFORM_BUFFER);
}

void sstv_task_entry(void *arg)
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
}