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

// Start value to help generate sine wave oscillations.
float complex output_osc = 0.25;

// Used to keep track of output bit while transmitting.
uint32_t current_output_bit = 0xFFFFFFFF;

void write_pulse(float freq, float ms, uint32_t *offset)
{
    // Convert ms to samples.
    uint32_t nsamp_bits = (uint32_t)roundf(SAMPLE_RATE * ms / 1000.) * BITS_PER_SAMPLE;
    float complex m = cexp(I * 2.0 * M_PI * freq / (float)SAMPLE_RATE);

    for (uint32_t i = 0; i < nsamp_bits; i += BITS_PER_SAMPLE)
    {
        output_osc *= m;
        float r = crealf(output_osc);

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
                    OUTPUT_BUFFER[byte_idx] |= 1UL << (bit_idx + k);
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
}

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

void sstv_camera_line(camera_fb_t *pic, uint16_t line, uint8_t color_channel, uint32_t *offset)
{
    // Start at current line.
    uint32_t l_idx = pic->width * 2 * line;

    // Iterate over length of width * 2 to account for 2 uint8 -> uint16_t.
    for (uint16_t i = 0; i < pic->width * 2; i += 2)
    {
        // Go to correct pixel, convert to frequency and write to the buffer.
        uint16_t pixel = ((uint16_t)pic->buf[l_idx + i] << 8) | pic->buf[l_idx + (i + 1)];
        float freq = sstv_pixel_to_freq(pixel, color_channel);
        write_pulse(freq, SSTV_SCOTTIE_1_SCAN_MS, offset);
    }
}

void sstv_callsign_line(uint16_t pic_width, uint16_t line, uint8_t color_channel, uint32_t *offset)
{
    // Start at current line.
    uint32_t l_idx = pic_width * line;

    // Iterate over length of width.
    for (uint16_t i = 0; i < pic_width; ++i)
    {
        // Go to correct pixel, convert to frequency and write to the buffer.
        float freq = sstv_pixel_to_freq(SSTV_CALLSIGN[l_idx + i], color_channel);
        write_pulse(freq, SSTV_SCOTTIE_1_SCAN_MS, offset);
    }
}

void sstv_generate_output(camera_fb_t *pic)
{
    // First part of message.
    // Offset automatically incremented to correct location.
    uint32_t offset = 0;

    // Initial Vox tone.
    write_pulse(SSTV_1900HZ, SSTV_100MS, &offset);
    write_pulse(SSTV_1500HZ, SSTV_100MS, &offset);
    write_pulse(SSTV_1900HZ, SSTV_100MS, &offset);
    write_pulse(SSTV_1500HZ, SSTV_100MS, &offset);
    write_pulse(SSTV_2300HZ, SSTV_100MS, &offset);
    write_pulse(SSTV_1500HZ, SSTV_100MS, &offset);
    write_pulse(SSTV_2300HZ, SSTV_100MS, &offset);
    write_pulse(SSTV_1500HZ, SSTV_100MS, &offset);

    // Playing VIS code.
    write_pulse(SSTV_1900HZ, SSTV_300MS, &offset);
    write_pulse(SSTV_1200HZ, SSTV_10MS, &offset);
    write_pulse(SSTV_1900HZ, SSTV_300MS, &offset);
    write_pulse(SSTV_1200HZ, SSTV_30MS, &offset);

    uint8_t code = SSTV_SCOTTIE_1_VIS_CODE;
    uint8_t parity = 0;
    for (uint8_t i = 0; i < 7; i++)
    {
        if (code & 1)
        {
            write_pulse(SSTV_1100HZ, SSTV_30MS, &offset);
            ++parity;
        }
        else
        {
            write_pulse(SSTV_1300HZ, SSTV_30MS, &offset);
        }
        code = code >> 1;
    }

    // Add parity!
    if (parity & 1)
    {
        write_pulse(SSTV_1100HZ, SSTV_30MS, &offset); /* Output a 1 */
    }
    else
    {
        write_pulse(SSTV_1300HZ, SSTV_30MS, &offset); /* Output a 0 */
    }

    // Stop bit.
    write_pulse(SSTV_1200HZ, SSTV_30MS, &offset);

    // First line pulse.
    write_pulse(SSTV_1200HZ, SSTV_SCOTTIE_1_PULSE_MS, &offset);

    // Transmit Callsign first.
    // Based on correct protocol.
    for (uint16_t i = 0; i < CALLSIGN_HEIGHT; ++i)
    {
        write_pulse(SSTV_1500HZ, SSTV_SCOTTIE_1_SEPARATOR_MS, &offset);

        sstv_callsign_line(pic->width, i, GREEN, &offset);

        write_pulse(SSTV_1500HZ, SSTV_SCOTTIE_1_SEPARATOR_MS, &offset);

        sstv_callsign_line(pic->width, i, BLUE, &offset);

        write_pulse(SSTV_1200HZ, SSTV_SCOTTIE_1_PULSE_MS, &offset);
        write_pulse(SSTV_1500HZ, SSTV_SCOTTIE_1_PORCH_MS, &offset);

        sstv_callsign_line(pic->width, i, RED, &offset);
    }

    // Transmit 320x240 image.
    // Based on correct protocol.
    for (uint16_t i = 0; i < pic->height; ++i)
    {
        write_pulse(SSTV_1500HZ, SSTV_SCOTTIE_1_SEPARATOR_MS, &offset);

        sstv_camera_line(pic, i, GREEN, &offset);

        write_pulse(SSTV_1500HZ, SSTV_SCOTTIE_1_SEPARATOR_MS, &offset);

        sstv_camera_line(pic, i, BLUE, &offset);

        write_pulse(SSTV_1200HZ, SSTV_SCOTTIE_1_PULSE_MS, &offset);
        write_pulse(SSTV_1500HZ, SSTV_SCOTTIE_1_PORCH_MS, &offset);

        sstv_camera_line(pic, i, RED, &offset);
    }
}

// Timer interrupt for SSTV transmission.
// If interrupt is during transmission, play current transmission bit.
void IRAM_ATTR sstv_transmit_output_isr(void *para)
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
        if (current_output_bit < (SAMPLE_RATE * CLIP_LENGTH * BITS_PER_SAMPLE))
        {
            // Convert transmission bit into gpio level.
            uint32_t byte_idx = current_output_bit / BYTE_SIZE;
            uint8_t bit_idx = current_output_bit % BYTE_SIZE;
            uint8_t level = (OUTPUT_BUFFER[byte_idx] & BIT(bit_idx)) > 0;

            gpio_set_level(OUTPUT_GPIO, level);
            ++current_output_bit;
        }

        TIMERG0.int_clr_timers.t1 = 1;
    }

    // After the alarm has been triggered.
    // We need enable it again, so it is triggered the next time.
    TIMERG0.hw_timer[timer_idx].config.alarm_en = TIMER_ALARM_EN;
}

void sstv_timer_init(void)
{
    // Select and initialize basic parameters of the timer.
    timer_config_t config = {
        .divider = SSTV_TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = 1,
    }; // default clock source is APB

    timer_init(TIMER_GROUP_0, SSTV_OUTPUT_TIMER, &config);

    // Timer's counter will initially start from value below.
    // Also, if auto_reload is set, this value will be automatically reload on alarm.
    timer_set_counter_value(TIMER_GROUP_0, SSTV_OUTPUT_TIMER, 0);

    // Configure the alarm value and the interrupt on alarm.
    timer_set_alarm_value(TIMER_GROUP_0, SSTV_OUTPUT_TIMER, 1);
    timer_enable_intr(TIMER_GROUP_0, SSTV_OUTPUT_TIMER);
    timer_isr_register(TIMER_GROUP_0, SSTV_OUTPUT_TIMER, sstv_transmit_output_isr,
                       (void *)SSTV_OUTPUT_TIMER, ESP_INTR_FLAG_IRAM, NULL);

    // Start timer from initialization.
    timer_start(TIMER_GROUP_0, SSTV_OUTPUT_TIMER);
}

void sstv_init(void)
{
    // GPIO configuration for transmission pin.
    gpio_config_t io_conf = {
        .pin_bit_mask = BIT(OUTPUT_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_PIN_INTR_DISABLE,
    };

    // Configure and set direction.
    gpio_config(&io_conf);
    gpio_set_direction(OUTPUT_GPIO, GPIO_MODE_OUTPUT);

    // Initialize SSTVE timer.
    sstv_timer_init();
}

void sstv_buffer_init(void)
{
    // Use calloc to start everything at 0.
    OUTPUT_BUFFER = (uint8_t *)calloc(OUTPUT_BUFFER_LEN, sizeof(uint8_t));
}

void sstv_buffer_clear(void)
{
    // Free buffer pointer.
    free(OUTPUT_BUFFER);
}

void vSSTVTakePicture(void *pvParameters)
{
    // Initialize SSTV transmission and camera recording.
    sstv_init();
    camera_init();

    // Infinite loop for taking pictures.
    static const char *TAG = "SSTV Camera";
    for (;;)
    {
        // Create buffer.
        sstv_buffer_init();

        ESP_LOGI(TAG, "Taking picture...");
        camera_fb_t *pic = esp_camera_fb_get();

        // use pic->buf to access the image.
        ESP_LOGI(TAG, "Picture taken! Its size was: %zu bytes", pic->len);
        esp_camera_fb_return(pic);

        ESP_LOGI(TAG, "Processing picture with SCOTTIE 1 SSTV...");

        // Generate wave data.
        sstv_generate_output(pic);

        ESP_LOGI(TAG, "Finished Processing!");
        ESP_LOGI(TAG, "Transmitting SSTV signal...");

        // Start the timer to output data.
        current_output_bit = 0;

        // Delay while timer writes transmission to GPIO.
        vTaskDelay(CLIP_LENGTH * 1000);

        ESP_LOGI(TAG, "Finished Transmitting!");

        // Clear buffer after completed transmission.
        sstv_buffer_clear();

        // Delay before next image is captured.
        vTaskDelay(5000);
    }
}