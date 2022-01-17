#include "sstv.h"

// MODIFIED FROM:
// An implementation of a Martin M1 encoder
// Written by Mark VandeWettering (K6HX)
// https://github.com/brainwagon/sstv-encoders/blob/master/martin.c

// Write pulse for specified frequency and amount of time
// Convert output from raw sine wave to PWM.

float complex audio_osc = 0.25;
uint8_t read_audio_bit;

void write_pulse(float freq, float ms, uint32_t *offset)
{
    /* convert ms to samples */
    uint32_t nsamp_bits = (uint32_t)roundf(SAMPLE_RATE * ms / 1000.) * BITS_PER_SAMPLE;
    float complex m = cexp(I * 2.0 * M_PI * freq / (float)SAMPLE_RATE);

    for (uint32_t i = 0; i < nsamp_bits; i += BITS_PER_SAMPLE)
    {
        audio_osc *= m;
        float r = crealf(audio_osc);

        // Convert continuous sine wave into PWM signals to play on ESP32CAM.
        // No access to ADC :-(
        // Save to buffer for playback later.
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
                    AUDIO_BUFFER[byte_idx] |= 1UL << (bit_idx + k);
                }

                // Break because we found the right range.
                break;
            }

            min += SSTV_PWM_RANGE_ADDR;
            max += SSTV_PWM_RANGE_ADDR;
        }
    }
    *offset += nsamp_bits;
}

float sstv_pixel_to_freq(uint16_t pix, uint8_t color)
{
    float freq = 0.0;

    // Convert into index to turn into freq (Based on RGB565 for bitmasking)
    if (color == RED)
    {
        pix = (pix & 0b1111100000000000) >> 11;
        freq = RB_FREQ_LUT[pix];
    }
    else if (color == GREEN)
    {
        pix = (pix & 0b0000011111100000) >> 5;
        freq = G_FREQ_LUT[pix];
    }
    else // BLUE
    {
        pix = pix & 0b0000000000011111;
        freq = RB_FREQ_LUT[pix];
    }
    return freq;
}

void sstv_camera_line(camera_fb_t *pic, uint16_t line, uint8_t color, uint32_t *offset)
{
    // Start at current line
    uint32_t l_idx = pic->width * 2 * line;

    // Iterate over length of width * 2 to account for 2 uint8 -> uint16_t
    for (uint16_t i = 0; i < pic->width * 2; i += 2)
    {
        uint16_t pix = ((uint16_t)pic->buf[l_idx + i] << 8) | pic->buf[l_idx + (i + 1)];
        float freq = sstv_pixel_to_freq(pix, color);
        write_pulse(freq, SSTV_SCOTTIE_1_SCAN_MS, offset);
    }
}

void sstv_callsign_line(uint16_t pic_width, uint16_t line, uint8_t color, uint32_t *offset)
{
    // Start at current line
    uint32_t l_idx = pic_width * line;

    // Iterate over length of width
    for (uint16_t i = 0; i < pic_width; ++i)
    {
        float freq = sstv_pixel_to_freq(SSTV_CALLSIGN[l_idx + i], color);
        write_pulse(freq, SSTV_SCOTTIE_1_SCAN_MS, offset);
    }
}

void sstv_generate_audio(camera_fb_t *pic)
{
    // First part of message.
    // Offset automatically incremented to correct location.
    uint32_t offset = 0;

    /** VOX TONE (OPTIONAL) **/
    write_pulse(SSTV_1900HZ, SSTV_100MS, &offset);
    write_pulse(SSTV_1500HZ, SSTV_100MS, &offset);
    write_pulse(SSTV_1900HZ, SSTV_100MS, &offset);
    write_pulse(SSTV_1500HZ, SSTV_100MS, &offset);
    write_pulse(SSTV_2300HZ, SSTV_100MS, &offset);
    write_pulse(SSTV_1500HZ, SSTV_100MS, &offset);
    write_pulse(SSTV_2300HZ, SSTV_100MS, &offset);
    write_pulse(SSTV_1500HZ, SSTV_100MS, &offset);

    /** Playing VIS CODE **/
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
            ++parity; /* parity */
        }
        else
        {
            write_pulse(SSTV_1300HZ, SSTV_30MS, &offset);
        }
        code = code >> 1;
    }

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

    // Main Message.
    // Transmit callsign

    // First line pulse.
    write_pulse(SSTV_1200HZ, SSTV_SCOTTIE_1_PULSE_MS, &offset);

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

    // Transmit image
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

/*
 * Timer group0 ISR handler
 *
 * Note:
 * We don't call the timer API here because they are not declared with IRAM_ATTR.
 * If we're okay with the timer irq not being serviced while SPI flash cache is disabled,
 * we can allocate this interrupt without the ESP_INTR_FLAG_IRAM flag and use the normal API.
 */
void IRAM_ATTR sstv_play_audio_isr(void *para)
{
    int timer_idx = (int)para;

    /* Retrieve the interrupt status and the counter value
       from the timer that reported the interrupt */
    uint32_t intr_status = TIMERG0.int_st_timers.val;
    TIMERG0.hw_timer[timer_idx].update = 1;

    /* Clear the interrupt */
    if ((intr_status & BIT(timer_idx)) && timer_idx == TIMER_1)
    {
        read_audio_bit = 1;
        TIMERG0.int_clr_timers.t1 = 1;
    }

    /* After the alarm has been triggered
      we need enable it again, so it is triggered the next time */
    TIMERG0.hw_timer[timer_idx].config.alarm_en = TIMER_ALARM_EN;
}

void sstv_init_timer()
{
    /* Select and initialize basic parameters of the timer */
    timer_config_t config = {
        .divider = SSTV_TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = 1,
    }; // default clock source is APB
    timer_init(TIMER_GROUP_0, SSTV_AUDIO_TIMER, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(TIMER_GROUP_0, SSTV_AUDIO_TIMER, 0);

    /* Configure the alarm value and the interrupt on alarm. */
    timer_set_alarm_value(TIMER_GROUP_0, SSTV_AUDIO_TIMER, 1);
    timer_enable_intr(TIMER_GROUP_0, SSTV_AUDIO_TIMER);
    timer_isr_register(TIMER_GROUP_0, SSTV_AUDIO_TIMER, sstv_play_audio_isr,
                       (void *)SSTV_AUDIO_TIMER, ESP_INTR_FLAG_IRAM, NULL);
}

void sstv_init()
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = BIT(AUDIO_GPIO);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    gpio_set_direction(AUDIO_GPIO, GPIO_MODE_OUTPUT);

    AUDIO_BUFFER = (uint8_t *)calloc(AUDIO_BUFFER_LEN, sizeof(uint8_t));

    sstv_init_timer();
}

void sstv_close()
{
    free(AUDIO_BUFFER);
}

void sstv_play_audio()
{

    timer_start(TIMER_GROUP_0, SSTV_AUDIO_TIMER);
    uint32_t current_audio_bit = 0;
    while (1)
    {
        // Retrieve interrupt flag change.
        if (read_audio_bit)
        {
            uint32_t byte_idx = current_audio_bit / BYTE_SIZE;
            uint8_t bit_idx = current_audio_bit % BYTE_SIZE;
            uint8_t level = (AUDIO_BUFFER[byte_idx] & BIT(bit_idx)) > 0;

            gpio_set_level(AUDIO_GPIO, level);
            ++current_audio_bit;
            read_audio_bit = 0;

            // Once played, audio no longer available!
            if (current_audio_bit >= (SAMPLE_RATE * CLIP_LENGTH * BITS_PER_SAMPLE))
                return;
        }
    }
    timer_pause(TIMER_GROUP_0, SSTV_AUDIO_TIMER);
}