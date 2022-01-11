#include "sstv.h"

float complex audio_osc = 0.25;
uint8_t read_audio_sample;

// MODIFIED FROM:
// An implementation of a Martin M1 encoder
// Written by Mark VandeWettering (K6HX)
// https://github.com/brainwagon/sstv-encoders/blob/master/martin.c

// Write pulse for specified frequency and amount of time
// Converted to an integer later to output square wave.
void write_pulse(float freq, float ms, uint32_t *offset)
{
    /* convert ms to samples */
    uint32_t nsamp = (uint32_t)roundf(SAMPLE_RATE * ms / 1000.);
    uint32_t i;
    float r;
    float complex m = cexp(I * 2.0 * M_PI * freq / (float)SAMPLE_RATE);

    for (i = 0; i < nsamp; i++)
    {
        audio_osc *= m;
        r = crealf(audio_osc);

        if (r > 0.0)
        {
            uint32_t byte_idx = (*offset + i) / BYTE_SIZE;
            uint8_t bit_idx = (*offset + i) % BYTE_SIZE;
            AUDIO_BUFFER[byte_idx] |= 1UL << bit_idx;
        }
    }
    *offset += nsamp;
}

float sstv_pixel_to_period(uint16_t pix, uint8_t color)
{
    float period = 0.;

    // Convert into index to turn into freq (Based on RGB565 for bitmasking)
    if (color == RED)
    {
        pix = (pix & 0b1111100000000000) >> 11;
        period = RB_PERIOD_LUT[pix];
    }
    else if (color == GREEN)
    {
        pix = (pix & 0b0000011111100000) >> 5;
        period = G_PERIOD_LUT[pix];
    }
    else // BLUE
    {
        pix = pix & 0b0000000000011111;
        period = RB_PERIOD_LUT[pix];
    }
    return period;
}

void sstv_start_line(uint32_t *offset)
{
    write_pulse(SSTV_1200HZ_PERIOD, SSTV_MARTIN_2_PULSE_PERIOD, offset);
    write_pulse(SSTV_1500HZ_PERIOD, SSTV_MARTIN_2_PORCH_PERIOD, offset);
}

void sstv_camera_line(camera_fb_t *pic, uint16_t line, uint8_t color, uint32_t *offset)
{
    // Iterate over length of width * 2 to account for 2 uint8 -> uint16_t
    for (uint16_t i = 0; i < (pic->width << 1); i += 2)
    {
        uint16_t pix = ((uint16_t)pic->buf[i * line] << 8) | pic->buf[(i + 1) * line];
        float freq_period = sstv_pixel_to_period(pix, color);
        write_pulse(freq_period, SSTV_MARTIN_2_SCAN_PERIOD, offset);
        //write_pulse(SSTV_1500HZ_PERIOD, SSTV_MARTIN_2_SCAN_PERIOD, offset);
    }
    write_pulse(SSTV_1500HZ_PERIOD, SSTV_MARTIN_2_SEPARATOR_PERIOD, offset);
}

void sstv_banner_line(uint16_t pic_width, uint16_t line, uint8_t color, uint32_t *offset)
{
    // Iterate over length of width * 2 to account for 2 uint8 -> uint16_t
    for (uint16_t i = 0; i < pic_width; ++i)
    {
        float freq_period = sstv_pixel_to_period(SSTV_CALLSIGN[i * line], color);
        write_pulse(freq_period, SSTV_MARTIN_2_SCAN_PERIOD, offset);
    }
    write_pulse(SSTV_1500HZ_PERIOD, SSTV_MARTIN_2_SEPARATOR_PERIOD, offset);
}

void sstv_generate_audio(camera_fb_t *pic)
{
    // First part of message.
    // Offset automatically incremented to correct location.
    uint32_t offset = 0;
    write_pulse(SSTV_1900HZ_PERIOD, SSTV_300MS_PERIOD, &offset);
    write_pulse(SSTV_1200HZ_PERIOD, SSTV_10MS_PERIOD, &offset);
    write_pulse(SSTV_1900HZ_PERIOD, SSTV_300MS_PERIOD, &offset);
    write_pulse(SSTV_1200HZ_PERIOD, SSTV_30MS_PERIOD, &offset);

    for (int i = 0; i < BYTE_SIZE; ++i)
    {
        if (SSTV_MARTIN_2_VIS_BITS[i])
        {
            write_pulse(SSTV_1100HZ_PERIOD, SSTV_30MS_PERIOD, &offset);
        }
        else
        {
            write_pulse(SSTV_1300HZ_PERIOD, SSTV_30MS_PERIOD, &offset);
        }
    }
    write_pulse(SSTV_1200HZ_PERIOD, SSTV_30MS_PERIOD, &offset);

    // Main Message.

    // Transmit banner
    for (uint16_t i = 0; i < BANNER_HEIGHT; ++i)
    {
        sstv_start_line(&offset);
        sstv_banner_line(pic->width, i, GREEN, &offset);
        sstv_banner_line(pic->width, i, BLUE, &offset);
        sstv_banner_line(pic->width, i, RED, &offset);
    }

    // Transmit image
    for (uint16_t i = 0; i < pic->height; ++i)
    {
        sstv_start_line(&offset);
        sstv_camera_line(pic, i, GREEN, &offset);
        sstv_camera_line(pic, i, BLUE, &offset);
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
        read_audio_sample = 1;
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
    timer_start(TIMER_GROUP_0, SSTV_AUDIO_TIMER);
}

void sstv_close()
{
    free(AUDIO_BUFFER);
}

void sstv_play_audio()
{
    uint32_t current_audio_sample = 0;
    while (1)
    {
        if (read_audio_sample)
        {
            uint32_t byte_idx = current_audio_sample / BYTE_SIZE;
            uint8_t bit_idx = current_audio_sample % BYTE_SIZE;
            uint8_t level = (AUDIO_BUFFER[byte_idx] & BIT(bit_idx)) > 0;

            gpio_set_level(AUDIO_GPIO, level);
            ++current_audio_sample;
            read_audio_sample = 0;

            // Play current sample.
            if (current_audio_sample >= (SAMPLE_RATE * CLIP_LENGTH))
            {
                // Once played, audio no longer available!
                return;
            }
        }
    }
}