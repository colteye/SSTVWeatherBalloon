#include "sstv.h"

/*
 * Timer group0 ISR handler
 *
 * Note:
 * We don't call the timer API here because they are not declared with IRAM_ATTR.
 * If we're okay with the timer irq not being serviced while SPI flash cache is disabled,
 * we can allocate this interrupt without the ESP_INTR_FLAG_IRAM flag and use the normal API.
 */
void IRAM_ATTR sstv_pixel_isr(void *para)
{
    int timer_idx = (int)para;

    /* Retrieve the interrupt status and the counter value
       from the timer that reported the interrupt */
    uint32_t intr_status = TIMERG0.int_st_timers.val;
    TIMERG0.hw_timer[timer_idx].update = 1;
    uint64_t timer_counter_value =
        ((uint64_t)TIMERG0.hw_timer[timer_idx].cnt_high) << 32 | TIMERG0.hw_timer[timer_idx].cnt_low;

    /* Clear the interrupt */
    if ((intr_status & BIT(timer_idx)) && timer_idx == TIMER_1)
    {
        pixel_available = 1;
        TIMERG0.int_clr_timers.t1 = 1;
    }

    /* After the alarm has been triggered
      we need enable it again, so it is triggered the next time */
    TIMERG0.hw_timer[timer_idx].config.alarm_en = TIMER_ALARM_EN;
}

void init_sstv_timer()
{
    /* Select and initialize basic parameters of the timer */
    timer_config_t config = {
        .divider = SSTV_TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = 1,
    }; // default clock source is APB
    timer_init(TIMER_GROUP_0, SSTV_PIXEL_TIMER, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(TIMER_GROUP_0, SSTV_PIXEL_TIMER, 0);

    /* Configure the alarm value and the interrupt on alarm. */
    timer_set_alarm_value(TIMER_GROUP_0, SSTV_PIXEL_TIMER, 1);
    timer_enable_intr(TIMER_GROUP_0, SSTV_PIXEL_TIMER);
    timer_isr_register(TIMER_GROUP_0, SSTV_PIXEL_TIMER, sstv_pixel_isr,
                       (void *)SSTV_PIXEL_TIMER, ESP_INTR_FLAG_IRAM, NULL);
}
void init_tone(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = 5000, // Set output frequency at 5 kHz
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = LEDC_OUTPUT_IO,
        .duty = LEDC_DUTY,
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    ESP_ERROR_CHECK(ledc_fade_func_install(0));
}

// Change tones with PWM.
void play_tone(uint32_t freq_hz)
{
    ledc_set_duty_and_update(LEDC_MODE, LEDC_CHANNEL, 1, 0);

    //ESP_LOGI(TAG2, "FREQ: %d", freq_hz);
    ledc_set_freq(LEDC_MODE, LEDC_TIMER, freq_hz);
}

void sstv_pixel_to_tone(uint16_t pix, uint8_t color)
{
    uint32_t freq = 0;

    // Convert into index to turn into freq (Based on RGB565 for bitmasking)
    if (color == RED)
    {
        pix = (pix & 0b1111100000000000) >> 11;
        freq = RB_FREQ_LUT[pix - 1];
    }
    else if (color == GREEN)
    {
        pix = (pix & 0b0000011111100000) >> 5;
        freq = G_FREQ_LUT[pix - 1];
    }
    else // BLUE
    {
        pix = pix & 0b0000000000011111;
        freq = RB_FREQ_LUT[pix - 1];
    }
    play_tone(freq);
    //ets_delay_us(SSTV_MARTIN_2_SCAN_US);
}

// Calibration Header for VIS sequence
void sstv_start_transmission(void)
{
    play_tone(SSTV_1900HZ);
    vTaskDelay(300);
    play_tone(SSTV_1200HZ);
    vTaskDelay(10);
    play_tone(SSTV_1900HZ);
    vTaskDelay(300);
    play_tone(SSTV_1200HZ);
    vTaskDelay(30);

    for (int i = 0; i < NUM_VIS_BITS; ++i)
    {
        if (SSTV_MARTIN_2_VIS_BITS[i])
        {
            play_tone(SSTV_1100HZ);
        }
        else
        {
            play_tone(SSTV_1300HZ);
        }
        vTaskDelay(30);
    }

    play_tone(SSTV_1200HZ);
    vTaskDelay(30);
}

void sstv_start_line(void)
{
    play_tone(SSTV_1200HZ);
    vTaskDelay(SSTV_MARTIN_2_PULSE_MS);
    ets_delay_us(SSTV_MARTIN_2_PULSE_US);

    play_tone(SSTV_1500HZ);
    ets_delay_us(SSTV_MARTIN_2_PORCH_US);
}

void sstv_camera_line(camera_fb_t *pic, uint16_t line, uint8_t color)
{
    uint16_t i = 0;
    while (i < (pic->width << 1))
    {
        if (pixel_available)
        {
            // Convert 2 uint8_ts into a uint16_t
            uint16_t pix = ((uint16_t)pic->buf[i * line] << 8) | pic->buf[(i + 1) * line];
            sstv_pixel_to_tone(pix, color);
            i += 2;
            pixel_available = 0;
        }
    }
}

void sstv_banner_line(uint16_t pic_width, uint16_t line, uint8_t color)
{
    /*// Iterate over length of width * 2 to account for 2 uint8 -> uint16_t
    for (uint16_t i = 0; i < pic_width; ++i)
    {
        sstv_pixel_to_tone(SSTV_CALLSIGN[i * line], color);
    }
    play_tone(SSTV_1500HZ);
    ets_delay_us(SSTV_MARTIN_2_SEPARATOR_US);*/
    uint16_t i = 0;
    while (i < pic_width)
    {
        if (pixel_available)
        {
            sstv_pixel_to_tone(SSTV_CALLSIGN[i * line], color);
            ++i;
            pixel_available = 0;
        }
    }
}

// Martin 2 SSTV protocol for transmitting data
void sstv_transmit(camera_fb_t *pic)
{
    pixel_available = 0;
    sstv_start_transmission();
    init_sstv_timer();
    timer_start(TIMER_GROUP_0, SSTV_PIXEL_TIMER);

    play_tone(SSTV_1200HZ);
    ets_delay_us(9000);

    //vTaskDelay(SSTV_SCOTTIE_1_PULSE_MS);

    // Transmit image
    for (uint16_t i = 0; i < pic->height; ++i)
    {
        play_tone(SSTV_1500HZ);
        //vTaskDelay(SSTV_SCOTTIE_1_SEPARATOR_MS);
        //ets_delay_us(SSTV_SCOTTIE_1_SEPARATOR_US);
        ets_delay_us(1500);

        sstv_camera_line(pic, i, GREEN);

        play_tone(SSTV_1500HZ);
        //vTaskDelay(SSTV_SCOTTIE_1_SEPARATOR_MS);
        //ets_delay_us(SSTV_SCOTTIE_1_SEPARATOR_US);
        ets_delay_us(1500);

        sstv_camera_line(pic, i, BLUE);

        play_tone(SSTV_1200HZ);
        //vTaskDelay(SSTV_SCOTTIE_1_PULSE_MS);
        ets_delay_us(9000);

        play_tone(SSTV_1500HZ);
        //vTaskDelay(SSTV_SCOTTIE_1_PORCH_MS);
        //ets_delay_us(SSTV_SCOTTIE_1_PORCH_US);
        ets_delay_us(1500);

        sstv_camera_line(pic, i, RED);
    }

    // Transmit banner
    for (uint16_t i = 0; i < BANNER_HEIGHT; ++i)
    {
        /*sstv_banner_line(pic->width, i, GREEN);
        sstv_banner_line(pic->width, i, BLUE);
        sstv_banner_line(pic->width, i, RED);*/

        play_tone(SSTV_1500HZ);
        //vTaskDelay(SSTV_SCOTTIE_1_SEPARATOR_MS);
        //ets_delay_us(SSTV_SCOTTIE_1_SEPARATOR_US);
        ets_delay_us(1500);

        sstv_banner_line(pic->width, i, GREEN);

        play_tone(SSTV_1500HZ);
        //vTaskDelay(SSTV_SCOTTIE_1_SEPARATOR_MS);
        //ets_delay_us(SSTV_SCOTTIE_1_SEPARATOR_US);
        ets_delay_us(1500);

        sstv_banner_line(pic->width, i, BLUE);

        play_tone(SSTV_1200HZ);
        //vTaskDelay(SSTV_SCOTTIE_1_PULSE_MS);
        ets_delay_us(9000);

        play_tone(SSTV_1500HZ);
        //vTaskDelay(SSTV_SCOTTIE_1_PORCH_MS);
        //ets_delay_us(SSTV_SCOTTIE_1_PORCH_US);
        ets_delay_us(1500);

        sstv_banner_line(pic->width, i, RED);
    }

    ledc_set_duty_and_update(LEDC_MODE, LEDC_CHANNEL, 0, 0);
}