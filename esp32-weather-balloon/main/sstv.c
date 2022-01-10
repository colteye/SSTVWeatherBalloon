#include "sstv.h"

void init_tone(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQUENCY, // Set output frequency at 5 kHz
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
}

// Change tones with PWM.
void play_tone(uint32_t freq_hz)
{
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
    if (freq > 0)
    {
        play_tone(freq);
        ets_delay_us(SSTV_MARTIN_2_SCAN_US);
    }
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
    // Iterate over length of width * 2 to account for 2 uint8 -> uint16_t
    for (uint16_t i = 0; i < (pic->width << 1); i += 2)
    {
        // Convert 2 uint8_ts into a uint16_t
        uint16_t pix = ((uint16_t)pic->buf[i * line] << 8) | pic->buf[(i + 1) * line];
        sstv_pixel_to_tone(pix, color);
    }
    play_tone(SSTV_1500HZ);
    ets_delay_us(SSTV_MARTIN_2_SEPARATOR_US);
}

void sstv_banner_line(uint16_t pic_width, uint16_t line, uint8_t color)
{
    // Iterate over length of width * 2 to account for 2 uint8 -> uint16_t
    for (uint16_t i = 0; i < pic_width; ++i)
    {
        sstv_pixel_to_tone(SSTV_CALLSIGN[i * line], color);
    }
    play_tone(SSTV_1500HZ);
    ets_delay_us(SSTV_MARTIN_2_SEPARATOR_US);
}

// Martin 2 SSTV protocol for transmitting data
void sstv_transmit(camera_fb_t *pic)
{
    sstv_start_transmission();

    // Transmit image
    for (uint16_t i = 0; i < pic->height; ++i)
    {
        sstv_start_line();
        sstv_camera_line(pic, i, GREEN);
        sstv_camera_line(pic, i, BLUE);
        sstv_camera_line(pic, i, RED);
    }

    // Transmit banner
    for (uint16_t i = 0; i < BANNER_HEIGHT; ++i)
    {
        sstv_start_line();
        sstv_banner_line(pic->width, i, GREEN);
        sstv_banner_line(pic->width, i, BLUE);
        sstv_banner_line(pic->width, i, RED);
    }
}