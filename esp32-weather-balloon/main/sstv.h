#ifndef _SSTV_H
#define _SSTV_H

#include <complex.h>
#include <math.h>

#include "esp32/rom/ets_sys.h"
#include "driver/ledc.h"
#include "esp_camera.h"
#include "driver/timer.h"

#include <esp_log.h>
static const char *TAG3 = "example:take_picture";

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "sstv_callsign.h"

// NOTE: FREERTOS timing needs to be set to 1000hz!
// This allows us to use vTaskDelay and still keep our watchdog timer
// Dont need to divide vTaskDelay by anything, as 1 tick = 1 ms
#define IN_RANGE(low, high, x) (low <= x && x <= high)
// Pick color for SSTV scanlines
#define RED 0
#define GREEN 1
#define BLUE 2

#define BYTE_SIZE 8
#define CLIP_LENGTH 59 // in seconds
#define BITS_PER_SAMPLE 2
#define SAMPLE_RATE 80000
#define AUDIO_BUFFER_LEN ((SAMPLE_RATE * CLIP_LENGTH * BITS_PER_SAMPLE) / BYTE_SIZE)
#define AUDIO_GPIO 2

// Final image size should be 320 x 256.
#define CALLSIGN_HEIGHT 16

#define SSTV_NUM_PWM_LEVELS 3
#define SSTV_SINE_MIN -0.25
#define SSTV_SINE_MAX 0.25
#define SSTV_PWM_RANGE_ADDR (SSTV_SINE_MAX - SSTV_SINE_MIN) / (float)SSTV_NUM_PWM_LEVELS
static const float SSTV_PWM_RANGES[(SSTV_NUM_PWM_LEVELS + 1)] = {SSTV_SINE_MIN, SSTV_SINE_MIN + SSTV_PWM_RANGE_ADDR,
                                                                 SSTV_SINE_MIN + (2 * SSTV_PWM_RANGE_ADDR), SSTV_SINE_MAX};

#define SSTV_TIMER_DIVIDER (APB_CLK_FREQ / (SAMPLE_RATE * BITS_PER_SAMPLE)) //Subtract to try to compensate for extra operations being done?
#define SSTV_AUDIO_TIMER (1)                                                // done with auto reload

// Martin 2 VIS is 40 (0b101000) (LSB = 0b000101), 0 in front, and even parity (0) in back.
static const uint8_t SSTV_MARTIN_2_VIS_BITS[8] = {0, 0, 0, 0, 1, 0, 1, 0};

#define SSTV_MARTIN_2_PULSE_MS 4.862
#define SSTV_MARTIN_2_PORCH_MS 0.572
#define SSTV_MARTIN_2_SEPARATOR_MS 0.572
#define SSTV_MARTIN_2_SCAN_MS 0.2288

#define SSTV_300MS 300.
#define SSTV_10MS 10.
#define SSTV_30MS 30.

#define SSTV_1100HZ 1100.
#define SSTV_1200HZ 1200.
#define SSTV_1300HZ 1300.
#define SSTV_1500HZ 1500.
#define SSTV_1900HZ 1900.
static const float RB_FREQ_LUT[32] = {1500.0, 1526.0980392, 1551.1960784, 1576.2941176, 1601.3921568, 1626.490196, 1651.5882351999999, 1676.6862744, 1701.7843136, 1726.8823528, 1751.980392, 1777.0784312, 1802.1764704, 1827.2745095999999, 1852.3725488, 1877.470588, 1902.5686272, 1927.6666664, 1952.7647056, 1977.8627448, 2002.9607839999999, 2028.0588232, 2053.1568624, 2078.2549016, 2103.3529408, 2128.45098, 2153.5490191999997, 2178.6470584, 2203.7450976, 2228.8431368, 2253.941176, 2279.0392152};

static const float G_FREQ_LUT[64] = {1500.0, 1513.5490196, 1526.0980392, 1538.6470588, 1551.1960784, 1563.745098, 1576.2941176, 1588.8431372, 1601.3921568, 1613.9411764, 1626.490196, 1639.0392156, 1651.5882351999999, 1664.1372548, 1676.6862744, 1689.235294, 1701.7843136, 1714.3333332, 1726.8823528, 1739.4313723999999, 1751.980392, 1764.5294116, 1777.0784312, 1789.6274508, 1802.1764704, 1814.72549, 1827.2745095999999, 1839.8235292, 1852.3725488, 1864.9215684, 1877.470588, 1890.0196076, 1902.5686272, 1915.1176467999999, 1927.6666664, 1940.215686, 1952.7647056, 1965.3137252, 1977.8627448, 1990.4117644, 2002.9607839999999, 2015.5098036, 2028.0588232, 2040.6078428, 2053.1568624, 2065.705882, 2078.2549016, 2090.8039212, 2103.3529408, 2115.9019604, 2128.45098, 2140.9999996, 2153.5490191999997, 2166.0980388, 2178.6470584, 2191.196078, 2203.7450976, 2216.2941172, 2228.8431368, 2241.3921564, 2253.941176, 2266.4901956, 2279.0392152, 2291.5882348};
uint8_t *AUDIO_BUFFER;

void write_pulse(float freq, float ms, uint32_t *offset);

void sstv_init();
void sstv_close();

float sstv_pixel_to_freq(uint16_t pix, uint8_t color);
void sstv_start_line(uint32_t *offset);
void sstv_camera_line(camera_fb_t *pic, uint16_t line, uint8_t color, uint32_t *offset);
void sstv_callsign_line(uint16_t pic_width, uint16_t line, uint8_t color, uint32_t *offset);
void sstv_generate_audio(camera_fb_t *pic);

void sstv_init_timer();
void sstv_play_audio();

#endif /* _SSTV_H */
