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
#define CLIP_LENGTH 111 // in seconds
#define BITS_PER_SAMPLE 2
#define SAMPLE_RATE 40000
#define AUDIO_BUFFER_LEN ((SAMPLE_RATE * CLIP_LENGTH * BITS_PER_SAMPLE) / BYTE_SIZE)
#define AUDIO_GPIO 2

// Final image size should be 320 x 256.
#define CALLSIGN_HEIGHT 16

#define SSTV_NUM_PWM_LEVELS (BITS_PER_SAMPLE + 1)
#define SSTV_SINE_MIN -0.25f
#define SSTV_SINE_MAX 0.25f
#define SSTV_PWM_RANGE_ADDR (SSTV_SINE_MAX - SSTV_SINE_MIN) / (float)SSTV_NUM_PWM_LEVELS

#define SSTV_TIMER_DIVIDER (APB_CLK_FREQ / (SAMPLE_RATE * BITS_PER_SAMPLE)) //Subtract to try to compensate for extra operations being done?
#define SSTV_AUDIO_TIMER (1)                                                // done with auto reload

#define SSTV_SCOTTIE_1_VIS_CODE 60
#define SSTV_SCOTTIE_1_PULSE_MS (9.0f)
#define SSTV_SCOTTIE_1_PORCH_MS (1.5f)
#define SSTV_SCOTTIE_1_SEPARATOR_MS (1.5f)
#define SSTV_SCOTTIE_1_SCAN_MS (0.4320f)

#define SSTV_300MS 300.0f
#define SSTV_100MS 100.0f
#define SSTV_10MS 10.0f
#define SSTV_30MS 30.0f

#define SSTV_1100HZ (1100.0f)
#define SSTV_1200HZ 1200.0f
#define SSTV_1300HZ 1300.0f
#define SSTV_1500HZ 1500.0f
#define SSTV_1900HZ 1900.0f
#define SSTV_2300HZ 2300.0f
static const float RB_FREQ_LUT[32] = {1500.0f, 1525.0980392f, 1550.1960784f, 1575.2941176f, 1600.3921568f, 1625.490196f, 1650.5882351999999f, 1675.6862744f, 1700.7843136f, 1725.8823528f, 1750.980392f, 1776.0784312f, 1801.1764704f, 1826.2745095999999f, 1851.3725488f, 1876.470588f, 1901.5686272f, 1926.6666664f, 1951.7647056f, 1976.8627448f, 2001.9607839999999f, 2027.0588232f, 2052.1568624f, 2077.2549016f, 2102.3529408f, 2127.45098f, 2152.5490191999997f, 2177.6470584f, 2202.7450976f, 2227.8431368f, 2252.941176f, 2278.0392152f};

static const float G_FREQ_LUT[64] = {1500.0f, 1512.5490196f, 1525.0980392f, 1537.6470588f, 1550.1960784f, 1562.745098f, 1575.2941176f, 1587.8431372f, 1600.3921568f, 1612.9411764f, 1625.490196f, 1638.0392156f, 1650.5882351999999f, 1663.1372548f, 1675.6862744f, 1688.235294f, 1700.7843136f, 1713.3333332f, 1725.8823528f, 1738.4313723999999f, 1750.980392f, 1763.5294116f, 1776.0784312f, 1788.6274508f, 1801.1764704f, 1813.72549f, 1826.2745095999999f, 1838.8235292f, 1851.3725488f, 1863.9215684f, 1876.470588f, 1889.0196076f, 1901.5686272f, 1914.1176467999999f, 1926.6666664f, 1939.215686f, 1951.7647056f, 1964.3137252f, 1976.8627448f, 1989.4117644f, 2001.9607839999999f, 2014.5098036f, 2027.0588232f, 2039.6078428f, 2052.1568624f, 2064.705882f, 2077.2549016f, 2089.8039212f, 2102.3529408f, 2114.9019604f, 2127.45098f, 2139.9999996f, 2152.5490191999997f, 2165.0980388f, 2177.6470584f, 2190.196078f, 2202.7450976f, 2215.2941172f, 2227.8431368f, 2240.3921564f, 2252.941176f, 2265.4901956f, 2278.0392152f, 2290.5882348f};

uint8_t *AUDIO_BUFFER;

void write_pulse(float freq, float ms, uint32_t *offset);

void sstv_init();
void sstv_close();

float sstv_pixel_to_freq(uint16_t pix, uint8_t color);
void sstv_camera_line(camera_fb_t *pic, uint16_t line, uint8_t color, uint32_t *offset);
void sstv_callsign_line(uint16_t pic_width, uint16_t line, uint8_t color, uint32_t *offset);
void sstv_generate_audio(camera_fb_t *pic);

void sstv_init_timer();
void sstv_play_audio();

#endif /* _SSTV_H */
