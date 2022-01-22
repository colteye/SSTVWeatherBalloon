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

// -------------------------------------------------------------------------- //
// Declare different functions and variables for SSTV transmission of images. //
// SSTV Protocol: Scottie 1                                                   //
// -------------------------------------------------------------------------- //

#ifndef _SSTV_H
#define _SSTV_H

#include <complex.h>
#include <math.h>

#include "esp32/rom/ets_sys.h"
#include "driver/timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "camera.h"
#include "sstv_callsign.h"

// NOTE: FREERTOS timing needs to be set to 1000hz!
// This allows us to use vTaskDelay and still keep our watchdog timer
// Dont need to divide vTaskDelay by anything, as 1 tick = 1 ms

// Check range of a value between 2 values
#define IN_RANGE(low, high, x) (low <= x && x <= high)

// Pick color channel for SSTV scanlines
#define RED (0)
#define GREEN (1)
#define BLUE (2)

// Output buffer data
uint8_t *OUTPUT_BUFFER;
#define BITS_LEN (SAMPLE_RATE * CLIP_LENGTH * BITS_PER_SAMPLE)
#define OUTPUT_BUFFER_LEN (BITS_LEN / BYTE_SIZE)

// SSTV output related defines
#define BYTE_SIZE (8)
#define CLIP_LENGTH (110) // in seconds
#define BITS_PER_SAMPLE (2)
#define SAMPLE_RATE (40000)
#define OUTPUT_GPIO (2)

#define SSTV_NUM_PWM_LEVELS (BITS_PER_SAMPLE + 1)
#define SSTV_SINE_MIN (-0.25f)
#define SSTV_SINE_MAX (0.25f)
#define SSTV_PWM_RANGE_ADDR (SSTV_SINE_MAX - SSTV_SINE_MIN) / (float)SSTV_NUM_PWM_LEVELS

// Final image size should be 320 x 256.
#define CALLSIGN_HEIGHT (16)

#define SSTV_TIMER_DIVIDER (APB_CLK_FREQ / (SAMPLE_RATE * BITS_PER_SAMPLE)) //Subtract to try to compensate for extra operations being done?
#define SSTV_OUTPUT_TIMER (1)                                               // done with auto reload

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

// Frequency Lookup Tables for RGB565
static const float RB_FREQ_LUT[32] = {1500.0f, 1525.0980392f, 1550.1960784f, 1575.2941176f, 1600.3921568f,
                                      1625.490196f, 1650.5882351999999f, 1675.6862744f, 1700.7843136f, 1725.8823528f,
                                      1750.980392f, 1776.0784312f, 1801.1764704f, 1826.2745095999999f, 1851.3725488f,
                                      1876.470588f, 1901.5686272f, 1926.6666664f, 1951.7647056f, 1976.8627448f,
                                      2001.9607839999999f, 2027.0588232f, 2052.1568624f, 2077.2549016f, 2102.3529408f,
                                      2127.45098f, 2152.5490191999997f, 2177.6470584f, 2202.7450976f, 2227.8431368f,
                                      2252.941176f, 2278.0392152f};

static const float G_FREQ_LUT[64] = {1500.0f, 1512.5490196f, 1525.0980392f, 1537.6470588f, 1550.1960784f,
                                     1562.745098f, 1575.2941176f, 1587.8431372f, 1600.3921568f, 1612.9411764f,
                                     1625.490196f, 1638.0392156f, 1650.5882351999999f, 1663.1372548f, 1675.6862744f,
                                     1688.235294f, 1700.7843136f, 1713.3333332f, 1725.8823528f, 1738.4313723999999f,
                                     1750.980392f, 1763.5294116f, 1776.0784312f, 1788.6274508f, 1801.1764704f,
                                     1813.72549f, 1826.2745095999999f, 1838.8235292f, 1851.3725488f, 1863.9215684f,
                                     1876.470588f, 1889.0196076f, 1901.5686272f, 1914.1176467999999f, 1926.6666664f,
                                     1939.215686f, 1951.7647056f, 1964.3137252f, 1976.8627448f, 1989.4117644f,
                                     2001.9607839999999f, 2014.5098036f, 2027.0588232f, 2039.6078428f, 2052.1568624f,
                                     2064.705882f, 2077.2549016f, 2089.8039212f, 2102.3529408f, 2114.9019604f,
                                     2127.45098f, 2139.9999996f, 2152.5490191999997f, 2165.0980388f, 2177.6470584f,
                                     2190.196078f, 2202.7450976f, 2215.2941172f, 2227.8431368f, 2240.3921564f,
                                     2252.941176f, 2265.4901956f, 2278.0392152f, 2290.5882348f};

// Write a frequency for a specific number of time into the SSTV buffer as PWM.
// Uses code from https://github.com/brainwagon/sstv-encoders/blob/master/martin.c
esp_err_t write_pulse(float freq, float ms, uint32_t *offset);

// Initialize GPIO and Timer for SSTV.
esp_err_t sstv_init(void);

// Initialize timer with SSTV options.
esp_err_t sstv_timer_init(void);

// Initialize or clear SSTV buffer.
esp_err_t sstv_buffer_init(void);
void sstv_buffer_clear(void);

// Convert pixel from specific color channel into a frequency.
float sstv_pixel_to_freq(uint16_t pixel, uint8_t color_channel);

// Generate SSTV waveform for a single line from either camera or callsign.
esp_err_t sstv_camera_line(camera_fb_t *pic, uint16_t line, uint8_t color_channel, uint32_t *offset);
esp_err_t sstv_callsign_line(uint16_t pic_width, uint16_t line, uint8_t color_channel, uint32_t *offset);

// Generate SSTV waveform of entire image.
esp_err_t sstv_generate_output(camera_fb_t *pic);

// Task which repeatedly takes pictures and transmits them via SSTV.
void SSTVCameraServiceTask(void *pvParameters);

#endif /* _SSTV_H */
