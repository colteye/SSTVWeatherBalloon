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

#ifndef _RADIO_TRANSMITTER_H
#define _RADIO_TRANSMITTER_H

#include <esp_system.h>
#include <complex.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define HAM_CALLSIGN ("XXXXXX")
#define HAM_CALLSIGN_LEN (6)
#define HAM_SSID (6)

#define BYTE_SIZE (8)

// Waveform related defines

#define R_BITS_PER_SAMPLE (2)
#define R_SAMPLE_RATE (20000)
#define R_GPIO (GPIO_NUM_2)

#define R_NUM_PWM_LEVELS (R_BITS_PER_SAMPLE + 1)
#define R_PWM_RANGE_ADDR (R_SINE_MAX - R_SINE_MIN) / (float)R_NUM_PWM_LEVELS

#define R_WAVEFORM_TIMER_DIVIDER (APB_CLK_FREQ / (R_SAMPLE_RATE * R_BITS_PER_SAMPLE)) // Subtract to try to compensate for extra operations being done?
#define R_WAVEFORM_TIMER (1)

#define R_SINE_MIN (-0.25f)
#define R_SINE_MAX (0.25f)

typedef struct
{
    float pwm_range_addr;    // Sine oscillation range for PWM levels.
    uint8_t num_pwm_levels;  // Number of  PWM levels for generating signals.
    uint8_t bits_per_sample; // Number of bits in a single samples.
    uint16_t sample_rate;    // Sample rate (samples/second).
    int timer;               // Timer index.
    uint32_t timer_divider;  // Timer divider for radio timer initialization.
    uint8_t gpio;            // GPIO pin for radio output.
} radio_transmitter_config_t;

typedef struct
{
    float complex current_osc;     // Start value to help generate sine wave oscillations.
    uint32_t current_transmit_bit; // Used to keep track of waveform bit while transmitting.
    uint32_t current_offset_bit;   // Current bit of offset when writing pulses.

    uint8_t clip_seconds;     // Number of seconds in the buffer.
    uint32_t buffer_bits_len; // Number of bits in the buffer.
    uint8_t *buffer;          // Waveform buffer.
} radio_waveform_data_t;

radio_transmitter_config_t *radio_transmitter_get_config(void);
xSemaphoreHandle radio_transmitter_get_mutex(void);
xQueueHandle radio_transmitter_get_waveform_queue(void);

// Write a frequency for a specific number of time into the SSTV buffer as PWM.
// Uses code from https://github.com/brainwagon/sstv-encoders/blob/master/martin.c
esp_err_t radio_transmitter_write_pulse(float freq, float ms, radio_waveform_data_t *waveform);

esp_err_t radio_transmitter_init(radio_transmitter_config_t radio_config);

#endif // _RADIO_TRANSMITTER_H
