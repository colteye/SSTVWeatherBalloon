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

#define BYTE_SIZE (8)

#define SINE_MIN (-0.25f)
#define SINE_MAX (0.25f)

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