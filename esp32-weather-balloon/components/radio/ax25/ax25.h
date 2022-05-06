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

// -------------------------------------------------------------------------------- //
// Declare different functions and variables for AX.25 transmission of sensor data. //
// -------------------------------------------------------------------------------- //

#ifndef _AX25_H
#define _AX25_H

#include "radio_transmitter.h"
#include "esp32/rom/ets_sys.h"

// NOTE: FREERTOS timing needs to be set to 1000hz!
// This allows us to use vTaskDelay and still keep our watchdog timer
// Dont need to divide vTaskDelay by anything, as 1 tick = 1 ms

// Generate AX25 waveform from image!
esp_err_t ax25_generate_waveform(uint8_t *data, uint8_t data_len, radio_waveform_data_t *sstv_buf);

#endif // _AX25_H
