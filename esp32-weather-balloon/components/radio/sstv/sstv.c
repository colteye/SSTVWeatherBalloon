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

// ------------------------------------------------------------------------- //
// Define different functions and variables for SSTV transmission of images. //
// SSTV Protocol: Scottie 1                                                  //
// ------------------------------------------------------------------------- //

#include "sstv.h"
#include "sstv_luts.h"

#include "bit_ops.h"
#include "error_handling.h"

// Pick color channel for SSTV scanlines
#define RED (0)
#define GREEN (1)
#define BLUE (2)

// Final image size should be 320 x 256.
#define CALLSIGN_HEIGHT (16)

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

// Convert pixel from specific color channel into a frequency.
float sstv_pixel_to_freq(uint16_t pixel, uint8_t color_channel)
{
    // Init frequency float.
    float freq = 0.0;

    // Convert into index to turn into freq.
    // Based on RGB565 for bitmasking.
    if (color_channel == RED)
    {
        pixel = (pixel & 0b1111100000000000) >> 11;
        freq = RB_FREQ_LUT[pixel];
    }
    else if (color_channel == GREEN)
    {
        pixel = (pixel & 0b0000011111100000) >> 5;
        freq = G_FREQ_LUT[pixel];
    }
    else // BLUE
    {
        pixel = pixel & 0b0000000000011111;
        freq = RB_FREQ_LUT[pixel];
    }
    return freq;
}

// Generate SSTV waveform for a single line of the camera.
esp_err_t sstv_camera_line(camera_fb_t *pic, uint16_t line, uint8_t color_channel, radio_waveform_data_t *sstv_buf)
{
    // Init error.
    esp_err_t err = ESP_OK;

    // Start at current line.
    uint32_t l_idx = pic->width * 2 * line;

    // Iterate over length of width * 2 to account for 2 uint8 -> uint16_t.
    for (uint16_t i = 0; i < pic->width * 2; i += 2)
    {
        // Go to correct pixel, convert to frequency and write to the buffer.

        uint16_t pixel = U8S_2_U16((pic->buf[l_idx + i]), (pic->buf[l_idx + i + 1]));
        float freq = sstv_pixel_to_freq(pixel, color_channel);
        ESP_ERROR_VALIDATE("SSTV Camera Line",
                           err,
                           radio_transmitter_write_pulse(freq, SSTV_SCOTTIE_1_SCAN_MS, sstv_buf))
    }
    return ESP_OK;
}

// Generate SSTV waveform for a single line of the callsign.
esp_err_t sstv_callsign_line(uint16_t pic_width, uint16_t line, uint8_t color_channel, radio_waveform_data_t *sstv_buf)
{
    // Init error.
    esp_err_t err = ESP_OK;

    // Start at current line.
    uint32_t l_idx = pic_width * line;

    // Iterate over length of width.
    for (uint16_t i = 0; i < pic_width; ++i)
    {
        // Go to correct pixel, convert to frequency and write to the buffer.
        float freq = sstv_pixel_to_freq(SSTV_CALLSIGN[l_idx + i], color_channel);
        ESP_ERROR_VALIDATE("SSTV Callsign Line",
                           err,
                           radio_transmitter_write_pulse(freq, SSTV_SCOTTIE_1_SCAN_MS, sstv_buf))
    }
    return ESP_OK;
}

// Generate SSTV waveform of entire image.
esp_err_t sstv_generate_waveform(camera_fb_t *pic, radio_waveform_data_t *sstv_buf)
{
    // Init error.
    esp_err_t err = ESP_OK;

    sstv_buf->current_offset_bit = 0;

    // Initial optional vox tone.
    ESP_ERROR_VALIDATE("VOX Tone",
                       err,
                       radio_transmitter_write_pulse(SSTV_1900HZ, SSTV_100MS, sstv_buf))
    ESP_ERROR_VALIDATE("VOX Tone",
                       err,
                       radio_transmitter_write_pulse(SSTV_1500HZ, SSTV_100MS, sstv_buf))
    ESP_ERROR_VALIDATE("VOX Tone",
                       err,
                       radio_transmitter_write_pulse(SSTV_1900HZ, SSTV_100MS, sstv_buf))
    ESP_ERROR_VALIDATE("VOX Tone",
                       err,
                       radio_transmitter_write_pulse(SSTV_1500HZ, SSTV_100MS, sstv_buf))
    ESP_ERROR_VALIDATE("VOX Tone",
                       err,
                       radio_transmitter_write_pulse(SSTV_2300HZ, SSTV_100MS, sstv_buf))
    ESP_ERROR_VALIDATE("VOX Tone",
                       err,
                       radio_transmitter_write_pulse(SSTV_1500HZ, SSTV_100MS, sstv_buf))
    ESP_ERROR_VALIDATE("VOX Tone",
                       err,
                       radio_transmitter_write_pulse(SSTV_2300HZ, SSTV_100MS, sstv_buf))
    ESP_ERROR_VALIDATE("VOX Tone",
                       err,
                       radio_transmitter_write_pulse(SSTV_1500HZ, SSTV_100MS, sstv_buf))

    // Start tone
    ESP_ERROR_VALIDATE("Start Tone",
                       err,
                       radio_transmitter_write_pulse(SSTV_1900HZ, SSTV_300MS, sstv_buf))
    ESP_ERROR_VALIDATE("Start Tone",
                       err,
                       radio_transmitter_write_pulse(SSTV_1200HZ, SSTV_10MS, sstv_buf))
    ESP_ERROR_VALIDATE("Start Tone",
                       err,
                       radio_transmitter_write_pulse(SSTV_1900HZ, SSTV_300MS, sstv_buf))
    ESP_ERROR_VALIDATE("Start Tone",
                       err,
                       radio_transmitter_write_pulse(SSTV_1200HZ, SSTV_30MS, sstv_buf))

    // Playing VIS code.
    uint8_t code = SSTV_SCOTTIE_1_VIS_CODE;
    uint8_t parity = 0;
    for (uint8_t i = 0; i < 7; i++)
    {
        if (code & 1)
        {
            ESP_ERROR_VALIDATE("VIS Code",
                               err,
                               radio_transmitter_write_pulse(SSTV_1100HZ, SSTV_30MS, sstv_buf))
            ++parity;
        }
        else
        {
            ESP_ERROR_VALIDATE("VIS Code",
                               err,
                               radio_transmitter_write_pulse(SSTV_1300HZ, SSTV_30MS, sstv_buf))
        }
        code = code >> 1;
    }

    // Add parity!
    if (parity & 1)
    {
        ESP_ERROR_VALIDATE("VIS Parity",
                           err,
                           radio_transmitter_write_pulse(SSTV_1100HZ, SSTV_30MS, sstv_buf)) /* Output a 1 */
    }
    else
    {
        ESP_ERROR_VALIDATE("VIS Parity",
                           err,
                           radio_transmitter_write_pulse(SSTV_1300HZ, SSTV_30MS, sstv_buf)) /* Output a 0 */
    }

    // Stop bit.
    ESP_ERROR_VALIDATE("Stop Bit",
                       err,
                       radio_transmitter_write_pulse(SSTV_1200HZ, SSTV_30MS, sstv_buf))

    // First line pulse.
    ESP_ERROR_VALIDATE("First Pulse",
                       err,
                       radio_transmitter_write_pulse(SSTV_1200HZ, SSTV_SCOTTIE_1_PULSE_MS, sstv_buf))

    // Transmit callsign (320x16).
    for (uint16_t i = 0; i < CALLSIGN_HEIGHT; ++i)
    {
        ESP_ERROR_VALIDATE("Callsign Line Separator",
                           err,
                           radio_transmitter_write_pulse(SSTV_1500HZ, SSTV_SCOTTIE_1_SEPARATOR_MS, sstv_buf))

        ESP_ERROR_VALIDATE("Callsign Line Green",
                           err,
                           sstv_callsign_line(pic->width, i, GREEN, sstv_buf))

        ESP_ERROR_VALIDATE("Callsign Line Separator",
                           err,
                           radio_transmitter_write_pulse(SSTV_1500HZ, SSTV_SCOTTIE_1_SEPARATOR_MS, sstv_buf))

        ESP_ERROR_VALIDATE("Callsign Line Blue",
                           err,
                           sstv_callsign_line(pic->width, i, BLUE, sstv_buf))

        ESP_ERROR_VALIDATE("Callsign Pulse",
                           err,
                           radio_transmitter_write_pulse(SSTV_1200HZ, SSTV_SCOTTIE_1_PULSE_MS, sstv_buf))
        ESP_ERROR_VALIDATE("Callsign Porch",
                           err,
                           radio_transmitter_write_pulse(SSTV_1500HZ, SSTV_SCOTTIE_1_PORCH_MS, sstv_buf))

        ESP_ERROR_VALIDATE("Callsign Line Red",
                           err,
                           sstv_callsign_line(pic->width, i, RED, sstv_buf))
    }

    // Transmit image (320x240).
    // Using QVGA from camera
    for (uint16_t i = 0; i < pic->height; ++i)
    {

        ESP_ERROR_VALIDATE("Camera Line Separator",
                           err,
                           radio_transmitter_write_pulse(SSTV_1500HZ, SSTV_SCOTTIE_1_SEPARATOR_MS, sstv_buf))

        ESP_ERROR_VALIDATE("Camera Line Green",
                           err,
                           sstv_camera_line(pic, i, GREEN, sstv_buf))

        ESP_ERROR_VALIDATE("Camera Line Separator",
                           err,
                           radio_transmitter_write_pulse(SSTV_1500HZ, SSTV_SCOTTIE_1_SEPARATOR_MS, sstv_buf))

        ESP_ERROR_VALIDATE("Camera Line Blue",
                           err,
                           sstv_camera_line(pic, i, BLUE, sstv_buf))

        ESP_ERROR_VALIDATE("Camera Pulse",
                           err,
                           radio_transmitter_write_pulse(SSTV_1200HZ, SSTV_SCOTTIE_1_PULSE_MS, sstv_buf))
        ESP_ERROR_VALIDATE("Camera Porch",
                           err,
                           radio_transmitter_write_pulse(SSTV_1500HZ, SSTV_SCOTTIE_1_PORCH_MS, sstv_buf))

        ESP_ERROR_VALIDATE("Camera Line Red",
                           err,
                           sstv_camera_line(pic, i, RED, sstv_buf))
    }

    return ESP_OK;
}