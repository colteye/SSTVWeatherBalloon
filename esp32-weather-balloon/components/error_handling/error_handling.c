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

// ------------------------------------- //
// Define error handlers for ESP32 code. //
// ------------------------------------- //

#include "error_handling.h"

void ESP_ERROR_REDO_ABORT_VALIDATE(const char *tag, esp_err_t (*func)(), uint16_t wait_ms, uint16_t timeout_iters)
{
    esp_err_t err = func();
    if (err != ESP_OK)
    {
        ESP_LOGE(tag, "Failed! Retrying...");
        // If timeout iterations set.
        // Abort device if unable to exit loop.
        if (timeout_iters > 0)
        {
            for (uint16_t i = 0; i < timeout_iters; ++i)
            {
                if (wait_ms > 0)
                {
                    vTaskDelay(wait_ms / portTICK_PERIOD_MS);
                }
                err = func();

                if (err == ESP_OK)
                    break;
            }

            // Abort (set to panic mode) if unable to exit error.
            abort();
        }
        // Otherwise, allow forever looping.
        else
        {
            while (err != ESP_OK)
            {
                vTaskDelay(wait_ms / portTICK_PERIOD_MS);
                err = func();
            }
        }

        ESP_LOGI(tag, "Fixed! Continuing...");
    }
}

void ESP_ERROR_REDO_TASK_VALIDATE(const char *tag, esp_err_t (*func)(), uint16_t wait_ms, uint16_t timeout_iters)
{
    esp_err_t err = func();
    if (err != ESP_OK)
    {
        ESP_LOGE(tag, "Failed! Retrying...");
        // If timeout iterations set.
        // Abort device if unable to exit loop.
        if (timeout_iters > 0)
        {
            for (uint16_t i = 0; i < timeout_iters; ++i)
            {
                if (wait_ms > 0)
                {
                    vTaskDelay(wait_ms / portTICK_PERIOD_MS);
                }
                err = func();

                if (err == ESP_OK)
                    break;
            }

            // Abort (set to panic mode) if unable to exit error.
            vTaskDelete(NULL);
        }
        // Otherwise, allow forever looping.
        else
        {
            while (err != ESP_OK)
            {
                vTaskDelay(wait_ms / portTICK_PERIOD_MS);
                err = func();
            }
        }

        ESP_LOGI(tag, "Fixed! Continuing...");
    }
}