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

// -------------------------------------- //
// Declare error handlers for ESP32 code. //
// -------------------------------------- //

#ifndef _ERROR_HANDLING_H
#define _ERROR_HANDLING_H

#include <esp_system.h>
#include <sys/param.h>
#include <esp_log.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// If error is detected, return it.
#define ESP_ERROR_VALIDATE(tag, err, ret_val)        \
    err = ret_val;                                   \
    if (err != ESP_OK)                               \
    {                                                \
        ESP_LOGE(tag, "Failed! Returning Error..."); \
        return err;                                  \
    }

// If error is detected within a for loop, just wait and go to the next iteration.
#define ESP_ERROR_CONTINUE_VALIDATE(tag, err, wait_ms, ret_val) \
    err = ret_val;                                              \
    if (err != ESP_OK)                                          \
    {                                                           \
        ESP_LOGE(tag, "Failed! Continuing...");                 \
        if (wait_ms > 0)                                        \
        {                                                       \
            vTaskDelay(wait_ms / portTICK_PERIOD_MS);           \
        }                                                       \
        continue;                                               \
    }

// If error is detected, keep looping with task delay until error resolves itself.
// Also provides option to set a fixed number of iterations. If not achieved, abort.
void ESP_ERROR_REDO_VALIDATE(const char *tag, esp_err_t (*func)(), uint16_t wait_ms, uint16_t timeout_iters);

#endif // _ERROR_HANDLING_H