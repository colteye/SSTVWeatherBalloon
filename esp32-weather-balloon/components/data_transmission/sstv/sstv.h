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

#include "esp32/rom/ets_sys.h"

// NOTE: FREERTOS timing needs to be set to 1000hz!
// This allows us to use vTaskDelay and still keep our watchdog timer
// Dont need to divide vTaskDelay by anything, as 1 tick = 1 ms

// Task which repeatedly takes pictures and transmits them via SSTV.
void sstv_task_entry(void *arg);

#endif // _SSTV_H
