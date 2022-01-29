#ifndef _CAMERA_H
#define _CAMERA_H

#include <esp_log.h>
#include <esp_system.h>
#include "esp_camera.h"

// Initialize camera.
esp_err_t camera_init(void);

#endif // _CAMERA_H