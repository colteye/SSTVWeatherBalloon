#ifndef _GPS_H
#define _GPS_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// Data collected by GPS.
typedef struct
{
    float latitude;  // Latitude (degrees)
    float longitude; // Longitude (degrees)
    float altitude;  // Altitude (meters)
    float speed;     //Ground speed (meters/second)
} gps_data_out_t;

esp_err_t gps_init(void);
esp_err_t gps_deinit(void);
void gps_read(gps_data_out_t *data);

#endif // _GPS_H