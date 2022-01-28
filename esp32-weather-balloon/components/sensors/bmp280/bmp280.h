#ifndef _BMP280_H
#define _BMP280_H

#include <esp_system.h>

// Data collected by BMP280.
typedef struct
{
    float temperature; // Temperature (Celsius)
    float pressure;    // Pressure (Pascals)
    float altitude;    // Altitude (Meters)
} bmp280_data_out_t;

// Initialize the BMP280 chip.
esp_err_t bmp280_init(void);

// Reset the BMP280 chip.
esp_err_t bmp280_reset(void);

// Read all BMP280 data at once.
esp_err_t bmp280_read(bmp280_data_out_t *data);

#endif // _BMP280_H