#ifndef _BMP280_H
#define _BMP280_H

#include <esp_system.h>

// Sea level HPA for altitude calculations (depends on location and time!)
#define SEALEVEL_HPA (1018.0f)

// Info addresses
#define BMP280_ADDR (0x77)
#define BMP280_CHIPID (0x58)

// Info registers
#define BMP280_CHIPID_REG (0xD0)
#define BMP280_VERSION_REG (0xD1)

// Coefficient registers
#define BMP280_DIG_T1_REG (0x88)
#define BMP280_DIG_T2_REG (0x8A)
#define BMP280_DIG_T3_REG (0x8C)
#define BMP280_DIG_P1_REG (0x8E)
#define BMP280_DIG_P2_REG (0x90)
#define BMP280_DIG_P3_REG (0x92)
#define BMP280_DIG_P4_REG (0x94)
#define BMP280_DIG_P5_REG (0x96)
#define BMP280_DIG_P6_REG (0x98)
#define BMP280_DIG_P7_REG (0x9A)
#define BMP280_DIG_P8_REG (0x9C)
#define BMP280_DIG_P9_REG (0x9E)

// Other configuration registers
#define BMP280_SOFTRESET_REG (0xE0)
#define BMP280_CAL26_REG (0xE1)
#define BMP280_STATUS_REG (0xF3)
#define BMP280_CONTROL_REG (0xF4)
#define BMP280_CONFIG_REG (0xF5)

// Data registers
#define BMP280_PRESSUREDATA_REG (0xF7)
#define BMP280_TEMPDATA_REG (0xFA)

// Data sampling types
#define SAMPLING_NONE (0x00)
#define SAMPLING_X1 (0x01)
#define SAMPLING_X2 (0x02)
#define SAMPLING_X4 (0x03)
#define SAMPLING_X8 (0x04)
#define SAMPLING_X16 (0x05)

// Different collection modes
#define MODE_SLEEP (0x00)
#define MODE_FORCED (0x01)
#define MODE_NORMAL (0x03)
#define MODE_SOFT_RESET_CODE (0xB6)

// Filters
#define FILTER_OFF (0x00)
#define FILTER_X2 (0x01)
#define FILTER_X4 (0x02)
#define FILTER_X8 (0x03)
#define FILTER_X16 (0x04)

// Wait times
#define STANDBY_MS_1 (0x00)
#define STANDBY_MS_63 (0x01)
#define STANDBY_MS_125 (0x02)
#define STANDBY_MS_250 (0x03)
#define STANDBY_MS_500 (0x04)
#define STANDBY_MS_1000 (0x05)
#define STANDBY_MS_2000 (0x06)
#define STANDBY_MS_4000 (0x07)

// Saving coefficient register values for calculations
typedef struct
{
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;

    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
} bmp280_coefficients_t;

// Data collected by BMP280.
typedef struct
{
    float temperature;
    float pressure;
    float altitude;
} bmp280_data_out_t;

// Read calibration coefficient values for initialization.
esp_err_t bmp280_read_coeffs(bmp280_coefficients_t *coeffs);

// Initialize the BMP280 chip.
esp_err_t bmp280_init(void);

// Reset the BMP280 chip.
esp_err_t bmp280_reset(void);

// Read BMP280 temperature in Celsius.
esp_err_t bmp280_read_temperature(bmp280_data_out_t *output);

// Read BMP280 pressure in Pascals.
esp_err_t bmp280_read_pressure(bmp280_data_out_t *output);

// Read BMP280 altitude above sea level in Meters.
void bmp280_read_altitude(bmp280_data_out_t *output);

// Read all BMP280 data at once.
esp_err_t bmp280_read(bmp280_data_out_t *data);

#endif // _BMP280_H