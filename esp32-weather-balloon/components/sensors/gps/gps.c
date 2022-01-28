#include "gps.h"
#include "error_handling.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nmea_parser.h"

#define GPS_TAG ("GPS")

#define TIME_ZONE (+0)   //UTC Time
#define YEAR_BASE (2000) //date in GPS starts from 2000

// Global variables for the GPS system
nmea_parser_handle_t nmea_hdl;
gps_data_out_t current_data;

/**
 * @brief GPS Event Handler
 *
 * @param event_handler_arg handler specific arguments
 * @param event_base event base, here is fixed to ESP_NMEA_EVENT
 * @param event_id event id
 * @param event_data event specific arguments
 */
void gps_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    gps_t *gps = NULL;
    switch (event_id)
    {
    case GPS_UPDATE:
        gps = (gps_t *)event_data;
        /* print information parsed from GPS statements 
        ESP_LOGI(GPS_TAG, "%d/%d/%d %d:%d:%d => \r\n"
                          "\t\t\t\t\t\tlatitude   = %.05f°N\r\n"
                          "\t\t\t\t\t\tlongitude = %.05f°E\r\n"
                          "\t\t\t\t\t\taltitude   = %.02fm\r\n"
                          "\t\t\t\t\t\tspeed      = %fm/s",
                 gps->date.year + YEAR_BASE, gps->date.month, gps->date.day,
                 gps->tim.hour + TIME_ZONE, gps->tim.minute, gps->tim.second,
                 gps->latitude, gps->longitude, gps->altitude, gps->speed);*/

        current_data = (gps_data_out_t){.latitude = gps->latitude,
                                        .longitude = gps->longitude,
                                        .altitude = gps->altitude,
                                        .speed = gps->speed};

        break;
    case GPS_UNKNOWN:
        /* print unknown statements */
        ESP_LOGW(GPS_TAG, "Unknown statement:%s", (char *)event_data);
        break;
    default:
        break;
    }
}

esp_err_t gps_init()
{
    // Init error.
    esp_err_t err = ESP_OK;

    current_data = (gps_data_out_t){.latitude = 0.0f,
                                    .longitude = 0.0f,
                                    .altitude = 0.0f,
                                    .speed = 0.0f};

    // NMEA parser configuration
    nmea_parser_config_t config = NMEA_PARSER_CONFIG_DEFAULT();

    // Init NMEA parser library
    nmea_hdl = nmea_parser_init(&config);

    // Register event handler for NMEA parser library
    ESP_ERROR_VALIDATE("NMEA Add Handler",
                       err,
                       nmea_parser_add_handler(nmea_hdl, gps_event_handler, NULL))

    return ESP_OK;
}

esp_err_t gps_deinit()
{
    // Init error.
    esp_err_t err = ESP_OK;

    // Unregister event handler
    ESP_ERROR_VALIDATE("NMEA Remove Handler",
                       err,
                       nmea_parser_remove_handler(nmea_hdl, gps_event_handler))

    // Deinit NMEA parser library
    ESP_ERROR_VALIDATE("NMEA Deinit",
                       err,
                       nmea_parser_deinit(nmea_hdl))
    return ESP_OK;
}

void gps_read(gps_data_out_t *data)
{
    // Copy data in current buffer from NMEA event loop to output.
    *data = current_data;
}