#include "camera.h"
#include "error_handling.h"

// Camera pins used for ESP Camera.
#define CAM_PIN_PWDN 32
#define CAM_PIN_RESET -1 //software reset will be performed
#define CAM_PIN_XCLK 0
#define CAM_PIN_SIOD 26
#define CAM_PIN_SIOC 27

#define CAM_PIN_D7 35
#define CAM_PIN_D6 34
#define CAM_PIN_D5 39
#define CAM_PIN_D4 36
#define CAM_PIN_D3 21
#define CAM_PIN_D2 19
#define CAM_PIN_D1 18
#define CAM_PIN_D0 5
#define CAM_PIN_VSYNC 25
#define CAM_PIN_HREF 23
#define CAM_PIN_PCLK 22

esp_err_t camera_init()
{
    // Init error.
    esp_err_t err = ESP_OK;

    camera_config_t camera_config = {
        .pin_pwdn = CAM_PIN_PWDN,
        .pin_reset = CAM_PIN_RESET,
        .pin_xclk = CAM_PIN_XCLK,
        .pin_sscb_sda = CAM_PIN_SIOD,
        .pin_sscb_scl = CAM_PIN_SIOC,

        .pin_d7 = CAM_PIN_D7,
        .pin_d6 = CAM_PIN_D6,
        .pin_d5 = CAM_PIN_D5,
        .pin_d4 = CAM_PIN_D4,
        .pin_d3 = CAM_PIN_D3,
        .pin_d2 = CAM_PIN_D2,
        .pin_d1 = CAM_PIN_D1,
        .pin_d0 = CAM_PIN_D0,
        .pin_vsync = CAM_PIN_VSYNC,
        .pin_href = CAM_PIN_HREF,
        .pin_pclk = CAM_PIN_PCLK,

        // XCLK up to 20MHz for OV2640 double FPS (Experimental)
        // Usually set to 10 MHz
        // Set it to 15 MHz for now to avoid issues with timing
        .xclk_freq_hz = 16000000,
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,

        .pixel_format = PIXFORMAT_RGB565, //YUV422,GRAYSCALE,RGB565,JPEG
        .frame_size = FRAMESIZE_QVGA,     //QQVGA-UXGA Do not use sizes above QVGA when not JPEG

        .jpeg_quality = 10, //0-63 lower number means higher quality
        .fb_count = 1,      //if more than one, i2s runs in continuous mode. Use only with JPEG
        .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
    };

    ESP_ERROR_VALIDATE("Camera Configuration",
                       err,
                       esp_camera_init(&camera_config))

    ESP_LOGI("Camera Configuration", "Camera initialized successfully!");
    return ESP_OK;
}

void camera_read(camera_fb_t *pic)
{
    ESP_LOGI(TAG, "Taking picture...");
    pic = esp_camera_fb_get();

    // use pic->buf to access the image.
    ESP_LOGI(TAG, "Picture taken! Its size was: %zu bytes", pic->len);
    esp_camera_fb_return(pic);
}