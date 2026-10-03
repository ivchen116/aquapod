#include "esp_camera.h"
#include "esp_err.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define PWDN_GPIO_NUM 17
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 8
#define SIOD_GPIO_NUM 21
#define SIOC_GPIO_NUM 16

#define Y9_GPIO_NUM 2
#define Y8_GPIO_NUM 7
#define Y7_GPIO_NUM 10
#define Y6_GPIO_NUM 14
#define Y5_GPIO_NUM 11
#define Y4_GPIO_NUM 15
#define Y3_GPIO_NUM 13
#define Y2_GPIO_NUM 12
#define VSYNC_GPIO_NUM 6
#define HREF_GPIO_NUM 4
#define PCLK_GPIO_NUM 9

static const char *TAG = "camera_base";

static void camera_task(void *param);
static esp_err_t camera_init(void);

static esp_err_t camera_init(void)
{
    camera_config_t config = {
        .ledc_channel = LEDC_CHANNEL_0,
        .ledc_timer = LEDC_TIMER_1,
        .pin_d0 = Y2_GPIO_NUM,
        .pin_d1 = Y3_GPIO_NUM,
        .pin_d2 = Y4_GPIO_NUM,
        .pin_d3 = Y5_GPIO_NUM,
        .pin_d4 = Y6_GPIO_NUM,
        .pin_d5 = Y7_GPIO_NUM,
        .pin_d6 = Y8_GPIO_NUM,
        .pin_d7 = Y9_GPIO_NUM,
        .pin_xclk = XCLK_GPIO_NUM,
        .pin_pclk = PCLK_GPIO_NUM,
        .pin_vsync = VSYNC_GPIO_NUM,
        .pin_href = HREF_GPIO_NUM,
        .pin_sccb_sda = SIOD_GPIO_NUM,
        .pin_sccb_scl = SIOC_GPIO_NUM,
        .pin_pwdn = PWDN_GPIO_NUM,
        .pin_reset = RESET_GPIO_NUM,
        .xclk_freq_hz = 20000000,
        .frame_size = FRAMESIZE_HVGA,
        .pixel_format = PIXFORMAT_RGB565,
        .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
        .fb_location = CAMERA_FB_IN_PSRAM,
        .jpeg_quality = 12,
        .fb_count = 1,
    };

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed: 0x%x", err);
        return err;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (s != NULL) {
        s->set_hmirror(s, 1);
    }

    ESP_LOGI(TAG, "Camera initialized");
    return ESP_OK;
}

static void camera_task(void *param)
{
    (void)param;

    while (1) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb == NULL) {
            ESP_LOGW(TAG, "Failed to get camera frame");
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        ESP_LOGI(TAG, "Frame captured: len=%u, width=%u, height=%u, format=%d",
                 (unsigned)fb->len,
                 (unsigned)fb->width,
                 (unsigned)fb->height,
                 (int)fb->format);

        esp_camera_fb_return(fb);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main1(void)
{
    if (camera_init() != ESP_OK) {
        ESP_LOGE(TAG, "Camera setup failed, stop boot process");
        return;
    }

    xTaskCreatePinnedToCore(camera_task, "camera_task", 4096, NULL, 5, NULL, 0);
}
