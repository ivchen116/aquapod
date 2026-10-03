/*
 * SPDX-FileCopyrightText: 2022-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_camera.h"
#include "usb_device_uvc.h"
#include "msc_disk.h"
#include "cdc_uart.h"
// #include "uvc_frame_config.h"


static const char *TAG = "usb_webcam";

#define CAMERA_XCLK_FREQ           20000000
#define CAMERA_FB_COUNT            2

#if CONFIG_IDF_TARGET_ESP32S3
#define UVC_MAX_FRAMESIZE_SIZE     (75*1024)
#else
#define UVC_MAX_FRAMESIZE_SIZE     (60*1024)
#endif

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

typedef struct {
    camera_fb_t *cam_fb_p;
    uvc_fb_t uvc_fb;
} fb_t;

static fb_t s_fb;

static esp_err_t camera_init(uint32_t xclk_freq_hz, pixformat_t pixel_format, framesize_t frame_size, int jpeg_quality, uint8_t fb_count)
{
    static bool inited = false;
    static uint32_t cur_xclk_freq_hz = 0;
    static pixformat_t cur_pixel_format = 0;
    static framesize_t cur_frame_size = 0;
    static int cur_jpeg_quality = 0;
    static uint8_t cur_fb_count = 0;

    if ((inited && cur_xclk_freq_hz == xclk_freq_hz && cur_pixel_format == pixel_format
            && cur_frame_size == frame_size && cur_fb_count == fb_count && cur_jpeg_quality == jpeg_quality)) {
        ESP_LOGD(TAG, "camera already inited");
        return ESP_OK;
    } else if (inited) {
        esp_camera_return_all();
        esp_camera_deinit();
        inited = false;
        ESP_LOGI(TAG, "camera RESTART");
    }
    // config = (camera_config_t){
    //     .ledc_channel    = LEDC_CHANNEL_0,
    //     .ledc_timer      = LEDC_TIMER_1,
    //     .pin_d0          = Y2_GPIO_NUM,
    //     .pin_d1          = Y3_GPIO_NUM,
    //     .pin_d2          = Y4_GPIO_NUM,
    //     .pin_d3          = Y5_GPIO_NUM,
    //     .pin_d4          = Y6_GPIO_NUM,
    //     .pin_d5          = Y7_GPIO_NUM,
    //     .pin_d6          = Y8_GPIO_NUM,
    //     .pin_d7          = Y9_GPIO_NUM,
    //     .pin_xclk        = XCLK_GPIO_NUM,
    //     .pin_pclk        = PCLK_GPIO_NUM,
    //     .pin_vsync       = VSYNC_GPIO_NUM,
    //     .pin_href        = HREF_GPIO_NUM,
    //     .pin_sccb_sda    = SIOD_GPIO_NUM,
    //     .pin_sccb_scl    = SIOC_GPIO_NUM,
    //     .pin_pwdn        = PWDN_GPIO_NUM,
    //     .pin_reset       = RESET_GPIO_NUM,
    //     .xclk_freq_hz    = 20000000,
    //     .frame_size      = fsize,
    //     .pixel_format    = PIXFORMAT_JPEG,
    //     .grab_mode       = CAMERA_GRAB_LATEST,
    //     .fb_location     = CAMERA_FB_IN_PSRAM,
    //     .jpeg_quality    = jpeg_quality,
    //     .fb_count        = FB_COUNT,
    // };
    camera_config_t camera_config = {
        .pin_pwdn = PWDN_GPIO_NUM,
        .pin_reset = RESET_GPIO_NUM,
        .pin_xclk = XCLK_GPIO_NUM,
        .pin_sccb_sda = SIOD_GPIO_NUM,
        .pin_sccb_scl = SIOC_GPIO_NUM,

        .pin_d0 = Y2_GPIO_NUM,
        .pin_d1 = Y3_GPIO_NUM,
        .pin_d2 = Y4_GPIO_NUM,
        .pin_d3 = Y5_GPIO_NUM,
        .pin_d4 = Y6_GPIO_NUM,
        .pin_d5 = Y7_GPIO_NUM,
        .pin_d6 = Y8_GPIO_NUM,
        .pin_d7 = Y9_GPIO_NUM,
        .pin_vsync = VSYNC_GPIO_NUM,
        .pin_href = HREF_GPIO_NUM,
        .pin_pclk = PCLK_GPIO_NUM,

        .xclk_freq_hz = xclk_freq_hz,
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,

        .pixel_format = pixel_format,
        .frame_size = frame_size,

        .jpeg_quality = jpeg_quality,
        .fb_count = fb_count,
        .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
        .fb_location = CAMERA_FB_IN_PSRAM
    };

    // initialize the camera sensor
    esp_err_t ret = esp_camera_init(&camera_config);
    if (ret != ESP_OK) {
        return ret;
    }

    // Get the sensor object, and then use some of its functions to adjust the parameters when taking a photo.
    // Note: Do not call functions that set resolution, set picture format and PLL clock,
    // If you need to reset the appeal parameters, please reinitialize the sensor.
    sensor_t *s = esp_camera_sensor_get();
    s->set_vflip(s, 1); // flip it back
    // initial sensors are flipped vertically and colors are a bit saturated
    if (s->id.PID == OV3660_PID) {
        s->set_brightness(s, 1); // up the blightness just a bit
        s->set_saturation(s, -2); // lower the saturation
    }

    if (s->id.PID == OV3660_PID || s->id.PID == OV2640_PID) {
        s->set_vflip(s, 1); // flip it back
    } else if (s->id.PID == GC0308_PID) {
        s->set_hmirror(s, 0);
    } else if (s->id.PID == GC032A_PID) {
        s->set_vflip(s, 1);
    } else {
        s->set_hmirror(s, 1);
        s->set_vflip(s, 0);
    }

    // Get the basic information of the sensor.
    camera_sensor_info_t *s_info = esp_camera_sensor_get_info(&(s->id));

    if (ESP_OK == ret && PIXFORMAT_JPEG == pixel_format && s_info->support_jpeg == true) {
        cur_xclk_freq_hz = xclk_freq_hz;
        cur_pixel_format = pixel_format;
        cur_frame_size = frame_size;
        cur_jpeg_quality = jpeg_quality;
        cur_fb_count = fb_count;
        inited = true;
    } else {
        ESP_LOGE(TAG, "JPEG format is not supported");
        return ESP_ERR_NOT_SUPPORTED;
    }

    return ret;
}

static void camera_stop_cb(void *cb_ctx)
{
    (void)cb_ctx;
    ESP_LOGI(TAG, "Camera Stop");
}

static esp_err_t camera_start_cb(uvc_format_t format, int width, int height, int rate, void *cb_ctx)
{
    (void)cb_ctx;
    ESP_LOGI(TAG, "Camera Start");
    ESP_LOGI(TAG, "Format: %d, width: %d, height: %d, rate: %d", format, width, height, rate);
    framesize_t frame_size = FRAMESIZE_QVGA;
    int jpeg_quality = 14;

    if (format != UVC_FORMAT_JPEG) {
        ESP_LOGE(TAG, "Only support MJPEG format");
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (width == 320 && height == 240) {
        frame_size = FRAMESIZE_QVGA;
        jpeg_quality = 10;
    } else if (width == 480 && height == 320) {
        frame_size = FRAMESIZE_HVGA;
        jpeg_quality = 10;
    } else if (width == 640 && height == 480) {
        frame_size = FRAMESIZE_VGA;
        jpeg_quality = 12;
    } else if (width == 800 && height == 600) {
        frame_size = FRAMESIZE_SVGA;
        jpeg_quality = 14;
    } else if (width == 1280 && height == 720) {
        frame_size = FRAMESIZE_HD;
        jpeg_quality = 16;
    } else if (width == 1920 && height == 1080) {
        frame_size = FRAMESIZE_FHD;
        jpeg_quality = 16;
    } else {
        ESP_LOGE(TAG, "Unsupported frame size %dx%d", width, height);
        return ESP_ERR_NOT_SUPPORTED;
    }

    esp_err_t ret = camera_init(CAMERA_XCLK_FREQ, PIXFORMAT_JPEG, frame_size, jpeg_quality, CAMERA_FB_COUNT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "camera init failed");
        return ret;
    }

    return ESP_OK;
}

static uvc_fb_t* camera_fb_get_cb(void *cb_ctx)
{
    (void)cb_ctx;
    s_fb.cam_fb_p = esp_camera_fb_get();
    if (!s_fb.cam_fb_p) {
        return NULL;
    }
    s_fb.uvc_fb.buf = s_fb.cam_fb_p->buf;
    s_fb.uvc_fb.len = s_fb.cam_fb_p->len;
    s_fb.uvc_fb.width = s_fb.cam_fb_p->width;
    s_fb.uvc_fb.height = s_fb.cam_fb_p->height;
    s_fb.uvc_fb.format = s_fb.cam_fb_p->format;
    s_fb.uvc_fb.timestamp = s_fb.cam_fb_p->timestamp;

    if (s_fb.uvc_fb.len > UVC_MAX_FRAMESIZE_SIZE) {
        ESP_LOGE(TAG, "Frame size %d is larger than max frame size %d", s_fb.uvc_fb.len, UVC_MAX_FRAMESIZE_SIZE);
        esp_camera_fb_return(s_fb.cam_fb_p);
        return NULL;
    }
    return &s_fb.uvc_fb;
}

static void camera_fb_return_cb(uvc_fb_t *fb, void *cb_ctx)
{
    (void)cb_ctx;
    assert(fb == &s_fb.uvc_fb);
    esp_camera_fb_return(s_fb.cam_fb_p);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Selected Camera Board %s", "esp32s3");

    // Composite device: MSC disk backend (non-fatal when no card inserted)
    msc_disk_init();

    uint8_t *uvc_buffer = (uint8_t *)malloc(UVC_MAX_FRAMESIZE_SIZE);
    if (uvc_buffer == NULL) {
        ESP_LOGE(TAG, "malloc frame buffer fail");
        return;
    }

    uvc_device_config_t config = {
        .uvc_buffer = uvc_buffer,
        .uvc_buffer_size = UVC_MAX_FRAMESIZE_SIZE,
        .start_cb = camera_start_cb,
        .fb_get_cb = camera_fb_get_cb,
        .fb_return_cb = camera_fb_return_cb,
        .stop_cb = camera_stop_cb,
    };

    ESP_LOGI(TAG, "Format List");
    ESP_LOGI(TAG, "\tFormat(1) = %s", "MJPEG");
    ESP_LOGI(TAG, "Frame List");
    //ESP_LOGI(TAG, "\tFrame(1) = %d * %d @%dfps", UVC_FRAMES_INFO[0][0].width, UVC_FRAMES_INFO[0][0].height, UVC_FRAMES_INFO[0][0].rate);
#if CONFIG_CAMERA_MULTI_FRAMESIZE
    ESP_LOGI(TAG, "\tFrame(2) = %d * %d @%dfps", UVC_FRAMES_INFO[0][1].width, UVC_FRAMES_INFO[0][1].height, UVC_FRAMES_INFO[0][1].rate);
    ESP_LOGI(TAG, "\tFrame(3) = %d * %d @%dfps", UVC_FRAMES_INFO[0][2].width, UVC_FRAMES_INFO[0][2].height, UVC_FRAMES_INFO[0][2].rate);
    ESP_LOGI(TAG, "\tFrame(3) = %d * %d @%dfps", UVC_FRAMES_INFO[0][3].width, UVC_FRAMES_INFO[0][3].height, UVC_FRAMES_INFO[0][3].rate);
#endif

    ESP_ERROR_CHECK(uvc_device_config(0, &config));
    ESP_ERROR_CHECK(uvc_device_init());

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(100));

        // Composite device: CDC ACM heartbeat (proves the CDC port works)
        static uint32_t heartbeat_tick = 0;
        if (++heartbeat_tick % 20 == 0) {
            cdc_printf("aquapod alive: %lu0ms\r\n", (unsigned long)(heartbeat_tick / 20));
        }
    }
}
