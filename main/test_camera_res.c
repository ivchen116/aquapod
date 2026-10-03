/**
 * test_camera_res.c — Standalone Camera Resolution & Frame Rate Test
 *
 * Usage:
 *   1. Modify main/CMakeLists.txt: replace "main.c" with "test_camera_res.c"
 *   2. Build and flash as usual:  idf.py build flash monitor
 *   3. Results are printed to serial console (ESP_LOG / printf).
 *
 * This test cycles through common resolutions, captures frames for a
 * fixed duration at each setting, and reports:
 *   - Actual frame rate (fps)
 *   - Frame dimensions (width × height)
 *   - Average JPEG size (bytes)
 *   - Min / max frame interval
 *
 * It also runs a JPEG quality sweep at a mid-range resolution so you
 * can see the quality ↔ size ↔ fps trade-off.
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_camera.h"

#define TAG "cam_test"

/* ---- Pin definitions (same as camera_service.c) ---- */
#define PWDN_GPIO_NUM   17
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM    8
#define SIOD_GPIO_NUM   21
#define SIOC_GPIO_NUM   16

#define Y9_GPIO_NUM      2
#define Y8_GPIO_NUM      7
#define Y7_GPIO_NUM     10
#define Y6_GPIO_NUM     14
#define Y5_GPIO_NUM     11
#define Y4_GPIO_NUM     15
#define Y3_GPIO_NUM     13
#define Y2_GPIO_NUM     12
#define VSYNC_GPIO_NUM   6
#define HREF_GPIO_NUM    4
#define PCLK_GPIO_NUM    9

/* ---- Test parameters ---- */
#define TEST_DURATION_MS    5000    /* capture duration per resolution  */
#define FRAME_TIMEOUT_MS    2000    /* max wait for a single frame     */
#define MAX_FRAMES          300     /* safety limit for frame arrays   */
#define FB_COUNT            6       /* frame buffers for driver        */
#define TEST_TASK_STACK_SIZE 12288   /* ample stack for printf + camera ops */

/* ---- Resolution list to test ---- */
typedef struct {
    framesize_t  frame_size;
    const char  *name;
    int          expected_w;
    int          expected_h;
} res_test_t;

static const res_test_t s_res_list[] = {
    // { FRAMESIZE_96X96,   "96x96",    96,   96   },
    // { FRAMESIZE_QQVGA,   "QQVGA",   160,  120  },
    // { FRAMESIZE_QCIF,    "QCIF",    176,  144  },
    // { FRAMESIZE_HQVGA,   "HQVGA",   240,  176  },
    // { FRAMESIZE_QVGA,    "QVGA",    320,  240  },
    // { FRAMESIZE_CIF,     "CIF",     400,  296  },
    { FRAMESIZE_HVGA,    "HVGA",    480,  320  },
    { FRAMESIZE_VGA,     "VGA",     640,  480  },
    { FRAMESIZE_SVGA,    "SVGA",    800,  600  },
    { FRAMESIZE_XGA,     "XGA",    1024,  768  },
    { FRAMESIZE_HD,      "HD",     1280,  720  },
    { FRAMESIZE_SXGA,    "SXGA",   1280, 1024  },
    { FRAMESIZE_UXGA,    "UXGA",   1600, 1200  },
    { FRAMESIZE_FHD,     "FHD",    1920, 1080  },
};

/* ---- Static storage for frame stats ---- */
static int64_t  s_frame_timestamps[MAX_FRAMES];
static uint32_t s_frame_sizes[MAX_FRAMES];
static int      s_frame_count;
static int      s_actual_w = 0;
static int      s_actual_h = 0;

/* ---- Helpers ---- */

static int64_t now_ms(void)
{
    return (int64_t)xTaskGetTickCount() * portTICK_PERIOD_MS;
}

/* ---- Camera init / deinit ---- */

static bool camera_init_for_test(framesize_t fsize, int jpeg_quality)
{
    /* static to avoid ~160-byte local on stack — esp_camera_init copies it */
    static camera_config_t config;

    memset(&config, 0, sizeof(config));
    config = (camera_config_t){
        .ledc_channel    = LEDC_CHANNEL_0,
        .ledc_timer      = LEDC_TIMER_1,
        .pin_d0          = Y2_GPIO_NUM,
        .pin_d1          = Y3_GPIO_NUM,
        .pin_d2          = Y4_GPIO_NUM,
        .pin_d3          = Y5_GPIO_NUM,
        .pin_d4          = Y6_GPIO_NUM,
        .pin_d5          = Y7_GPIO_NUM,
        .pin_d6          = Y8_GPIO_NUM,
        .pin_d7          = Y9_GPIO_NUM,
        .pin_xclk        = XCLK_GPIO_NUM,
        .pin_pclk        = PCLK_GPIO_NUM,
        .pin_vsync       = VSYNC_GPIO_NUM,
        .pin_href        = HREF_GPIO_NUM,
        .pin_sccb_sda    = SIOD_GPIO_NUM,
        .pin_sccb_scl    = SIOC_GPIO_NUM,
        .pin_pwdn        = PWDN_GPIO_NUM,
        .pin_reset       = RESET_GPIO_NUM,
        .xclk_freq_hz    = 20000000,
        .frame_size      = fsize,
        .pixel_format    = PIXFORMAT_JPEG,
        .grab_mode       = CAMERA_GRAB_LATEST,
        .fb_location     = CAMERA_FB_IN_PSRAM,
        .jpeg_quality    = jpeg_quality,
        .fb_count        = FB_COUNT,
    };

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "camera_init failed: 0x%x (%s)", err, esp_err_to_name(err));
        return false;
    }

    /* Apply mirror/flip if you need them (same as production) */
    sensor_t *s = esp_camera_sensor_get();
    if (s != NULL) {
        s->set_hmirror(s, 1);
        s->set_vflip(s, 0);
    }

    /* Let AGC/AEC settle */
    vTaskDelay(pdMS_TO_TICKS(800));

    return true;
}

static void camera_deinit_for_test(void)
{
    esp_camera_deinit();
}

/* ---- Warm-up: discard first few frames so AGC/AEC stabilises ---- */

static void camera_warmup(void)
{
    ESP_LOGI(TAG, "  Warm-up: discarding first 10 frames...");
    for (int i = 0; i < 10; i++) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb != NULL) {
            esp_camera_fb_return(fb);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* ---- Capture loop ---- */

static void camera_capture_loop(int duration_ms)
{
    s_frame_count = 0;
    int64_t t_start = now_ms();
    int64_t t_deadline = t_start + duration_ms;

    memset(s_frame_timestamps, 0, sizeof(s_frame_timestamps));
    memset(s_frame_sizes, 0, sizeof(s_frame_sizes));

    ESP_LOGI(TAG, "  Capturing for %d ms...", duration_ms);

    while (now_ms() < t_deadline && s_frame_count < MAX_FRAMES) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb != NULL) {
            int64_t t_now = now_ms();
            if (s_frame_count == 0) {
                s_actual_w = (int)fb->width;
                s_actual_h = (int)fb->height;
            }
            s_frame_timestamps[s_frame_count] = t_now;
            s_frame_sizes[s_frame_count]     = (uint32_t)fb->len;
            s_frame_count++;
            esp_camera_fb_return(fb);
        }
        /* Brief yield — esp_camera_fb_get() blocks internally, but
         * a tiny delay prevents tight-loop starvation of other tasks. */
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

/* ---- Statistics ---- */

static void print_stats(int expected_w, int expected_h)
{
    if (s_frame_count < 2) {
        printf("  *** NOT ENOUGH FRAMES (%d) — skip ***\n", s_frame_count);
        return;
    }

    int64_t t_first = s_frame_timestamps[0];
    int64_t t_last  = s_frame_timestamps[s_frame_count - 1];
    double total_sec = (double)(t_last - t_first) / 1000.0;

    double avg_fps = (double)(s_frame_count - 1) / total_sec; /* intervals, not frames */

    /* Min / max interval */
    int64_t min_iv = INT64_MAX, max_iv = 0;
    double sum_iv = 0.0;
    for (int i = 1; i < s_frame_count; i++) {
        int64_t iv = s_frame_timestamps[i] - s_frame_timestamps[i - 1];
        if (iv < min_iv) min_iv = iv;
        if (iv > max_iv) max_iv = iv;
        sum_iv += (double)iv;
    }
    double avg_iv_ms = sum_iv / (double)(s_frame_count - 1);

    /* Average JPEG size */
    double sum_size = 0.0;
    for (int i = 0; i < s_frame_count; i++) {
        sum_size += (double)s_frame_sizes[i];
    }
    double avg_size = sum_size / s_frame_count;

    printf("  Resolution:  %4d x %-4d  (expected %d x %d)\n",
           s_actual_w, s_actual_h, expected_w, expected_h);
    printf("  Frames:      %d captured\n", s_frame_count);
    printf("  Avg FPS:     %.1f\n", avg_fps);
    printf("  Avg interval: %.1f ms  (min %lld ms, max %lld ms)\n",
           avg_iv_ms, (long long)min_iv, (long long)max_iv);
    printf("  Avg JPEG:    %.1f KB  (%.1f kB)\n", avg_size / 1024.0, avg_size / 1000.0);
}

/* ---- Resolution sweep ---- */

static void test_resolution_sweep(int jpeg_quality)
{
    printf("\n"
           "============================================================\n"
           "  RESOLUTION SWEEP  (JPEG quality = %d)\n"
           "============================================================\n",
           jpeg_quality);

    for (size_t i = 0; i < sizeof(s_res_list) / sizeof(s_res_list[0]); i++) {

        framesize_t fsize = s_res_list[i].frame_size;
        const char *name  = s_res_list[i].name;
        int ew = s_res_list[i].expected_w;
        int eh = s_res_list[i].expected_h;

        printf("\n--- %s (%d x %d) ---\n", name, ew, eh);

        if (!camera_init_for_test(fsize, jpeg_quality)) {
            printf("  *** INIT FAILED — skipping ***\n");
            continue;
        }

        camera_warmup();
        camera_capture_loop(TEST_DURATION_MS);

        print_stats(ew, eh);

        camera_deinit_for_test();
        vTaskDelay(pdMS_TO_TICKS(500));  /* breathe between runs */
    }
}

/* ---- JPEG quality sweep at a fixed resolution ---- */

static void test_jpeg_quality_sweep(void)
{
    printf("\n"
           "============================================================\n"
           "  JPEG QUALITY SWEEP  (resolution = 1080p)\n"
           "  quality: lower number = higher quality = bigger file\n"
           "============================================================\n");

    /* Test quality from 0 (best) to 50 (smallest) in steps */
    const int qualities[] = { 5, 8, 10, 12, 15, 20, 30, 40, 50 };
    const int nq = sizeof(qualities) / sizeof(qualities[0]);

    printf("\n%-10s %10s %12s %12s %10s\n",
           "Quality", "FPS", "Avg JPEG(B)", "Avg JPEG(KiB)", "Interval(ms)");
    printf("--------------------------------------------------------------\n");

    for (int qi = 0; qi < nq; qi++) {
        int q = qualities[qi];

        if (!camera_init_for_test(FRAMESIZE_FHD, q)) {
            printf("%-10d *** INIT FAILED ***\n", q);
            continue;
        }

        camera_warmup();
        camera_capture_loop(TEST_DURATION_MS);

        if (s_frame_count >= 2) {
            int64_t t_first = s_frame_timestamps[0];
            int64_t t_last  = s_frame_timestamps[s_frame_count - 1];
            double total_sec = (double)(t_last - t_first) / 1000.0;
            double fps = (double)(s_frame_count - 1) / total_sec;

            double sum_size = 0.0;
            for (int i = 0; i < s_frame_count; i++) {
                sum_size += s_frame_sizes[i];
            }
            double avg_size = sum_size / s_frame_count;
            double avg_iv = total_sec / (double)(s_frame_count - 1) * 1000.0;

            printf("%-10d %10.1f %12.0f %12.1f %10.1f\n",
                   q, fps, avg_size, avg_size / 1024.0, avg_iv);
        } else {
            printf("%-10d %10s %12s %12s %10s\n",
                   q, "—", "—", "—", "—");
        }

        camera_deinit_for_test();
        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

/* ---- Sensor info ---- */

static void print_sensor_info(void)
{
    /* Need a temporary camera init to read sensor */
    if (!camera_init_for_test(FRAMESIZE_QVGA, 12)) {
        printf("Cannot detect sensor — camera init failed\n");
        return;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (s != NULL) {
        printf("Sensor PID:  0x%04X\n", s->id.PID);
        printf("Sensor VER:  0x%02X\n", s->id.VER);
        printf("I2C addr:    0x%02X\n", s->slv_addr);

        /* Model name lookup (PID values from camera_pid_t enum) */
        const char *model_name = "Unknown";
        switch (s->id.PID) {
        case 0x5640: model_name = "OV5640"; break;
        case 0x26:   model_name = "OV2640"; break;
        case 0x3660: model_name = "OV3660"; break;
        case 0x77:   model_name = "OV7725"; break;
        case 0x76:   model_name = "OV7670"; break;
        case 0x2145: model_name = "GC2145"; break;
        case 0x232a: model_name = "GC032A"; break;
        case 0x9b:   model_name = "GC0308"; break;
        case 0x30:   model_name = "BF3005"; break;
        case 0x20a6: model_name = "BF20A6"; break;
        case 0xda4a: model_name = "SC101IOT"; break;
        case 0x9a46: model_name = "SC030IOT"; break;
        case 0x0031: model_name = "SC031GS"; break;
        case 0x039E: model_name = "MEGA_CCM"; break;
        case 0x1410: model_name = "NT99141"; break;
        }
        printf("Model:       %s\n", model_name);
    }

    camera_deinit_for_test();
}

/* ---- Test task (runs at low priority on core 1) ---- */

static void test_task_fn(void *arg)
{
    (void)arg;

    printf("\n");
    printf("============================================================\n");
    printf("  AquaPod - Camera Resolution & FPS Test\n");
    printf("============================================================\n");
    printf("Platform: ESP32-S3  |  XCLK: 20 MHz  |  PSRAM: OCTO 80MHz\n");
    printf("Capture duration per test: %d ms\n", TEST_DURATION_MS);
    printf("Frame buffer count: %d  |  Grab mode: LATEST\n\n", FB_COUNT);

    /* 1. Print sensor info */
    printf("--- Sensor Detection ---\n");
    print_sensor_info();

    /* 2. Resolution sweep at default quality (similar to production) */
    test_resolution_sweep(10);

    /* 3. JPEG quality sweep at VGA */
    test_jpeg_quality_sweep();

    printf("\n"
           "============================================================\n"
           "  TEST COMPLETE\n"
           "============================================================\n\n");

    /* Sit idle so the user can read the output */
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* ---- Entry point (thin — spawns test task, then returns) ---- */

void app_main(void)
{
    /* 1. NVS (may be needed by camera sensor power-up) */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* 2. Spawn test task with large stack — app_main only has 3584 B,
     *    which is insufficient for printf with float formatting +
     *    camera_config_t on the stack of deeply nested calls. */
    xTaskCreatePinnedToCore(
        test_task_fn, "cam_test",
        TEST_TASK_STACK_SIZE,
        NULL,
        2,      /* low priority — no contention with WiFi/HTTP */
        NULL,
        1       /* core 1 — same as camera_task in production */
    );

    /* app_main returns; the idle task reclaims its stack */
}
