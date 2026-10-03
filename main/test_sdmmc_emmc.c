/**
 * test_sdmmc_emmc.c — SDMMC 1-bit Mode Read/Write Speed Test for ESP32S3
 *
 * This test uses the ESP32S3's native SDMMC peripheral in 1-bit mode to
 * benchmark SD card read/write performance at different clock speeds.
 *
 * Features:
 *   - 1-bit native SDMMC mode (NOT SPI) — same 3 pins as SPI wiring
 *   - 40 MHz and 20 MHz clock speed comparison
 *   - VFS/FAT filesystem write speed benchmark (multiple block sizes)
 *   - VFS/FAT filesystem read  speed benchmark (multiple block sizes)
 *   - Raw sector read/write test (bypasses filesystem overhead)
 *   - Card info dump (type, capacity, speed class, etc.)
 *
 * Usage:
 *   1. SD card wired to the GPIOs defined below (same pins as SPI mode)
 *   2. Build:  idf.py build flash monitor
 *      (This file is already in main/CMakeLists.txt)
 *   3. Results are printed to the serial console
 *
 * GPIO Pinout (1-bit SDMMC — same physical pins as SPI SD card):
 *   CLK: GPIO 39  (was SCK in SPI mode)
 *   CMD: GPIO 38  (was MOSI in SPI mode)
 *   D0:  GPIO 40  (was MISO in SPI mode)
 *   D1/D2/D3: not used in 1-bit mode
 *   CS: not needed in SDMMC mode
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/stat.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "nvs_flash.h"

#define TAG "sdmmc_emmc"

/* ================================================================
 *  GPIO Pin Configuration  (1-bit SDMMC, same pins as SPI mode)
 * ================================================================ */
#define SDMMC_CLK_GPIO  39   /* = SD_SCK  (SPI clock)  */
#define SDMMC_CMD_GPIO  38   /* = SD_MOSI (SPI MOSI)   */
#define SDMMC_D0_GPIO   40   /* = SD_MISO (SPI MISO)   */
/* D1/D2/D3 not connected in 1-bit mode */

/* ================================================================
 *  Test Parameters
 * ================================================================ */
#define SD_MOUNT_POINT      "/sdcard"
#define TEST_FILE           "/sdcard/speed_test.bin"
#define TEST_FILE_SIZE      (4 * 1024 * 1024)    /* 4 MB */
#define RAW_SECTOR_COUNT    8192                  /* 8192 × 512 = 4 MB */
#define RAW_START_SECTOR    0                     /* WARNING: overwrites data! */
                                                  /* Change to a safe offset on real cards */

/* Block sizes for filesystem benchmark */
static const size_t fs_block_sizes[] = {
    4096,       /* 4 KB */
    8192,       /* 8 KB */
    16384,      /* 16 KB */
    32768,      /* 32 KB */
    65536,      /* 64 KB */
    131072,     /* 128 KB */
};
#define NUM_FS_BLOCK_SIZES  (sizeof(fs_block_sizes) / sizeof(fs_block_sizes[0]))

/* Internal I/O buffer for raw sector tests (DMA-capable, word-aligned) */
static uint8_t  *s_raw_buf = NULL;
static size_t    s_raw_buf_size = 0;

/* Current card handle */
static sdmmc_card_t *s_card = NULL;

/* ================================================================
 *  Helpers
 * ================================================================ */

static void fill_pattern(uint8_t *buf, size_t len, uint8_t seed)
{
    for (size_t i = 0; i < len; i++) {
        buf[i] = (uint8_t)(i * 37 + seed);
    }
}

static const char *freq_str(int khz)
{
    if (khz >= 40000) return "40 MHz";
    if (khz >= 20000) return "20 MHz";
    static char s[16];
    snprintf(s, sizeof(s), "%d MHz", khz / 1000);
    return s;
}

/* ================================================================
 *  Mount / Unmount
 * ================================================================ */

static esp_err_t mount_sd(int freq_khz)
{
    ESP_LOGI(TAG, "----------------------------------------");
    ESP_LOGI(TAG, "Mounting SD card @ %s, 1-bit SDMMC mode...", freq_str(freq_khz));

    esp_vfs_fat_sdmmc_mount_config_t mount_cfg = {
        .format_if_mount_failed = false,
        .max_files             = 5,
        .allocation_unit_size  = 16 * 1024,
    };

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.max_freq_khz = freq_khz;
    /* 1-bit only — clear 4/8-bit and DDR flags for simpler init */
    host.flags &= ~(SDMMC_HOST_FLAG_4BIT | SDMMC_HOST_FLAG_8BIT | SDMMC_HOST_FLAG_DDR);
    host.flags |= SDMMC_HOST_FLAG_1BIT;
    host.command_timeout_ms = 5000;  /* generous timeout */

    sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
    slot.clk     = SDMMC_CLK_GPIO;
    slot.cmd     = SDMMC_CMD_GPIO;
    slot.d0      = SDMMC_D0_GPIO;
    slot.d1      = GPIO_NUM_NC;   /* not connected in 1-bit mode */
    slot.d2      = GPIO_NUM_NC;
    slot.d3      = GPIO_NUM_NC;
    slot.gpio_cd = GPIO_NUM_NC;   /* no card-detect pin */
    slot.gpio_wp = GPIO_NUM_NC;   /* no write-protect pin */
    slot.width   = 1;             /* explicit 1-bit */
    slot.flags  |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;  /* enable internal pull-ups */

    esp_err_t ret = esp_vfs_fat_sdmmc_mount(SD_MOUNT_POINT, &host, &slot,
                                             &mount_cfg, &s_card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Mount failed: %s (0x%x)", esp_err_to_name(ret), ret);
        ESP_LOGE(TAG, "Check: 1) SD card inserted?  2) External 10k pull-up");
        ESP_LOGE(TAG, "  on CMD (GPIO%d) and D0 (GPIO%d) to 3.3V?", SDMMC_CMD_GPIO, SDMMC_D0_GPIO);
        ESP_LOGE(TAG, "  3) GPIOs not used by another peripheral?");
        s_card = NULL;
        return ret;
    }

    sdmmc_card_print_info(stdout, s_card);

    /* Log actual negotiated bus width and frequency */
    ESP_LOGI(TAG, "Negotiated bus width: %d bit(s)",
             s_card->log_bus_width);
    ESP_LOGI(TAG, "Card is %s", s_card->is_ddr ? "DDR" : "SDR");
    ESP_LOGI(TAG, "Mounted OK @ %s", freq_str(freq_khz));

    return ESP_OK;
}

static void unmount_sd(void)
{
    if (s_card) {
        esp_vfs_fat_sdcard_unmount(SD_MOUNT_POINT, s_card);
        s_card = NULL;
        ESP_LOGI(TAG, "SD card unmounted");
    }
}

/* ================================================================
 *  Filesystem Write Test
 * ================================================================ */

static void test_fs_write(size_t total_size)
{
    ESP_LOGI(TAG, "--- FS WRITE ---");

    for (int i = 0; i < NUM_FS_BLOCK_SIZES; i++) {
        size_t blk = fs_block_sizes[i];
        size_t nblk = total_size / blk;

        uint8_t *buf = malloc(blk);
        if (!buf) {
            ESP_LOGE(TAG, "OOM for %u B write buffer", (unsigned)blk);
            continue;
        }
        fill_pattern(buf, blk, (uint8_t)i);

        FILE *fp = fopen(TEST_FILE, "wb");
        if (!fp) {
            ESP_LOGE(TAG, "Cannot create %s", TEST_FILE);
            free(buf);
            return;
        }
        /* Use 32 KB stdio buffer to reduce fwrite overhead */
        setvbuf(fp, NULL, _IOFBF, 32768);

        int64_t t0 = esp_timer_get_time();

        size_t written_total = 0;
        for (size_t b = 0; b < nblk; b++) {
            written_total += fwrite(buf, 1, blk, fp);
        }
        fflush(fp);
        int64_t t1 = esp_timer_get_time();
        fclose(fp);
        free(buf);
        unlink(TEST_FILE);

        float sec  = (t1 - t0) / 1000000.0f;
        float kbps = (written_total / 1024.0f) / sec;

        ESP_LOGI(TAG, "  WRITE  block=%5u B  %6u KB  %7.2f s  %9.1f KB/s",
                 (unsigned)blk, (unsigned)(written_total / 1024),
                 sec, kbps);
    }
}

/* ================================================================
 *  Filesystem Read Test
 * ================================================================ */

static void test_fs_read(size_t total_size)
{
    ESP_LOGI(TAG, "--- FS READ ---");

    for (int i = 0; i < NUM_FS_BLOCK_SIZES; i++) {
        size_t blk = fs_block_sizes[i];
        size_t nblk = total_size / blk;

        uint8_t *buf = malloc(blk);
        if (!buf) {
            ESP_LOGE(TAG, "OOM for %u B read buffer", (unsigned)blk);
            continue;
        }

        /* Pre-create the test file */
        FILE *fp = fopen(TEST_FILE, "wb");
        if (!fp) {
            ESP_LOGE(TAG, "Cannot create %s", TEST_FILE);
            free(buf);
            return;
        }
        setvbuf(fp, NULL, _IOFBF, 32768);
        fill_pattern(buf, blk, 0xCC);
        for (size_t b = 0; b < nblk; b++) fwrite(buf, 1, blk, fp);
        fclose(fp);

        /* Now read it back */
        fp = fopen(TEST_FILE, "rb");
        if (!fp) {
            ESP_LOGE(TAG, "Cannot open %s for reading", TEST_FILE);
            free(buf);
            unlink(TEST_FILE);
            return;
        }
        setvbuf(fp, NULL, _IOFBF, 32768);

        int64_t t0 = esp_timer_get_time();

        size_t read_total = 0;
        for (size_t b = 0; b < nblk; b++) {
            read_total += fread(buf, 1, blk, fp);
        }
        int64_t t1 = esp_timer_get_time();
        fclose(fp);
        free(buf);
        unlink(TEST_FILE);

        float sec  = (t1 - t0) / 1000000.0f;
        float kbps = (read_total / 1024.0f) / sec;

        ESP_LOGI(TAG, "  READ   block=%5u B  %6u KB  %7.2f s  %9.1f KB/s",
                 (unsigned)blk, (unsigned)(read_total / 1024),
                 sec, kbps);
    }
}

/* ================================================================
 *  Raw Sector Write Test  (bypasses FAT/VFS)
 * ================================================================ */

static void test_raw_write(void)
{
    if (!s_card || !s_raw_buf) return;

    size_t sector_size = s_card->csd.sector_size;
    size_t total_bytes = RAW_SECTOR_COUNT * sector_size;
    size_t sectors_per_chunk = s_raw_buf_size / sector_size;

    if (sectors_per_chunk == 0) {
        ESP_LOGE(TAG, "Raw buffer too small for one sector");
        return;
    }

    ESP_LOGI(TAG, "--- RAW WRITE (%u sectors, %u KB, buffer=%u sectors) ---",
             (unsigned)RAW_SECTOR_COUNT, (unsigned)(total_bytes / 1024),
             (unsigned)sectors_per_chunk);

    fill_pattern(s_raw_buf, s_raw_buf_size, 0x5A);

    int64_t t0 = esp_timer_get_time();
    esp_err_t err = ESP_OK;

    for (size_t sec = RAW_START_SECTOR; sec < RAW_START_SECTOR + RAW_SECTOR_COUNT; ) {
        size_t chunk = RAW_SECTOR_COUNT - (sec - RAW_START_SECTOR);
        if (chunk > sectors_per_chunk) chunk = sectors_per_chunk;

        err = sdmmc_write_sectors(s_card, s_raw_buf, sec, chunk);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Raw write error at sector %u: %s", (unsigned)sec, esp_err_to_name(err));
            break;
        }
        sec += chunk;
    }

    int64_t t1 = esp_timer_get_time();

    if (err == ESP_OK) {
        float sec_elapsed = (t1 - t0) / 1000000.0f;
        float kbps = (total_bytes / 1024.0f) / sec_elapsed;
        ESP_LOGI(TAG, "  RAW WRITE  %u KB  %7.2f s  %9.1f KB/s  (%s)",
                 (unsigned)(total_bytes / 1024), sec_elapsed, kbps,
                 s_card->is_ddr ? "DDR" : "SDR");
    }
}

/* ================================================================
 *  Raw Sector Read Test  (bypasses FAT/VFS)
 * ================================================================ */

static void test_raw_read(void)
{
    if (!s_card || !s_raw_buf) return;

    size_t sector_size = s_card->csd.sector_size;
    size_t total_bytes = RAW_SECTOR_COUNT * sector_size;
    size_t sectors_per_chunk = s_raw_buf_size / sector_size;

    if (sectors_per_chunk == 0) {
        ESP_LOGE(TAG, "Raw buffer too small for one sector");
        return;
    }

    ESP_LOGI(TAG, "--- RAW READ (%u sectors, %u KB, buffer=%u sectors) ---",
             (unsigned)RAW_SECTOR_COUNT, (unsigned)(total_bytes / 1024),
             (unsigned)sectors_per_chunk);

    int64_t t0 = esp_timer_get_time();
    esp_err_t err = ESP_OK;

    for (size_t sec = RAW_START_SECTOR; sec < RAW_START_SECTOR + RAW_SECTOR_COUNT; ) {
        size_t chunk = RAW_SECTOR_COUNT - (sec - RAW_START_SECTOR);
        if (chunk > sectors_per_chunk) chunk = sectors_per_chunk;

        err = sdmmc_read_sectors(s_card, s_raw_buf, sec, chunk);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Raw read error at sector %u: %s", (unsigned)sec, esp_err_to_name(err));
            break;
        }
        sec += chunk;
    }

    int64_t t1 = esp_timer_get_time();

    if (err == ESP_OK) {
        float sec_elapsed = (t1 - t0) / 1000000.0f;
        float kbps = (total_bytes / 1024.0f) / sec_elapsed;
        ESP_LOGI(TAG, "  RAW READ   %u KB  %7.2f s  %9.1f KB/s  (%s)",
                 (unsigned)(total_bytes / 1024), sec_elapsed, kbps,
                 s_card->is_ddr ? "DDR" : "SDR");
    }
}

/* ================================================================
 *  Run all tests at a given frequency
 * ================================================================ */

static void run_all_tests(int freq_khz)
{
    if (mount_sd(freq_khz) != ESP_OK) {
        ESP_LOGW(TAG, "Skipping %s tests — mount failed", freq_str(freq_khz));
        return;
    }

    /* Filesystem benchmarks */
    test_fs_write(TEST_FILE_SIZE);
    test_fs_read(TEST_FILE_SIZE);

    /* Raw sector benchmarks */
    // test_raw_write();
    // test_raw_read();

    unmount_sd();
}

/* ================================================================
 *  Entry point
 * ================================================================ */

void app_main(void)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=================================================");
    ESP_LOGI(TAG, "  ESP32S3  SDMMC  1-bit  Speed Test");
    ESP_LOGI(TAG, "=================================================");
    ESP_LOGI(TAG, "Chip:      %s (%d cores)", CONFIG_IDF_TARGET,
             configNUMBER_OF_CORES);
    // ESP_LOGI(TAG, "Flash:     %u MB", spi_flash_get_chip_size() / (1024 * 1024));
    ESP_LOGI(TAG, "Free heap: %" PRIu32 " KB",
             (uint32_t)(esp_get_free_heap_size() / 1024));
    ESP_LOGI(TAG, "Test file: %u KB", TEST_FILE_SIZE / 1024);
    ESP_LOGI(TAG, "");

    /* Init NVS */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    /* Allocate raw I/O buffer — use DMA-capable memory */
    s_raw_buf_size = 65536;   /* 64 KB — good trade-off for multi-sector ops */
    s_raw_buf = (uint8_t *)heap_caps_malloc(s_raw_buf_size,
                                             MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    if (!s_raw_buf) {
        /* Fallback: try without DMA */
        s_raw_buf = (uint8_t *)malloc(s_raw_buf_size);
    }
    if (!s_raw_buf) {
        ESP_LOGE(TAG, "FATAL: cannot allocate raw I/O buffer");
        return;
    }
    // ESP_LOGI(TAG, "Raw I/O buffer: %u KB %s",
    //          (unsigned)(s_raw_buf_size / 1024),
    //          heap_caps_get_flags(s_raw_buf) & MALLOC_CAP_DMA ? "(DMA)" : "(no DMA)");
    // ESP_LOGI(TAG, "");

    /* ======== Test @ 40 MHz ======== */
    ESP_LOGI(TAG, "#################################################");
    ESP_LOGI(TAG, "###             TEST 1:  40 MHz               ###");
    ESP_LOGI(TAG, "#################################################");
    run_all_tests(40000);

    vTaskDelay(pdMS_TO_TICKS(2000));   /* let hardware settle */

    /* ======== Test @ 20 MHz ======== */
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "#################################################");
    ESP_LOGI(TAG, "###             TEST 2:  20 MHz               ###");
    ESP_LOGI(TAG, "#################################################");
    run_all_tests(20000);

    /* Cleanup */
    free(s_raw_buf);
    s_raw_buf = NULL;

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=================================================");
    ESP_LOGI(TAG, "  All tests complete.  Review the logs above.");
    ESP_LOGI(TAG, "=================================================");

    /* Keep the task alive so monitor output is readable */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
