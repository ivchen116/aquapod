/*
 * msc_disk.c — USB MSC backend on raw SD/eMMC access (1-bit SDMMC)
 *
 * Same wiring and host config as test_sdmmc_emmc.c:
 *   CLK: GPIO 39, CMD: GPIO 38, D0: GPIO 40
 *
 * IDF >= 5.4 SDMMC API: sdmmc_card_init() takes a caller-allocated
 * sdmmc_card_t instance (zero-initialized), not a pointer-to-pointer.
 */

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "tusb.h"
#include "msc_disk.h"

static const char *TAG = "msc_disk";

#define DISK_CLK_GPIO       39
#define DISK_CMD_GPIO       38
#define DISK_D0_GPIO        40
#define DISK_MAX_FREQ_KHZ   20000

static sdmmc_card_t s_card;             /* zero-initialized before init */
static bool         s_card_inited = false;

esp_err_t msc_disk_init(void)
{
    ESP_LOGI(TAG, "Initializing SDMMC 1-bit disk...");

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.max_freq_khz = DISK_MAX_FREQ_KHZ;
    /* 1-bit only — clear 4/8-bit and DDR flags for simpler init */
    host.flags &= ~(SDMMC_HOST_FLAG_4BIT | SDMMC_HOST_FLAG_8BIT | SDMMC_HOST_FLAG_DDR);
    host.flags |= SDMMC_HOST_FLAG_1BIT;
    host.command_timeout_ms = 5000;

    sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
    slot.clk     = DISK_CLK_GPIO;
    slot.cmd     = DISK_CMD_GPIO;
    slot.d0      = DISK_D0_GPIO;
    slot.d1      = GPIO_NUM_NC;
    slot.d2      = GPIO_NUM_NC;
    slot.d3      = GPIO_NUM_NC;
    slot.gpio_cd = GPIO_NUM_NC;
    slot.gpio_wp = GPIO_NUM_NC;
    slot.width   = 1;
    slot.flags  |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    esp_err_t ret = sdmmc_host_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "sdmmc_host_init failed: %s (0x%x)", esp_err_to_name(ret), ret);
        return ret;
    }

    ret = sdmmc_host_init_slot(host.slot, &slot);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "sdmmc_host_init_slot failed: %s (0x%x)", esp_err_to_name(ret), ret);
        return ret;
    }

    memset(&s_card, 0, sizeof(s_card));
    ret = sdmmc_card_init(&host, &s_card);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Disk init failed: %s (0x%x) — MSC will report no medium", esp_err_to_name(ret), ret);
        s_card_inited = false;
        return ret;
    }

    s_card_inited = true;
    sdmmc_card_print_info(stdout, &s_card);
    ESP_LOGI(TAG, "Disk ready: %u sectors x %u bytes", (unsigned)s_card.csd.capacity,
             (unsigned)s_card.csd.sector_size);
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/* TinyUSB MSC callbacks                                               */
/* ------------------------------------------------------------------ */

void tud_msc_inquiry_cb(uint8_t lun, uint8_t vendor_id[8], uint8_t product_id[16], uint8_t product_rev[4])
{
    (void)lun;
    const char vid[] = "Espressif";
    const char pid[] = "AquaPod Disk";
    const char rev[] = "1.0";

    memcpy(vendor_id, vid, strlen(vid));
    memcpy(product_id, pid, strlen(pid));
    memcpy(product_rev, rev, strlen(rev));
}

bool tud_msc_test_unit_ready_cb(uint8_t lun)
{
    (void)lun;
    return s_card_inited;
}

void tud_msc_capacity_cb(uint8_t lun, uint32_t *block_count, uint16_t *block_size)
{
    (void)lun;
    if (!s_card_inited) {
        *block_count = 0;
        *block_size  = 512;
        return;
    }
    *block_count = s_card.csd.capacity;
    *block_size  = (uint16_t)s_card.csd.sector_size;
}

bool tud_msc_start_stop_cb(uint8_t lun, uint8_t power_condition, bool start, bool load_eject)
{
    (void)lun;
    (void)power_condition;
    (void)start;
    (void)load_eject;
    return true;
}

bool tud_msc_prevent_allow_medium_removal_cb(uint8_t lun, uint8_t prohibit_removal, uint8_t control)
{
    (void)lun;
    (void)prohibit_removal;
    (void)control;
    return true;
}

bool tud_msc_is_writable_cb(uint8_t lun)
{
    (void)lun;
    return s_card_inited;
}

int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset, void *buffer, uint32_t bufsize)
{
    (void)lun;
    if (!s_card_inited || offset != 0) {
        return -1;
    }

    size_t sector_size = s_card.csd.sector_size;
    if (bufsize % sector_size != 0) {
        ESP_LOGW(TAG, "read10: bufsize %u not sector-aligned", (unsigned)bufsize);
        return -1;
    }

    esp_err_t ret = sdmmc_read_sectors(&s_card, buffer, lba, bufsize / sector_size);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "read10 failed at lba %u: %s", (unsigned)lba, esp_err_to_name(ret));
        return -1;
    }
    return (int32_t)bufsize;
}

int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset, uint8_t *buffer, uint32_t bufsize)
{
    (void)lun;
    if (!s_card_inited || offset != 0) {
        return -1;
    }

    size_t sector_size = s_card.csd.sector_size;
    if (bufsize % sector_size != 0) {
        ESP_LOGW(TAG, "write10: bufsize %u not sector-aligned", (unsigned)bufsize);
        return -1;
    }

    esp_err_t ret = sdmmc_write_sectors(&s_card, buffer, lba, bufsize / sector_size);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "write10 failed at lba %u: %s", (unsigned)lba, esp_err_to_name(ret));
        return -1;
    }
    return (int32_t)bufsize;
}

/* Invoked for SCSI commands not built into the espressif tinyusb fork
 * (vendor-specific commands). Unlike upstream TinyUSB, this callback is
 * declared non-weak in this fork, so the app must provide it. */
int32_t tud_msc_scsi_cb(uint8_t lun, uint8_t const scsi_cmd[16], void *buffer, uint16_t bufsize)
{
    (void)buffer;
    (void)bufsize;

    switch (scsi_cmd[0]) {
        default:
            /* Set Sense = Invalid Command Operation */
            tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x20, 0x00);
            /* negative means error -> tinyusb responds with failed status */
            return -1;
    }
}
