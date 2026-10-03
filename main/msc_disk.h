/*
 * msc_disk.h — USB MSC (Mass Storage Class) backend on raw SD/eMMC
 */

#pragma once

#include "esp_err.h"

/**
 * @brief Initialize the SD/eMMC disk (1-bit SDMMC, raw sector access).
 *
 * No FATFS is mounted by the device; the disk is exported as a raw
 * block device to the USB host, which mounts the filesystem itself.
 *
 * IMPORTANT: while the host has the disk mounted, the ESP32 must not
 * write to it through any other path (FATFS etc.), or the filesystem
 * will get corrupted.
 *
 * @return ESP_OK on success. On failure MSC reports "no medium".
 */
esp_err_t msc_disk_init(void);
