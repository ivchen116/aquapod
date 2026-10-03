# NOTE: main.c / test_sdmmc_emmc.c / test_camera_res.c each define their own
# app_main() and must be compiled one at a time (duplicate symbol otherwise).
idf_component_register(SRCS "usb_webcam_main.c" "msc_disk.c" "cdc_uart.c"
                    INCLUDE_DIRS "."
                    # espressif__tinyusb: msc_disk.c / cdc_uart.c implement the
                    # tud_msc_* / tud_cdc_* callbacks and include tusb.h directly.
                    REQUIRES freertos esp_http_server esp_wifi nvs_flash fatfs
                              sdmmc driver esp_timer esp_hw_support usb_device_uvc
                              espressif__tinyusb)