/*
 * cdc_uart.c — USB CDC ACM debug console bridge
 *
 * For now: echoes everything the host sends back to the host, and
 * provides cdc_printf() so the application can print debug messages
 * over the composite device's CDC port.
 */

#include <stdarg.h>
#include <stdio.h>
#include "esp_log.h"
#include "tusb.h"
#include "cdc_uart.h"

static const char *TAG = "cdc_acm";

/* Invoked when the host sends data on the CDC interface */
void tud_cdc_rx_cb(uint8_t itf)
{
    uint8_t buf[64];

    while (tud_cdc_n_available(itf)) {
        uint32_t count = tud_cdc_n_read(itf, buf, sizeof(buf));
        (void)count;
        /* echo back */
        tud_cdc_n_write(itf, buf, count);
    }
    tud_cdc_n_write_flush(itf);
}

/* Invoked when the host changes DTR/RTS line state */
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
    (void)itf;
    ESP_LOGI(TAG, "Host port opened: dtr=%d rts=%d", dtr, rts);
}

int cdc_printf(const char *fmt, ...)
{
    if (!tud_cdc_n_connected(0)) {
        return -1;
    }

    char buf[128];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    if (len < 0) {
        return -1;
    }

    uint32_t written = tud_cdc_n_write(0, buf, (uint32_t)len);
    tud_cdc_n_write_flush(0);
    return (int)written;
}
