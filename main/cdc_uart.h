/*
 * cdc_uart.h — USB CDC ACM debug console bridge
 */

#pragma once

/**
 * @brief Print a formatted string to the CDC ACM port (interface 0).
 *
 * No-op when the host has not opened the port (tud_cdc_n_connected == false).
 *
 * @return number of characters written, or -1 when not connected
 */
int cdc_printf(const char *fmt, ...);
