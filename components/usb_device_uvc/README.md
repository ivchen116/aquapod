[![Component Registry](https://components.espressif.com/components/espressif/usb_device_uvc/badge.svg)](https://components.espressif.com/components/espressif/usb_device_uvc)

## USB Device UVC Component

`usb_device_uvc` is a USB `UVC` device driver for ESP32-S2/ESP32-S3, which supports streaming JPEG frames to the USB Host. User can wrapper the Camera or any devices as a UVC standard device through the callback functions.

Features:

1. Support video stream through the UVC Stream interface
2. Support both isochronous and bulk mode
3. Support multiple resolutions and frame rates
4. Support trans two cam's picture

> Note: If `UVC_SUPPORT_TWO_CAM` is enabled in `Kconfig` and multiple camera switches are required, please set both `UVC_CAM1_XFER_MODE` and `UVC_CAM2_XFER_MODE` to Isochronous mode.

### Add component to your project

Please use the component manager command `add-dependency` to add the `usb_device_uvc` to your project's dependency, during the `CMake` step the component will be downloaded automatically

```
idf.py add-dependency "espressif/usb_device_uvc=*"
```

### Examples

* [USB WebCamera: Make ESP32-S3-EYE as a USB Camera Device](https://github.com/espressif/esp-iot-solution/tree/master/examples/usb/device/usb_webcam)

## Local modifications (aquapod-fw fork)

This directory is a local fork of upstream `espressif/usb_device_uvc` v1.3.1
(see `idf_component.yml` `repository_info.commit_sha` for the upstream commit).
It extends the component from a pure UVC device to a **UVC + MSC + CDC composite device**:

| File | Change |
|---|---|
| `tusb/tusb_config.h` | Added `CFG_TUD_MSC` / `CFG_TUD_CDC` class enables and endpoint buffer sizes |
| `tusb/usb_descriptors.h` | Added MSC/CDC endpoint numbers (`EPNUM_MSC_*`, `EPNUM_CDC_*`) and interface enum entries |
| `tusb/usb_descriptors.c` | Appended `TUD_MSC_DESCRIPTOR` and `TUD_CDC_DESCRIPTOR` blocks to the configuration descriptor |

The MSC/CDC class callbacks (`tud_msc_*` / `tud_cdc_*`) are implemented in the
application (`main/msc_disk.c`, `main/cdc_uart.c`), not in this component.

Endpoint budget note: the ESP32-S3 DWC2 controller provides 7/7 endpoints and
5/5 IN FIFOs. The composite layout uses IN 0x81 (UVC ISO) + 0x82 (MSC) +
0x83 (CDC data) + 0x84 (CDC notif), which fills all 5 IN FIFOs, so
`CONFIG_UVC_SUPPORT_TWO_CAM` is incompatible with the MSC+CDC extension.

To upgrade: `git diff` against the upstream commit recorded above and re-apply
the changes to these three files only.
