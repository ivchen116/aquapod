if(${IDF_VERSION_MAJOR} LESS 6)
    set(usb_requires usb)
else()
    set(usb_requires esp_hw_support)
endif()

idf_component_register(SRCS usb_device_uvc.c
                    INCLUDE_DIRS "include"
                    # espressif__tinyusb listed explicitly so that tusb.h and the
                    # class callbacks (video/msc/cdc) propagate to dependents
                    # (e.g. the app implements tud_msc_* / tud_cdc_* callbacks).
                    REQUIRES ${usb_requires} esp_timer espressif__tinyusb)

idf_component_get_property(tusb_lib espressif__tinyusb COMPONENT_LIB)

target_include_directories(${tusb_lib} PUBLIC "${COMPONENT_DIR}/tusb")
target_sources(${tusb_lib} PUBLIC "${COMPONENT_DIR}/tusb/usb_descriptors.c")

include(package_manager)
cu_pkg_define_version(${CMAKE_CURRENT_LIST_DIR})
