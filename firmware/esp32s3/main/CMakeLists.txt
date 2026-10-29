idf_component_register(SRCS "main.c" "settings.c" "led.c" "link.cpp" "storage.c" "sdcard.c" "tier.c" "extras.c" "ota.c" "slicer_api.cpp" "crash.c" "oled.cpp" "pro.cpp" "accel.cpp" "usb_accel.cpp" "web_pro.cpp" "panel.cpp" "hostadc.c"
                       INCLUDE_DIRS "."
                       REQUIRES host esp_adc esp_wifi esp_event esp_netif esp_http_server nvs_flash driver esp_timer
                                spi_flash esp_rom dns_server fatfs sdmmc esp_driver_sdspi esp_driver_spi vfs
                                esp_psram efuse app_update esp_app_format mbedtls espcoredump ktft_oled usb)

# The build time in the firmware's self-identification (sf_build_id): a file in the build folder that
# is touched before every build, so it is compiled every time (the ESP-IDF app description and other
# files keep the time they were last compiled, which can be days old after a partial rebuild).
set(SF_STAMP "${CMAKE_CURRENT_BINARY_DIR}/sf_build_stamp.c")
file(WRITE "${SF_STAMP}" "const char sf_build_time[] = __DATE__ \" \" __TIME__;\n")
target_sources(${COMPONENT_LIB} PRIVATE "${SF_STAMP}")
add_custom_target(sf_build_stamp_touch COMMAND ${CMAKE_COMMAND} -E touch "${SF_STAMP}" VERBATIM)
add_dependencies(${COMPONENT_LIB} sf_build_stamp_touch)
