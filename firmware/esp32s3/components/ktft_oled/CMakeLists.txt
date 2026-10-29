# ktft_oled: little I2C OLED screens (SSD1306 / SSD1315 / SH1106) for ESP-IDF 5.x.
# Drop this folder into your project's components/ directory and add ktft_oled to REQUIRES.
idf_component_register(
    SRCS oled.cpp oled_fonts.cpp oled_i2c.cpp
    INCLUDE_DIRS include
    REQUIRES esp_driver_i2c
)
