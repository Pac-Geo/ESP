idf_component_register(
    SRCS "Frame_IDF.cpp"
    INCLUDE_DIRS "."
    REQUIRES
        arduino-esp32
        usb_host_msc
        mbedtls
        Adafruit_AMG88xx_Library
        Adafruit_GFX_Library
        Adafruit_SSD1306
        Adafruit_BusIO
)
