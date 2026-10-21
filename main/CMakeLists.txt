# ============================================================================
#  Sediment Tray Controller  ·  FW v1.3.0
#  ESP-IDF component CMakeLists  ·  single-file firmware
# ============================================================================
idf_component_register(
    SRCS "sediment v4.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES
        esp_driver_uart
        esp_driver_ledc
        esp_driver_gpio
        esp_driver_usb_serial_jtag
        esp_system
        esp_timer
        esp_hw_support
        esp_rom
        freertos
        nvs_flash
        log
)

