idf_component_register(
    SRCS
        "main.c"
        "common/crc16.c"
        "device/board.c"
        "device/profile.c"
        "device/scale_decode.c"
        "machine/control_task.c"
        "machine/machine_core.c"
        "machine/machine_settings.c"
        "modbus/modbus_master.c"
        "modbus/modbus_task.c"
        "settings/settings_codec.c"
        "settings/settings_store.c"
        "status/status_bus.c"
        "ui/encoder_decode.c"
        "ui/ui_logic.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_uart nvs_flash esp_timer
)
