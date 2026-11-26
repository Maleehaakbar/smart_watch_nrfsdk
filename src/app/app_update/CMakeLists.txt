set (APP_UPDATE_SRC
    ble_ota.c
    ble_gps_weather.c
    )

if(CONFIG_DFU_SUPPORT_ENABLE)
    list(APPEND APP_UPDATE_SRC
        usb_dfu.c
        app_update.c
    )
endif()

target_sources(app PRIVATE ${APP_UPDATE_SRC})

