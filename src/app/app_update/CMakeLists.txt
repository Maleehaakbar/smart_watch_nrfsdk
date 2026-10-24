set (APP_UPDATE_SRC
    app_update.c
    ble_ota.c
    usb_dfu.c
    )

if(CONFIG_DFU_SUPPORT_ENABLE)
target_sources(app PRIVATE ${APP_UPDATE_SRC})
endif()
