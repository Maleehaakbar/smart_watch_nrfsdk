set (APP_SOURCES
    magnetometer.c   #can include/exclude magnetometer.c, if qmc driver is enable. 
    rtc_time.c
    )  
    
 if(CONFIG_ENABLE_IMU)
    list(APPEND APP_SOURCES
        imu.c
        sensor_fusion.c  #include/exclude sensor fusion if imu is enable/disable
    )
 endif()


target_sources(app PRIVATE ${APP_SOURCES})
if(CONFIG_ENABLE_IMU)
    zephyr_include_directories(${PROJECT_SOURCE_DIR}/ext_libraries/fusion/Fusion) #PROJECT_SOURCE_DIR:source directory of the most recent project() call.
    zephyr_sources(${PROJECT_SOURCE_DIR}/ext_libraries/fusion/Fusion/FusionBias.c)
    zephyr_sources(${PROJECT_SOURCE_DIR}/ext_libraries/fusion/Fusion/FusionCompass.c)
    zephyr_sources(${PROJECT_SOURCE_DIR}/ext_libraries/fusion/Fusion/FusionAhrs.c)
endif()