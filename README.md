# smart_watch_nrfsdk

# overview

This smart watch project is developed using nrf connect sdk. It displays time and magnetic direction on watch display.

# Hardware
1. nrf52840 /nrf52840dk / xiao_nrf54l15
2. oled display
3. QMC5883l magnetometer
4. MPU6050 IMU
5. DS3231 RTC

# Software
1. Integration of Sensor fusion library for IMU orientation and magnetic direction.
2. LVGL module to support RTC display
3. Write own driver of qmc5883l

# Future development
1. Memory and Power optimization
2. OTA updates over BLE
3. Provide support for different boards and SOCs
