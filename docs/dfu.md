# DFU Over USB from application

1. nrf soc ->cpu -> serial data -> usb peripheral-> cdc acm ->PC
2. build application with sys build, Change the application to use CDC ACM for DFU.
3. Include Zephyr USB stack module.
4. Flash and update firmware


# External Flash Memory 

1. Nrf52840 internal flash : 1MB
2. EXternal flash : 8MB