#check if kconfig symbols are enable then preform build
if(CONFIG_ENABLE_TEST)
    # Add your source file directory
    zephyr_include_directories(.)

    # Add the source file you want to compile
    zephyr_library_sources(test.c)

endif()