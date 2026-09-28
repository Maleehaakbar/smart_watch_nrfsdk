#check if in kconfig  module is enable then preform build
if(CONFIG_ENABLE_QMC5883L_SENSOR)
    
    #take a directory name as module name
    zephyr_library()

    # Add the source file you want to compile
    zephyr_library_sources(qmc5883l.c)

    # Add your source file directory
    zephyr_include_directories(.)

    

   
endif()