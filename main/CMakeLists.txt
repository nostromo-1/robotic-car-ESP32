idf_component_register(SRCS main.c pcf8591.c sound.c imu.c INCLUDE_DIRS .)

#idf_build_set_property(COMPILE_OPTIONS "-Wno-unused-variable" APPEND)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-function)
spiffs_create_partition_image(storage ../sounds FLASH_IN_PROJECT)


