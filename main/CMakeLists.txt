idf_component_register(SRCS main.c sound.c imu.c network.c INCLUDE_DIRS .)

#idf_build_set_property(COMPILE_OPTIONS "-Wno-unused-variable" APPEND)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-function)
spiffs_create_partition_image(storage ../data FLASH_IN_PROJECT)


