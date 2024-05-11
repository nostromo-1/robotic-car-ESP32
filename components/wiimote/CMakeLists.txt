idf_component_register(SRCS "wiimote.c" "TinyWiimote.c"
                    INCLUDE_DIRS "include"
                    REQUIRES bt)
set_target_properties(${COMPONENT_LIB} PROPERTIES COMPILE_FLAGS -Wno-unused-variable)

