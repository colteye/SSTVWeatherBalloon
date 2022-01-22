idf_component_register(
    SRCS sstv.c camera.c
    INCLUDE_DIRS .
    REQUIRES esp32_camera error_handling
)