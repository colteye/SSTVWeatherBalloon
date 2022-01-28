idf_component_register(
    SRCS bmp280.c
    INCLUDE_DIRS .
    REQUIRES general sensor_i2c error_handling
)