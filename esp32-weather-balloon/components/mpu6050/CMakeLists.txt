idf_component_register(
    SRCS mpu6050.c
    INCLUDE_DIRS .
    REQUIRES sensor_i2c error_handling
)