idf_component_register(
    SRCS main.c
    INCLUDE_DIRS .
    REQUIRES sstv mpu6050 sensor_i2c error_handling
)