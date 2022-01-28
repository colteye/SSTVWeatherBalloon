idf_component_register(
    SRCS main.c
    INCLUDE_DIRS .
    REQUIRES general sensor_i2c error_handling sstv mpu6050 bmp280 gps
)