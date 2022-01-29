idf_component_register(
    SRCS "main.c" "data_manager.c"
    INCLUDE_DIRS .
    REQUIRES general sensor_i2c error_handling mpu6050 bmp280 gps camera sstv radio_transmitter
)