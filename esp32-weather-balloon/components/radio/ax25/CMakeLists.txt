idf_component_register(
    SRCS "ax25.c"
    INCLUDE_DIRS .
    REQUIRES general error_handling radio_transmitter
)