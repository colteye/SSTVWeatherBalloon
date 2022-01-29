idf_component_register(
    SRCS "sstv.c"
    INCLUDE_DIRS .
    REQUIRES general camera error_handling radio_transmitter
)