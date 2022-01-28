idf_component_register(
    SRCS gps.c nmea_parser.c
    INCLUDE_DIRS .
    REQUIRES error_handling
)