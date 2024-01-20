
set(AUTO_PARKING_SRCS
    geometry_parking_utils.cc
    geometry_parking_utils.h
    line_circle_connection.cc
    line_circle_connection.h
    reverse_verticle_parking.cc
    reverse_verticle_parking.h
    )

add_library(auto_parking ${AUTO_PARKING_SRCS})
target_link_libraries(auto_parking apollo_math)