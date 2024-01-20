
set(APOLLO_MATH_SRCS
    aabox2d.cc
    aabox2d.h
    box2d.cc
    box2d.h
    line_segment2d.cc
    line_segment2d.h
    math_utils.cc
    math_utils.h
    vec2d.cc
    vec2d.h
)
add_library(apollo_math ${APOLLO_MATH_SRCS})