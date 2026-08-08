cmake_minimum_required(VERSION 3.16)

add_library(tools OBJECT 
    extended_kalman_filter.cpp
    img_tools.cpp
    logger.cpp
    math_tools.cpp
    exiter.cpp
    plotter.cpp
)
