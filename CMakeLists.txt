cmake_minimum_required(VERSION 3.25)

project(
        mpu6050
        VERSION 0.1
        DESCRIPTION "Calculate Euler angles."
        LANGUAGES C
)

add_subdirectory(src)
add_subdirectory(example)

option(USE_FIFO "Use FIFO" OFF)

if(USE_FIFO)
    message(STATUS "FIFO enabled")
else()
    message(STATUS "FIFO disabled")
endif()
