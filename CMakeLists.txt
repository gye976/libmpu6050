cmake_minimum_required(VERSION 3.25)

project(
        mpu6050
        VERSION 0.1
        DESCRIPTION "Calculate Euler angles."
        LANGUAGES C
)

add_subdirectory(src)
add_subdirectory(example)
