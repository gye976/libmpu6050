set(SRCS
        mpu6050_core.c
        mpu6050_i2cdev.c
        mpu6050_iio.c
)

add_library(mpu6050 SHARED ${SRCS})

target_include_directories(mpu6050 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(mpu6050 PUBLIC -Wall -Wextra)
target_link_libraries(mpu6050 PRIVATE m iio pthread)

install(TARGETS mpu6050 DESTINATION lib)
install(FILES mpu6050.h mpu6050_i2cdev.h DESTINATION include)
