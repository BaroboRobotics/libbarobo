# The ODE Library

cmake_minimum_required(VERSION 2.6)
#include_directories(../opende/sys/include)
link_directories(../libi2c)

add_library(barobo
  barobo.c
  )

target_link_libraries(barobo i2c bluetooth)

add_executable(test
  test.c
  )

add_executable(stresstest stresstest.c)

add_executable(i2ctest i2ctest.c)

target_link_libraries(test barobo)
target_link_libraries(stresstest barobo)
