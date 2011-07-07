# The ODE Library
cmake_minimum_required(VERSION 2.6)

add_library(i2c STATIC
Crc8.c
DumpMem.c
i2c-api.c
  )

add_executable(i2c-hello
i2c-hello.c
)

target_link_libraries(i2c-hello
i2c
)
