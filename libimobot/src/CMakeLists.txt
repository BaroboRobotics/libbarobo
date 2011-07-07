cmake_minimum_required(VERSION 2.6)
#include_directories(../opende/sys/include)
#link_directories(../libi2c)

add_subdirectory(libi2c)

include_directories(libi2c)

add_library(imobot
  imobot.cpp
  libi2c/Crc8.c
  libi2c/DumpMem.c
  libi2c/i2c-api.c
  )

target_link_libraries(imobot bluetooth)

