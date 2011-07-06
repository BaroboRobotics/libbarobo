cmake_minimum_required(VERSION 2.6)
#include_directories(../opende/sys/include)
#link_directories(../libi2c)

add_subdirectory(libi2c)

include_directories(libi2c)
link_directories(libi2c)

add_library(imobot
  imobot.cpp
  )

target_link_libraries(imobot i2c bluetooth)

