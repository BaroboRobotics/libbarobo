# The ODE Library

cmake_minimum_required(VERSION 2.6)
#include_directories(../opende/sys/include)

add_library(imobotcomms
  imobotcomms.cpp
  )

target_link_libraries(imobotcomms bluetooth)
