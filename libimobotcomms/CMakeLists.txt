# The ODE Library

cmake_minimum_required(VERSION 2.6)
#include_directories(../opende/sys/include)

project(LIBIMOBOTCOMMS)

add_library(imobotcomms SHARED
  imobotcomms.cpp
  )

#add_library(imobotcomms STATIC
#  imobotcomms.cpp
#  )

if(CMAKE_HOST_WIN32)
target_link_libraries(imobotcomms Ws2_32)
else(CMAKE_HOST_WIN32)
target_link_libraries(imobotcomms bluetooth)
endif(CMAKE_HOST_WIN32)

#install(TARGETS imobotcomms LIBRARY DESTINATION lib)

add_subdirectory(demos)
