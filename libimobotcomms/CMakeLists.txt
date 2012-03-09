# The ODE Library

cmake_minimum_required(VERSION 2.6)
#include_directories(../opende/sys/include)

project(LIBIMOBOTCOMMS)


if(CMAKE_HOST_WIN32)
	add_library(mobot STATIC
	  mobot.cpp
	  )
	target_link_libraries(mobot Ws2_32)
elseif(CMAKE_HOST_APPLE)
  set(CMAKE_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/lib")
	add_library(mobot SHARED
	  mobot.cpp
	  )
	target_link_libraries(mobot pthread)
else(CMAKE_HOST_WIN32)
	add_library(mobot SHARED
	  mobot.cpp
	  )
	target_link_libraries(mobot bluetooth pthread rt)
endif(CMAKE_HOST_WIN32)

#install(TARGETS mobot LIBRARY DESTINATION lib)

add_subdirectory(demos)
