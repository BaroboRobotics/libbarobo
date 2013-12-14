cmake_minimum_required(VERSION 2.8)

project (BAROBOCONFIGFILE)

#include (FindPkgConfig)
#pkg_check_modules(MXML REQUIRED mxml)

if(CMAKE_HOST_WIN32)
else(CMAKE_HOST_WIN32)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
endif(CMAKE_HOST_WIN32)

add_subdirectory(mxml-2.7)

#include_directories(${MXML_INCLUDE_DIRS})
include_directories(${MXML_SOURCE_DIR})
#link_directories(${MXML_LIBRARY_DIRS})
#link_directories(${MXML_BINARY_DIR})

if(CMAKE_HOST_APPLE)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
endif()

add_library(bcfObjects OBJECT BaroboConfigFile.c)
