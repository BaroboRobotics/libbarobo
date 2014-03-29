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

add_definitions(-DNONRELEASE)

add_library(bcfObjects OBJECT BaroboConfigFile.c)

if(WIN32)
  add_definitions(-DWIN32_LEAN_AND_MEAN -D_WIN32_WINNT=0x0502 -DWINVER=0x0502)

  include("../findwindowssdk.cmake")
  if(NOT WINSDK_VERSION)
    message(FATAL_ERROR "libbarobo requires the Windows SDK")
  endif()

  find_path(WINSDK_UM_INCLUDE_DIR Windows.h
    PATHS "${WINSDK_INSTALL_DIR}/Include"
          "${WINSDK_INSTALL_DIR}/Include/um"
    NO_DEFAULT_PATH)

  if(NOT WINSDK_UM_INCLUDE_DIR)
    message(FATAL_ERROR "Unable to find Windows SDK user mode headers")
  endif()

  set_target_properties(bcfObjects PROPERTIES COMPILE_FLAGS
    "-idirafter \"${WINSDK_UM_INCLUDE_DIR}\"")
endif()

