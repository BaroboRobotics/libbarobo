# Barobo Robot Communications library
# Barobo, Inc.

cmake_minimum_required(VERSION 2.8.9)
project(LIBBAROBO C CXX)

set(VERSION_MAJOR 0)
set(VERSION_MINOR 1)
set(VERSION_PATCH 2)
set(VERSION ${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH})

##############################################################################
# Headers

set(PUBLIC_INCLUDE_DIR include)
include_directories(${PUBLIC_INCLUDE_DIR})

##############################################################################
# Setup

add_subdirectory(../libsfp libsfp)
add_subdirectory(../libstkcomms libstkcomms)

add_subdirectory(demos EXCLUDE_FROM_ALL)
add_subdirectory(BaroboConfigFile)

include_directories(${LIBSFP_INCLUDE_DIRS})
include_directories(${LIBSTKCOMMS_INCLUDE_DIRS})

include_directories(${BAROBOCONFIGFILE_SOURCE_DIR})
include_directories(${MXML_SOURCE_DIR})

add_definitions(-DNONRELEASE)
#add_definitions(-DVERBOSE -DDEBUG)

if(NOT DEFINED $ENV{NOBLUETOOTH})
  message("enabling bluetooth")
  add_definitions(-DENABLE_BLUETOOTH)
endif()

##############################################################################
# Targets

set(TARGET barobo)

set(SOURCES
  src/dongle.cpp
  src/dongle_get_tty.cpp
  src/eventqueue.cpp
  src/flash.cpp
  src/mobot.cpp
  src/mobot++.cpp
  src/linkbot++.cpp
  src/linkboti++.cpp
  src/linkbotl++.cpp
  src/mobot_get_functions.cpp
  src/mobot_get_functions++.cpp
  src/mobot_motion_functions.cpp
  src/mobot_motion_functions++.cpp
  src/mobot_movement_functions.cpp
  src/mobot_movement_functions++.cpp
  src/mobot_record_functions.cpp
  src/mobot_record_functions++.cpp
  src/mobot_set_functions.cpp
  src/mobot_set_functions++.cpp
  src/mobotgroup++.cpp
  src/mobotigroup++.cpp
  src/mobotlgroup++.cpp
  src/rgbhashtable.c)

if(UNIX)
  list(APPEND SOURCES src/popen3.c)
endif()

option(BAROBO_BUILD_CHMOBOT "build libbarobo for linking with Ch" OFF)

if(BAROBO_BUILD_CHMOBOT)
  set(DEF_BUILD_SHARED_LIBS OFF)
else()
  set(DEF_BUILD_SHARED_LIBS ON)
endif()

option(BUILD_SHARED_LIBS "build shared libraries" ${DEF_BUILD_SHARED_LIBS})
add_library(${TARGET} ${SOURCES} $<TARGET_OBJECTS:bcfObjects> $<TARGET_OBJECTS:mxmlObjects>)

target_link_libraries(${TARGET} ${LIBSFP_LIBRARIES} ${LIBSTKCOMMS_LIBRARIES})

set_target_properties(${TARGET} PROPERTIES
  VERSION ${VERSION}
  SOVERSION ${VERSION_MAJOR})

##############################################################################
# Platform Specific Setup

if(WIN32)
  include("findwindowssdk.cmake")
  if(NOT WINSDK_VERSION)
    message(FATAL_ERROR "libbarobo requires the Windows SDK")
  endif()

  # Prior to Windows SDK 8.0, there was only a single flat Include directory.
  # Thereafter, the headers were broken out into three separate directories:
  # winrt (Windows Store API), shared, and um (user mode). User mode is the
  # one which contains the classic headers we care about, like windows.h and
  # ws2bth.h. This command will figure out which one we're dealing with.
  find_path(WINSDK_UM_INCLUDE_DIR Windows.h
    PATHS "${WINSDK_INSTALL_DIR}/Include"
          "${WINSDK_INSTALL_DIR}/Include/um"
    NO_DEFAULT_PATH)

  if(NOT WINSDK_UM_INCLUDE_DIR)
    message(FATAL_ERROR "Unable to find Windows SDK user mode headers under ${WINSDK_INSTALL_DIR}")
  endif()

  add_definitions(-D_WIN32_WINNT=0x502 -DWIN32_LEAN_AND_MEAN)
  if( 
      (CMAKE_GENERATOR MATCHES "Visual Studio") OR
      (CMAKE_GENERATOR MATCHES "NMake")
    )
    message(STATUS "Compiling for Visual Studio detected")
    set_target_properties(${TARGET} PROPERTIES COMPILE_FLAGS "/GS- /QIfist /TP")
  else()
    message(STATUS "Compiling for MinGW detected")
    # include_directories() only seems to be able to generate -I and -isystem
    # gcc flags. That won't do--we need -idirafter to avoid picking up all of
    # the Windows SDK and MSVC headers, only the ones we need.
    set_target_properties(${TARGET} PROPERTIES COMPILE_FLAGS
      "-idirafter \"${WINSDK_UM_INCLUDE_DIR}\"")
  endif()

  target_link_libraries(${TARGET} ws2_32 setupapi)
  if(CMAKE_GENERATOR MATCHES "MSYS")
    target_link_libraries(${TARGET} pthread)
  endif()
else()
  # Non-Win32

  if(BAROBO_BUILD_CHMOBOT)
    if(NOT DEFINED ENV{CHHOME})
      message(FATAL_ERROR "BAROBO_BUILD_CHMOBOT set, but CHHOME envvar not found!")
    endif()

    add_subdirectory(chmobot_src)
  endif()

  if(CMAKE_HOST_APPLE)
    # OSX

    set(CMAKE_INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib")

    find_library(COREFOUNDATION_LIB CoreFoundation)
    find_library(IOKIT_LIB IOKit)
    target_link_libraries(${TARGET} pthread ${IOKIT_LIB} ${COREFOUNDATION_LIB})
  else()
    # Linux
    target_link_libraries(${TARGET} bluetooth pthread rt)
  endif()
endif()

##############################################################################
# Installation

if(UNIX)
  set(INSTALL_LIB_DIR lib)
  set(INSTALL_BIN_DIR bin)
else()
  set(INSTALL_LIB_DIR .)
  set(INSTALL_BIN_DIR .)
endif()

set(install_args)
if(NOT BUILD_SHARED_LIBS)
  list(APPEND install_args ARCHIVE DESTINATION ${INSTALL_LIB_DIR})
endif()

install(TARGETS ${TARGET}
  ${install_args}
  LIBRARY DESTINATION ${INSTALL_LIB_DIR}
  RUNTIME DESTINATION ${INSTALL_BIN_DIR})

# TODO figure out what to do with the headers on install?
#install(DIRECTORY ${PUBLIC_INCLUDE_DIR} DESTINATION .)

##############################################################################
# Configuration for dependent projects

get_filename_component(${PROJECT_NAME}_INCLUDE_DIRS ${PUBLIC_INCLUDE_DIR} ABSOLUTE CACHE)
set(${PROJECT_NAME}_LIBRARIES ${TARGET} CACHE INTERNAL "")
set(${PROJECT_NAME}_VERSION ${VERSION} CACHE INTERNAL "")
