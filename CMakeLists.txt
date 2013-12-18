# Barobo Robot Communications library
# Barobo, Inc.

cmake_minimum_required(VERSION 2.8.9)
project(LIBBAROBO C CXX)
set(TARGET barobo)

# If you're on Windows, you can set the following paths to override the auto
# detection scripts for the Windows SDK and Visual Studio include paths.
#set(WINSDK_INCLUDE_PATH "C:/Program Files/Microsoft SDKs/Windows/v6.0/Include")
#set(MSVC_INCLUDE_PATH "C:/Program Files (x86)/Microsoft Visual Studio 9.0/VC/include")

set(${PROJECT_NAME}_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
  CACHE INTERNAL "${PROJECT_NAME}: public include directories" FORCE)

include_directories(${${PROJECT_NAME}_INCLUDE_DIRS})

##############################################################################
# Platform Independent Setup

add_subdirectory(demos EXCLUDE_FROM_ALL)
add_subdirectory(BaroboConfigFile)

#include_directories(${CMAKE_INSTALL_PREFIX}/include)
#link_directories(${CMAKE_INSTALL_PREFIX}/lib)

find_path(SFP_INC serial_framing_protocol.h HINTS ${LIBSFP_INCLUDE_DIRS})
include_directories(${SFP_INC})

include_directories(${BAROBOCONFIGFILE_SOURCE_DIR})
#link_directories(${BAROBOCONFIGFILE_BINARY_DIR})

include_directories(${MXML_SOURCE_DIR})
link_directories(${MXML_BINARY_DIR})

add_definitions(-DNONRELEASE)
#add_definitions(-DVERBOSE -DDEBUG)

if(NOT DEFINED $ENV{NOBLUETOOTH})
  message("enabling bluetooth")
  add_definitions(-DENABLE_BLUETOOTH)
endif()

##############################################################################
# Targets

set(SRCS
      mobot.c
      mobot++.cpp
      mobot_get_functions.c
      mobot_get_functions++.cpp
      mobotgroup++.cpp
      mobotigroup++.cpp
      mobotlgroup++.cpp
      mobot_motion_functions.c
      mobot_motion_functions++.cpp
      mobot_movement_functions.c
      mobot_movement_functions++.cpp
      mobot_record_functions.c
      mobot_record_functions++.cpp
      mobot_set_functions.c
      mobot_set_functions++.cpp
      moboti++.cpp
      moboti_get_functions++.cpp
      moboti_motion_functions++.cpp
      moboti_movement_functions++.cpp
      moboti_record_functions++.cpp
      moboti_set_functions++.cpp
      mobotl++.cpp
      mobotl_get_functions++.cpp
      mobotl_motion_functions++.cpp
      mobotl_movement_functions++.cpp
      mobotl_record_functions++.cpp
      mobotl_set_functions++.cpp
      dongle.c
      dongle_get_tty.c
      )
    FILE(GLOB HEADERS include/*.h)
    #set(HEADERS mobot.h donglefwd.h ${BAROBOCONFIGFILE_SOURCE_DIR}/BaroboConfigFile.h ${MXML_SOURCE_DIR}/mxml.h)

add_library(${TARGET}-objs OBJECT ${SRCS})
add_library(${TARGET} SHARED $<TARGET_OBJECTS:${TARGET}-objs> $<TARGET_OBJECTS:bcfObjects> $<TARGET_OBJECTS:mxmlObjects>)

target_link_libraries(${TARGET} sfp)

install(TARGETS ${TARGET}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(FILES ${HEADERS} DESTINATION include)

##############################################################################
# Platform Specific Setup

if(CMAKE_HOST_WIN32)
  # Win32

  if(NOT WINSDK_INCLUDE_PATH)
    include("findwindowssdk.cmake")
  endif(NOT WINSDK_INCLUDE_PATH)
  if(NOT MSVC_INCLUDE_PATH)
    include("findmsvc.cmake")
  endif(NOT MSVC_INCLUDE_PATH)

  add_definitions(-D_WIN32_WINNT=0x502 -DWIN32_LEAN_AND_MEAN)
  if( 
      (CMAKE_GENERATOR MATCHES "Visual Studio") OR
      (CMAKE_GENERATOR MATCHES "NMake")
    )
    message(STATUS "Compiling for Visual Studio detected")
    set_target_properties(${TARGET}-objs PROPERTIES COMPILE_FLAGS "/GS- /QIfist /TP")
  else()
    message(STATUS "Compiling for MinGW detected")
    # include_directories() only seems to be able to generate -I and -isystem
    # gcc flags. That won't do--we need -idirafter to avoid picking up all of
    # the Windows SDK and MSVC headers, only the ones we need.
    set_target_properties(${TARGET}-objs PROPERTIES COMPILE_FLAGS
      "-idirafter \"${WINSDK_INCLUDE_PATH}\" -idirafter \"${MSVC_INCLUDE_PATH}\"")
  endif()

  target_link_libraries(${TARGET} Ws2_32 setupapi)
else(CMAKE_HOST_WIN32)
  # Non-Win32

  if(DEFINED $ENV{CHHOME})
    add_subdirectory(chmobot_src)
  else()
    message(WARNING "CHHOME environment variable not found -- CH support disabled")
  endif()

  if(CMAKE_HOST_APPLE)
    # OSX

    set(CMAKE_INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib")

    find_library(COREFOUNDATION_LIB CoreFoundation)
    find_library(IOKIT_LIB IOKit)
    target_link_libraries(${TARGET} pthread ${IOKIT_LIB} ${COREFOUNDATION_LIB})
  else()
    # Linux

    # hlh: See https://github.com/commontk/Log4Qt/blob/patched/CMakeLists.txt#L116
    # I had the same problem.
    if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
      set_target_properties(${TARGET}-objs PROPERTIES COMPILE_FLAGS "-fPIC")
    endif()

    target_link_libraries(${TARGET} bluetooth pthread rt)
  endif()
endif(CMAKE_HOST_WIN32)
