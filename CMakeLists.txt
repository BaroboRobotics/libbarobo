# Barobo Robot Communications library
# Barobo, Inc.

cmake_minimum_required(VERSION 2.8.9)
project(LIBBAROBO C CXX)

set(VERSION_MAJOR 0)
set(VERSION_MINOR 0)

##############################################################################
# Headers

# Install our public headers.
file(GLOB HEADERS include/*)
install(FILES ${HEADERS} DESTINATION include)

# Expose our public headers to dependent sibling subprojects in CMake.
set(${PROJECT_NAME}_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
  CACHE INTERNAL "${PROJECT_NAME}: public headers" FORCE)

include_directories(${${PROJECT_NAME}_INCLUDE_DIRS})

##############################################################################
# Setup

# If you're on Windows, you can set the following paths to override the auto
# detection scripts for the Windows SDK and Visual Studio include paths.
#set(WINSDK_INCLUDE_PATH "C:/Program Files/Microsoft SDKs/Windows/v6.0/Include")
#set(MSVC_INCLUDE_PATH "C:/Program Files (x86)/Microsoft Visual Studio 9.0/VC/include")

add_subdirectory(demos EXCLUDE_FROM_ALL)
add_subdirectory(BaroboConfigFile)

# Search for libsfp's public headers in the following locations, in order:
#   - $ENV{CMAKE_PREFIX_PATH}/include
#   - ${LIBSFP_INCLUDE_DIRS}
#   - system include paths (e.g., /usr/include)
# In other words, if we're building libbarobo individually, we'll find libsfp
# in the system include path. If we're building libbarobo as a subproject,
# we'll find libsfp in the libsfp sibling subproject. Both cases are
# overrideable by setting the CMAKE_PREFIX_PATH environment variable to the
# prefix to which libsfp is installed.
find_path(SFP_INC serial_framing_protocol.h HINTS ${LIBSFP_INCLUDE_DIRS})
include_directories(${SFP_INC})

# Find libsfp itself, if it isn't a CMake target.
if(TARGET sfp)
  set(SFP sfp)
else()
  find_library(SFP sfp)
endif()

include_directories(${BAROBOCONFIGFILE_SOURCE_DIR})
include_directories(${MXML_SOURCE_DIR})

#link_directories(${MXML_BINARY_DIR})

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
  src/dongle.c
  src/dongle_get_tty.c
  src/mobot.c
  src/mobot++.cpp
  src/moboti++.cpp
  src/mobotl++.cpp
  src/mobot_get_functions.c
  src/mobot_get_functions++.cpp
  src/moboti_get_functions++.cpp
  src/mobotl_get_functions++.cpp
  src/mobot_motion_functions.c
  src/mobot_motion_functions++.cpp
  src/moboti_motion_functions++.cpp
  src/mobotl_motion_functions++.cpp
  src/mobot_movement_functions.c
  src/mobot_movement_functions++.cpp
  src/moboti_movement_functions++.cpp
  src/mobotl_movement_functions++.cpp
  src/mobot_record_functions.c
  src/mobot_record_functions++.cpp
  src/moboti_record_functions++.cpp
  src/mobotl_record_functions++.cpp
  src/mobot_set_functions.c
  src/mobot_set_functions++.cpp
  src/moboti_set_functions++.cpp
  src/mobotl_set_functions++.cpp
  src/mobotgroup++.cpp
  src/mobotigroup++.cpp
  src/mobotlgroup++.cpp)

option(BUILD_SHARED_LIBS "build shared libraries" ON)
add_library(${TARGET} ${SOURCES} $<TARGET_OBJECTS:bcfObjects> $<TARGET_OBJECTS:mxmlObjects>)

set_target_properties(${TARGET} PROPERTIES
  VERSION ${VERSION_MAJOR}.${VERSION_MINOR} SOVERSION ${VERSION_MAJOR})

target_link_libraries(${TARGET} ${SFP})

install(TARGETS ${TARGET}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

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
else()
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
    target_link_libraries(${TARGET} bluetooth pthread rt)
  endif()
endif()
