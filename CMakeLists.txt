# Barobo Robot Communications library
# Barobo, Inc.

cmake_minimum_required(VERSION 2.8.9)
project(LIBBAROBO C CXX)

add_subdirectory(../libsfp libsfp)

set(VERSION_MAJOR 0)
set(VERSION_MINOR 0)
set(VERSION_PATCH 0)

# Boilerplate stuff
string(TOLOWER ${PROJECT_NAME} project_name)
set(EXPORT_TARGET_SET ${project_name}-targets)

##############################################################################
# Headers

set(INCLUDE_DIR include)
install(DIRECTORY ${INCLUDE_DIR} DESTINATION .)
include_directories(${INCLUDE_DIR})

##############################################################################
# Setup

find_package(LIBSFP)
add_subdirectory(demos EXCLUDE_FROM_ALL)
add_subdirectory(BaroboConfigFile)

include_directories(${LIBSFP_INCLUDE_DIRS})
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
  src/dongle.c
  src/dongle_get_tty.c
  src/mobot.c
  src/mobot++.cpp
  src/linkbot++.cpp
  src/linkboti++.cpp
  src/linkbotl++.cpp
  src/mobot_get_functions.c
  src/mobot_get_functions++.cpp
  src/mobot_motion_functions.c
  src/mobot_motion_functions++.cpp
  src/mobot_movement_functions.c
  src/mobot_movement_functions++.cpp
  src/mobot_record_functions.c
  src/mobot_record_functions++.cpp
  src/mobot_set_functions.c
  src/mobot_set_functions++.cpp
  src/mobotgroup++.cpp
  src/mobotigroup++.cpp
  src/mobotlgroup++.cpp
  src/rgbhashtable.c)

if(UNIX)
  list(APPEND SOURCES src/popen3.c)
endif()

option(BUILD_SHARED_LIBS "build shared libraries" ON)
add_library(${TARGET} ${SOURCES} $<TARGET_OBJECTS:bcfObjects> $<TARGET_OBJECTS:mxmlObjects>)

target_link_libraries(${TARGET} ${LIBSFP_LIBRARIES})

set_target_properties(${TARGET} PROPERTIES
  VERSION ${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}
  SOVERSION ${VERSION_MAJOR})

install(TARGETS ${TARGET} EXPORT ${EXPORT_TARGET_SET}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

add_library(${TARGET}Static STATIC ${SOURCES} $<TARGET_OBJECTS:bcfObjects> $<TARGET_OBJECTS:mxmlObjects>)
##############################################################################
# Platform Specific Setup

if(CMAKE_HOST_WIN32)
  # Win32

  # If you're on Windows, you can set the following paths to override the auto
  # detection scripts for the Windows SDK and Visual Studio include paths.
  #set(WINSDK_INCLUDE_PATH "C:/Program Files/Microsoft SDKs/Windows/v6.0/Include")
  #set(MSVC_INCLUDE_PATH "C:/Program Files (x86)/Microsoft Visual Studio 9.0/VC/include")

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
    set_target_properties(${TARGET} PROPERTIES COMPILE_FLAGS "/GS- /QIfist /TP")
  else()
    message(STATUS "Compiling for MinGW detected")
    # include_directories() only seems to be able to generate -I and -isystem
    # gcc flags. That won't do--we need -idirafter to avoid picking up all of
    # the Windows SDK and MSVC headers, only the ones we need.
    #include_directories(AFTER SYSTEM "${WINSDK_INCLUDE_PATH}")
    #include_directories(AFTER SYSTEM "${MSVC_INCLUDE_PATH}")
    set_target_properties(${TARGET} PROPERTIES COMPILE_FLAGS
      "-idirafter \"${WINSDK_INCLUDE_PATH}\" -idirafter \"${MSVC_INCLUDE_PATH}\"")
  endif()

  target_link_libraries(${TARGET} Ws2_32 setupapi)
  if(CMAKE_GENERATOR MATCHES "MSYS")
    target_link_libraries(${TARGET} pthread)
  endif()
else()
  # Non-Win32

  if(DEFINED ENV{CHHOME})
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
    set_target_properties(${TARGET}Static PROPERTIES COMPILE_FLAGS "-fPIC")
  endif()
endif()

##############################################################################
# Configuration for dependent projects

set(INSTALL_CMAKE_DIR lib/cmake/${project_name})

include(CMakePackageConfigHelpers)

write_basic_package_version_file(
  "${PROJECT_BINARY_DIR}/${project_name}-config-version.cmake"
  VERSION ${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}
  COMPATIBILITY SameMajorVersion)

# This must happen after INCLUDE_DIR and TARGET are set. The config file will
# use these two variables.
# Configure once for projects which are part of the same build as libsfp.
configure_file(inbuildconfig.cmake.in
  "${PROJECT_BINARY_DIR}/${project_name}-config.cmake" @ONLY)

# Add ${PROJECT_BINARY_DIR} to the find_package config.cmake search path.
export(PACKAGE ${PROJECT_NAME})

# And configure again for the final installation directory.
configure_package_config_file(config.cmake.in
  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${project_name}-config.cmake"
  INSTALL_DESTINATION "${INSTALL_CMAKE_DIR}"
  PATH_VARS INCLUDE_DIR)

install(FILES
  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${project_name}-config.cmake"
  "${PROJECT_BINARY_DIR}/${project_name}-config-version.cmake"
  DESTINATION "${INSTALL_CMAKE_DIR}")

# Generate a target export cmake script for the final installation directory.
install(EXPORT ${EXPORT_TARGET_SET} DESTINATION "${INSTALL_CMAKE_DIR}")
