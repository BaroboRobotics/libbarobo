# The ODE Library

cmake_minimum_required(VERSION 2.8.8)
#include_directories(../opende/sys/include)

# If you're on Windows, you can set the following paths to override the auto
# detection scripts for the Windows SDK and Visual Studio include paths.
#set(WINSDK_INCLUDE_PATH "C:/Program Files/Microsoft SDKs/Windows/v6.0/Include")
#set(MSVC_INCLUDE_PATH "C:/Program Files (x86)/Microsoft Visual Studio 9.0/VC/include")

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
      linkboti++.cpp
      linkbotl++.cpp
      dongle.c
      dongle_get_tty.c
      linkbot++.cpp
      #     libsfp/serial_framing_protocol.c
      #      libsfp/net_byte_order.c
      )

project(LIBIMOBOTCOMMS)

add_definitions(-DNONRELEASE)

#include_directories(${CMAKE_SOURCE_DIR}/../../BaroboConfigFile)
#link_directories(${CMAKE_SOURCE_DIR}/../../BaroboConfigFile)
add_subdirectory(BaroboConfigFile)
add_subdirectory(libsfp)
include_directories(${BAROBOCONFIGFILE_SOURCE_DIR} ${LIBSFP_SOURCE_DIR})
link_directories(${BAROBOCONFIGFILE_BINARY_DIR})

include_directories(${MXML_SOURCE_DIR})
link_directories(${MXML_BINARY_DIR})
if(NOT DEFINED ENV{NOBLUETOOTH})
  message("enabling bluetooth")
  add_definitions(-DENABLE_BLUETOOTH)
endif()

if(CMAKE_HOST_WIN32)
  #include(FindPkgConfig)
  #pkg_check_modules(MXML REQUIRED mxml)
  # hlh: FIXME there should be a more official CMake way of doing this... they
  # have a concept of packages, but that part of their web site was down all
  # day, so I don't know if it would help us. At any rate, we definitely need
  # to figure out if finding the Windows SDK and MSVC include paths is only
  # necessary for MinGW or for MSVC as well.
  if(NOT WINSDK_INCLUDE_PATH)
    include("findwindowssdk.cmake")
  endif(NOT WINSDK_INCLUDE_PATH)
  if(NOT MSVC_INCLUDE_PATH)
    include("findmsvc.cmake")
  endif(NOT MSVC_INCLUDE_PATH)

  add_definitions(-D_WIN32_WINNT=0x502 -DWIN32_LEAN_AND_MEAN -DENABLE_BLUETOOTH)
  add_library(barobo SHARED ${SRCS} $<TARGET_OBJECTS:sfpObjects>)
  add_library(baroboStatic STATIC ${SRCS} $<TARGET_OBJECTS:sfpObjects>)
  if( 
      (CMAKE_GENERATOR MATCHES "Visual Studio") OR
      (CMAKE_GENERATOR MATCHES "NMake")
    )
    message(STATUS "Compiling for Visual Studio detected")
    set_target_properties(barobo PROPERTIES COMPILE_FLAGS "/GS- /QIfist /TP")
    set_target_properties(baroboStatic PROPERTIES COMPILE_FLAGS "/GS- /QIfist /TP")
  else()
    message(STATUS "Compiling for MinGW detected")
    set_target_properties(baroboStatic PROPERTIES COMPILE_FLAGS "-static-libgcc")
    #add_definitions(-DMSYS)
    # include_directories() only seems to be able to generate -I and -isystem
    # gcc flags. That won't do--we need -idirafter to avoid picking up all of
    # the Windows SDK and MSVC headers, only the ones we need.
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -idirafter \"${WINSDK_INCLUDE_PATH}\" -idirafter \"${MSVC_INCLUDE_PATH}\"")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -idirafter \"${WINSDK_INCLUDE_PATH}\" -idirafter \"${MSVC_INCLUDE_PATH}\"")
    #include_directories(AFTER SYSTEM "${WINSDK_INCLUDE_PATH}" "${MSVC_INCLUDE_PATH}")
  endif()
  target_link_libraries(barobo Ws2_32 baroboconfigfile mxml setupapi)
  target_link_libraries(baroboStatic Ws2_32 baroboconfigfile mxml setupapi)
  add_subdirectory(demos EXCLUDE_FROM_ALL)
  install(TARGETS barobo
          ARCHIVE DESTINATION lib)
  install(FILES mobot.h DESTINATION include)
elseif(CMAKE_HOST_APPLE)
  #add_subdirectory(chmobot_src)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
  set(CMAKE_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/lib")
  add_library(barobo SHARED ${SRCS} $<TARGET_OBJECTS:sfpObjects>)
  add_library(baroboStatic STATIC ${SRCS} $<TARGET_OBJECTS:sfpObjects>)
  find_library(COREFOUNDATION_LIB CoreFoundation)
  find_library(IOKIT_LIB IOKit)
  target_link_libraries(barobo pthread baroboconfigfile ${IOKIT_LIB} ${COREFOUNDATION_LIB})
  target_link_libraries(baroboStatic pthread baroboconfigfile ${IOKIT_LIB} ${COREFOUNDATION_LIB})
  set_target_properties(barobo PROPERTIES
    INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib")
  install(TARGETS barobo
          LIBRARY DESTINATION lib)
  install(FILES mobot.h DESTINATION include)
  add_subdirectory(demos EXCLUDE_FROM_ALL)
else(CMAKE_HOST_WIN32) #Linux
  if(DEFINED ENV{CHHOME})
    add_subdirectory(chmobot_src)
  else()
    message(WARNING "CHHOME environment variable not found -- CH support disabled")
  endif()
  add_library(baroboStatic STATIC ${SRCS} $<TARGET_OBJECTS:sfpObjects>)
  add_library(barobo SHARED ${SRCS} $<TARGET_OBJECTS:sfpObjects>)
  set_target_properties(baroboStatic PROPERTIES COMPILE_FLAGS "-fPIC")
  target_link_libraries(barobo bluetooth pthread rt baroboconfigfile mxml)
  target_link_libraries(baroboStatic bluetooth pthread rt baroboconfigfile mxml)
#  install(TARGETS barobo 
#          LIBRARY DESTINATION lib
#          ARCHIVE DESTINATION lib)
#install(FILES mobot.h DESTINATION include)
  add_subdirectory(demos EXCLUDE_FROM_ALL)
endif(CMAKE_HOST_WIN32)

#install(TARGETS mobot LIBRARY DESTINATION lib)

