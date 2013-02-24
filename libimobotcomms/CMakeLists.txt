# The ODE Library

cmake_minimum_required(VERSION 2.6)
#include_directories(../opende/sys/include)

project(LIBIMOBOTCOMMS)

add_definitions(-DNONRELEASE)

include_directories(${CMAKE_SOURCE_DIR}/../../BaroboConfigFile)
link_directories(${CMAKE_SOURCE_DIR}/../../BaroboConfigFile)

if(CMAKE_HOST_WIN32)
  include(FindPkgConfig)
  pkg_check_modules(MXML REQUIRED mxml)
  include_directories(${MXML_INCLUDE_DIRS})
  link_directories(/usr/lib)
  add_definitions(-D_WIN32_WINNT=0x502 -DENABLE_BLUETOOTH)
	add_library(mobot STATIC
	  mobot.cpp
    mobot_get_functions.cpp
    mobot_motion_functions.cpp
    mobot_movement_functions.cpp
    mobot_record_functions.cpp
    mobot_set_functions.cpp
	  )
  if( 
      (CMAKE_GENERATOR MATCHES "Visual Studio") OR
      (CMAKE_GENERATOR MATCHES "NMake")
    )
    message(STATUS "Compiling for Visual Studio detected")
    set_target_properties(mobot PROPERTIES COMPILE_FLAGS "/GS- /QIfist")
    target_link_libraries(mobot Ws2_32 baroboconfigfile regex mxml thread)
  else()
    add_subdirectory(demos)
    target_link_libraries(mobot Ws2_32 baroboconfigfile regex mxml pthread)
  endif()
  install(TARGETS mobot
          ARCHIVE DESTINATION lib)
  install(FILES mobot.h DESTINATION include)
elseif(CMAKE_HOST_APPLE)
  set(CMAKE_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/lib")
	add_library(mobot SHARED
	  mobot.cpp
    mobot_get_functions.cpp
    mobot_motion_functions.cpp
    mobot_movement_functions.cpp
    mobot_record_functions.cpp
    mobot_set_functions.cpp
	  )
	target_link_libraries(mobot pthread baroboconfigfile)
  set_target_properties(mobot PROPERTIES
    INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib")
  install(TARGETS mobot
          LIBRARY DESTINATION lib)
  install(FILES mobot.h DESTINATION include)
  add_subdirectory(demos)
else(CMAKE_HOST_WIN32)
  add_definitions(-DENABLE_BLUETOOTH)
	add_library(mobotStatic STATIC
	  mobot.cpp
    mobot_get_functions.cpp
    mobot_motion_functions.cpp
    mobot_movement_functions.cpp
    mobot_record_functions.cpp
    mobot_set_functions.cpp
	  )
	add_library(mobot SHARED 
	  mobot.cpp
    mobot_get_functions.cpp
    mobot_motion_functions.cpp
    mobot_movement_functions.cpp
    mobot_record_functions.cpp
    mobot_set_functions.cpp
	  )
	target_link_libraries(mobot bluetooth pthread rt baroboconfigfile mxml)
  install(TARGETS mobot mobotStatic
          LIBRARY DESTINATION lib
          ARCHIVE DESTINATION lib)
  install(FILES mobot.h DESTINATION include)
  add_subdirectory(demos)
endif(CMAKE_HOST_WIN32)

#install(TARGETS mobot LIBRARY DESTINATION lib)

