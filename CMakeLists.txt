# The ODE Library

cmake_minimum_required(VERSION 2.6)
#include_directories(../opende/sys/include)

project(LIBIMOBOTCOMMS)

add_definitions(-DNONRELEASE)

#include_directories(${CMAKE_SOURCE_DIR}/../../BaroboConfigFile)
#link_directories(${CMAKE_SOURCE_DIR}/../../BaroboConfigFile)
add_subdirectory(BaroboConfigFile)
include_directories(${BAROBOCONFIGFILE_SOURCE_DIR})
link_directories(${BAROBOCONFIGFILE_BINARY_DIR})

include_directories(${MXML_SOURCE_DIR})
link_directories(${MXML_BINARY_DIR})
if(CMAKE_HOST_WIN32)
  #include(FindPkgConfig)
  #pkg_check_modules(MXML REQUIRED mxml)
  add_definitions(-D_WIN32_WINNT=0x502 -DENABLE_BLUETOOTH)
	add_library(barobo SHARED
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
	  )
	add_library(baroboStatic STATIC
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
	  )
  if( 
      (CMAKE_GENERATOR MATCHES "Visual Studio") OR
      (CMAKE_GENERATOR MATCHES "NMake")
    )
    message(STATUS "Compiling for Visual Studio detected")
    set_target_properties(barobo PROPERTIES COMPILE_FLAGS "/GS- /QIfist /TP")
    set_target_properties(baroboStatic PROPERTIES COMPILE_FLAGS "/GS- /QIfist /TP")
    target_link_libraries(barobo Ws2_32 baroboconfigfile mxml)
    target_link_libraries(baroboStatic Ws2_32 baroboconfigfile mxml)
    add_subdirectory(demos)
  else()
    add_subdirectory(demos)
    add_definitions(-DMSYS)
    target_link_libraries(barobo 
        Ws2_32 
        baroboconfigfile 
        mxml )
    target_link_libraries(baroboStatic 
        Ws2_32 
        baroboconfigfile 
        mxml)
  endif()
  install(TARGETS barobo
          ARCHIVE DESTINATION lib)
  install(FILES mobot.h DESTINATION include)
elseif(CMAKE_HOST_APPLE)
  set(CMAKE_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/lib")
	add_library(barobo SHARED
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
	  )
	add_library(baroboStatic STATIC 
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
	  )
	target_link_libraries(barobo pthread baroboconfigfile)
  set_target_properties(barobo PROPERTIES
    INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib")
  install(TARGETS barobo
          LIBRARY DESTINATION lib)
  install(FILES mobot.h DESTINATION include)
  add_subdirectory(demos)
else(CMAKE_HOST_WIN32)
  add_subdirectory(chmobot_src)
  add_definitions(-DENABLE_BLUETOOTH)
	add_library(baroboStatic STATIC
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
	  )
	add_library(barobo SHARED 
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
	  )
  set_target_properties(baroboStatic PROPERTIES COMPILE_FLAGS "-fPIC")
	target_link_libraries(barobo bluetooth pthread rt baroboconfigfile mxml)
	target_link_libraries(baroboStatic bluetooth pthread rt baroboconfigfile mxml)
  install(TARGETS barobo baroboStatic
          LIBRARY DESTINATION lib
          ARCHIVE DESTINATION lib)
  install(FILES mobot.h DESTINATION include)
  add_subdirectory(demos)
endif(CMAKE_HOST_WIN32)

#install(TARGETS mobot LIBRARY DESTINATION lib)

