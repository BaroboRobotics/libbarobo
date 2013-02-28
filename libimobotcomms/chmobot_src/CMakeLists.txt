# The ODE Library

cmake_minimum_required(VERSION 2.6)
#include_directories(../opende/sys/include)

project(LIBMOBOT_DL)

add_definitions(-DNONRELEASE)

#include_directories(${CMAKE_SOURCE_DIR}/../../BaroboConfigFile)
#link_directories(${CMAKE_SOURCE_DIR}/../../BaroboConfigFile)
include_directories(${BAROBOCONFIGFILE_SOURCE_DIR} $ENV{CHHOME}/extern/include)
link_directories(${BAROBOCONFIGFILE_BINARY_DIR} $ENV{CHHOME}/extern/lib)

include_directories(${MXML_SOURCE_DIR})
link_directories(${MXML_BINARY_DIR})

if(CMAKE_HOST_WIN32)
  #include(FindPkgConfig)
  #pkg_check_modules(MXML REQUIRED mxml)
  add_definitions(
      -D_WIN32_WINNT=0x502 
      -DENABLE_BLUETOOTH 
      -DWINDOWS 
      -D_REENTRANT 
      -DSTRICT 
      -DNeedFunctionPrototypes)
	add_library( chmobot SHARED
      chmobot_chdl.cpp
      )
  set_target_properties(chmobot PROPERTIES PREFIX lib)
  set_target_properties(chmobot PROPERTIES SUFFIX .dl)
  target_link_libraries(chmobot mobotStatic Ws2_32 baroboconfigfile mxml chsdk ch)
elseif(CMAKE_HOST_APPLE)
  set(CMAKE_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/lib")
	add_library(mobot SHARED
      mobot.c
      mobot.cpp
      mobot_get_functions.c
      mobot_get_functions.cpp
      mobotgroup.cpp
      mobot_motion_functions.c
      mobot_motion_functions.cpp
      mobot_movement_functions.c
      mobot_movement_functions.cpp
      mobot_record_functions.c
      mobot_record_functions.cpp
      mobot_set_functions.c
      mobot_set_functions.cpp
      moboti.cpp
      moboti_get_functions.cpp
      moboti_motion_functions.cpp
      moboti_movement_functions.cpp
      moboti_record_functions.cpp
      moboti_set_functions.cpp
      mobotl.cpp
      mobotl_get_functions.cpp
      mobotl_motion_functions.cpp
      mobotl_movement_functions.cpp
      mobotl_record_functions.cpp
      mobotl_set_functions.cpp
	  )
	add_library(mobotStatic STATIC 
      mobot.c
      mobot.cpp
      mobot_get_functions.c
      mobot_get_functions.cpp
      mobotgroup.cpp
      mobot_motion_functions.c
      mobot_motion_functions.cpp
      mobot_movement_functions.c
      mobot_movement_functions.cpp
      mobot_record_functions.c
      mobot_record_functions.cpp
      mobot_set_functions.c
      mobot_set_functions.cpp
      moboti.cpp
      moboti_get_functions.cpp
      moboti_motion_functions.cpp
      moboti_movement_functions.cpp
      moboti_record_functions.cpp
      moboti_set_functions.cpp
      mobotl.cpp
      mobotl_get_functions.cpp
      mobotl_motion_functions.cpp
      mobotl_movement_functions.cpp
      mobotl_record_functions.cpp
      mobotl_set_functions.cpp
	  )
	target_link_libraries(mobot pthread baroboconfigfile)
  set_target_properties(mobot PROPERTIES
    INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib")
  install(TARGETS mobot
          LIBRARY DESTINATION lib)
  install(FILES mobot.h DESTINATION include)
  add_subdirectory(demos)
else()
  add_definitions(-DENABLE_BLUETOOTH -DLINUX -D_REENTRANT -DNeedFunctionPrototypes)
	add_library(chmobot SHARED 
      chmobot_chdl.cpp
      chmoboti_chdl.cpp
      chmobotl_chdl.cpp
	  )
	target_link_libraries(chmobot mobotStatic bluetooth pthread rt baroboconfigfile mxml chsdk)
  set_target_properties(chmobot PROPERTIES SUFFIX .dl)
endif(CMAKE_HOST_WIN32)

#install(TARGETS mobot LIBRARY DESTINATION lib)

