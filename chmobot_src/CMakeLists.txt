# The ODE Library

cmake_minimum_required(VERSION 2.6)
#include_directories(../opende/sys/include)

project(LIBMOBOT_DL)

add_definitions(-DNONRELEASE)

#include_directories(${CMAKE_SOURCE_DIR}/../../BaroboConfigFile)
#link_directories(${CMAKE_SOURCE_DIR}/../../BaroboConfigFile)
if(NOT (EXISTS $ENV{CHHOME}))
  MESSAGE(FATAL_ERROR "Environment variable CHHOME does not refer to a valid directory.")
endif()
include_directories(${BAROBOCONFIGFILE_SOURCE_DIR} $ENV{CHHOME}/extern/include)
link_directories(${BAROBOCONFIGFILE_BINARY_DIR} $ENV{CHHOME}/extern/lib)

include_directories(${MXML_SOURCE_DIR})
link_directories(${MXML_BINARY_DIR})

include_directories(${LIBIMOBOTCOMMS_SOURCE_DIR})
link_directories(${LIBIMOBOTCOMMS_BINARY_DIR})

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
  target_link_libraries(chmobot baroboStatic Ws2_32 baroboconfigfile mxml chsdk ch)
elseif(CMAKE_HOST_APPLE)
  add_definitions(-DENABLE_BLUETOOTH -DDARWIN -D_REENTRANT -DNeedFunctionPrototypes)
	add_library(chmobot SHARED 
      chmobot_chdl.cpp
      chmoboti_chdl.cpp
      chmobotl_chdl.cpp
	  )
	target_link_libraries(chmobot baroboStatic bluetooth pthread rt baroboconfigfile mxml chsdk)
  set_target_properties(chmobot PROPERTIES SUFFIX .dl)
else()
  add_definitions(-DENABLE_BLUETOOTH -DLINUX -D_REENTRANT -DNeedFunctionPrototypes)
	add_library(chmobot SHARED 
      chmobot_chdl.cpp
      chmoboti_chdl.cpp
      chmobotl_chdl.cpp
	  )
	target_link_libraries(chmobot baroboStatic bluetooth pthread rt baroboconfigfile mxml chsdk)
  set_target_properties(chmobot PROPERTIES SUFFIX .dl)
endif(CMAKE_HOST_WIN32)

#install(TARGETS mobot LIBRARY DESTINATION lib)

