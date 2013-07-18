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
link_directories(${MXML_BINARY_DIR})

add_library(baroboconfigfile STATIC 
    BaroboConfigFile.c)

target_link_libraries(baroboconfigfile mxml)
if(CMAKE_HOST_WIN32)
    add_custom_command(TARGET baroboconfigfile
        POST_BUILD
        COMMAND cp libbaroboconfigfile.a baroboconfigfile.lib)
endif()
