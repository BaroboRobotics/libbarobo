cmake_minimum_required(VERSION 2.6)

include_directories( ${LIBIMOBOTCOMMS_SOURCE_DIR})

link_directories( ${LIBIMOBOTCOMMS_BINARY_DIR})

add_executable(voltages voltages.c)
target_link_libraries(voltages mobot)
