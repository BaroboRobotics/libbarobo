cmake_minimum_required(VERSION 2.6)

include_directories( ${LIBIMOBOTCOMMS_SOURCE_DIR})

link_directories( ${LIBIMOBOTCOMMS_BINARY_DIR})

add_executable(simple_cpp simple.cpp)
add_executable(smooth_record smooth_record.cpp)
add_executable(simple simple.c)
add_executable(copycat copycat.cpp)
target_link_libraries(simple_cpp mobot)
target_link_libraries(smooth_record mobot)
target_link_libraries(simple mobot)
target_link_libraries(copycat mobot)
