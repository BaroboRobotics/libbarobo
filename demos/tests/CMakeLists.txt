cmake_minimum_required(VERSION 2.6)

include_directories( ${LIBIMOBOTCOMMS_SOURCE_DIR})

link_directories( ${LIBIMOBOTCOMMS_BINARY_DIR})

add_executable(standtest standtest.cpp)
add_executable(continuousmotiontest continuousmotiontest.cpp)
target_link_libraries(standtest baroboStatic)
target_link_libraries(continuousmotiontest baroboStatic)
