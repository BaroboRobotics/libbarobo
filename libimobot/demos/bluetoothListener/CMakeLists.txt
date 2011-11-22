cmake_minimum_required(VERSION 2.6)

message(WARNING ${LIBIMOBOT_SOURCE_DIR} ${LIBIMOBOT_BINARY_DIR})

include_directories( ${LIBIMOBOT_SOURCE_DIR}/src )

link_directories( ${LIBIMOBOT_BINARY_DIR}/src )

add_executable(bluetoothListener bluetoothListener.cpp)
target_link_libraries(bluetoothListener imobot)
