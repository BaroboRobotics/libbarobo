cmake_minimum_required(VERSION 2.6)

include_directories(${LIBIMOBOT_SOURCE_DIR}/src)

link_directories(${LIBIMOBOT_BINARY_DIR}/src)
add_executable(stresstest stresstest.c)
target_link_libraries(stresstest imobot)
