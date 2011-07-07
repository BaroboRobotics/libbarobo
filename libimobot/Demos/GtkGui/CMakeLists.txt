cmake_minimum_required (VERSION 2.6)
project (GtkiMobotController)

# Set the version number
set (GtkiMobotController_VERSION_MAJOR 0)
set (GtkiMobotController_VERSION_MINOR 1)

# Set up a configure file
configure_file (
  "${PROJECT_SOURCE_DIR}/config.h.in"
  "${PROJECT_BINARY_DIR}/config.h"
  )

# Add the binary directory to include file search path
include_directories("${PROJECT_BINARY_DIR}")

# Add CiMobotSim library subdirectory and stuff
include_directories("../../../libimobotcomms" "../../src" "../../src/libi2c")
link_directories("../../../libimobotcomms" "../../src" "../../src/libi2c" "./")
set (EXTRA_LIBS ${EXTRA_LIBS} imobotcomms imobot i2c bluetooth interface)

find_package(GTK2 2.10 REQUIRED gtk glade)

add_custom_target(interface.o ld -r -b binary -o interface.o interface/interface.glade)
add_custom_target(libinterface.a ar rcs libinterface.a interface.o DEPENDS interface.o)

if(GTK2_FOUND)
  include_directories(${GTK2_INCLUDE_DIRS})
  add_executable(GtkiMobotController GtkiMobotController.cpp menuHandlers.cpp connectHandlers.cpp movementFunctions.cpp movementHandlers.cpp gait.cpp)
  add_dependencies(GtkiMobotController libinterface.a)
  target_link_libraries(GtkiMobotController ${EXTRA_LIBS} ${GTK2_LIBRARIES})
  install(TARGETS GtkiMobotController
          RUNTIME DESTINATION bin)
  install(FILES GtkiMobotController.desktop DESTINATION /usr/share/applications)
  install(FILES images/iMobot.png DESTINATION /usr/share/pixmaps)
endif()
