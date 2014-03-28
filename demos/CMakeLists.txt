if(WIN32)
  include("../findwindowssdk.cmake")
  if(NOT WINSDK_VERSION)
    message(FATAL_ERROR "libbarobo requires the Windows SDK")
  endif()

  find_path(WINSDK_UM_INCLUDE_DIR Windows.h
    PATHS "${WINSDK_INSTALL_DIR}/Include"
          "${WINSDK_INSTALL_DIR}/Include/um"
    NO_DEFAULT_PATH)

  if(NOT WINSDK_UM_INCLUDE_DIR)
    message(FATAL_ERROR "Unable to find Windows SDK user mode headers")
  endif()

  #if(NOT MSVC_INCLUDE_PATH)
  #  include("../findmsvc.cmake")
  #endif(NOT MSVC_INCLUDE_PATH)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -idirafter \"${WINSDK_UM_INCLUDE_DIR}\"")# -idirafter \"${MSVC_INCLUDE_PATH}\"")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -idirafter \"${WINSDK_UM_INCLUDE_DIR}\"")# -idirafter \"${MSVC_INCLUDE_PATH}\"")
endif()

add_subdirectory(simple)
add_subdirectory(stand)
add_subdirectory(inchworm)
add_subdirectory(multipleModules)
add_subdirectory(testVoltages)
add_subdirectory(tests)
