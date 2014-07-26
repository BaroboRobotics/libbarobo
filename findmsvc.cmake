# TODO factor this out into a FindMSVC.cmake module, allow specification of
# required version.

# This CMake script is meant to be included with the include() command. It
# queries the Windows registry for the location of the latest version of
# Microsoft Visual C++, and communicates this to the calling script via two
# output variables.
#
# Output variables:
#   MSVC_VERSION        latest version of Microsoft Visual C++ found
#   MSVC_INSTALL_DIR    absolute path to Microsoft Visual C++ installation
#                       i.e., C:\Program Files\Microsoft Visual Studio 10.0\VC

# All the versions of Visual C++ that we're aware of. The ones marked
# tested are known to work with this script.
set(versions
  12.0  # 2013 (tested)
  11.0  # 2012
  10.0  # 2010 (tested)
  9.0   # 2008
  8.0   # 2005
  # There are earlier versions, but they are too ancient to care about.
  )

if(NOT MSVC_VERSION)
  unset(MSVC_VERSION)
  foreach(ver ${versions})
    # Check the 64-bit location.
    get_filename_component(MSVC_INSTALL_DIR "[HKEY_LOCAL_MACHINE\\SOFTWARE\\Wow6432Node\\Microsoft\\VisualStudio\\${ver}\\Setup\\VC;ProductDir]" ABSOLUTE)
    if(EXISTS ${MSVC_INSTALL_DIR} AND IS_DIRECTORY ${MSVC_INSTALL_DIR})
      set(MSVC_VERSION ${ver})
      break()
    endif()

    # Check the 32-bit location.
    get_filename_component(MSVC_INSTALL_DIR "[HKEY_LOCAL_MACHINE\\SOFTWARE\\Microsoft\\VisualStudio\\${ver}\\Setup\\VC;ProductDir]" ABSOLUTE)
    if(EXISTS ${MSVC_INSTALL_DIR} AND IS_DIRECTORY ${MSVC_INSTALL_DIR})
      set(MSVC_VERSION ${ver})
      break()
    endif()
  endforeach()

  if(MSVC_VERSION)
    message(STATUS "Found Visual C++ ${MSVC_VERSION} in ${MSVC_INSTALL_DIR}")
  else()
    message(STATUS "Failed to find Visual C++")
  endif()
endif()
