# TODO factor this out into a FindMSVS.cmake module, allow specification of
# required version.

# This CMake script is meant to be included with the include() command. It
# queries the Windows registry for the location of the latest version of
# Microsoft Visual Studio, and communicates this to the calling script via
# two output variables.
#
# Output variables:
#   MSVS_VERSION        latest version of Microsoft Visual Studio found
#   MSVS_INSTALL_DIR    absolute path to Microsoft Visual Studio installation
#                       i.e., C:\Program Files\Visual Studio 10.0

# All the versions of Visual Studio that we're aware of. The ones marked
# tested are known to work with this script.
set(versions
  12.0  # 2013 (tested)
  11.0  # 2012
  10.0  # 2010 (tested)
  9.0   # 2008
  8.0   # 2005
  # There are earlier versions, but they are too ancient to care about.
  )

if(NOT MSVS_VERSION)
  unset(MSVS_VERSION)
  foreach(ver ${versions})
    # Check the 64-bit location.
    get_filename_component(MSVS_INSTALL_DIR "[HKEY_LOCAL_MACHINE\\SOFTWARE\\Wow6432Node\\Microsoft\\VisualStudio\\${ver}\\Setup\\VS;ProductDir]" ABSOLUTE)
    if(EXISTS ${MSVS_INSTALL_DIR} AND IS_DIRECTORY ${MSVS_INSTALL_DIR})
      set(MSVS_VERSION ${ver})
      break()
    endif()

    # Check the 32-bit location.
    get_filename_component(MSVS_INSTALL_DIR "[HKEY_LOCAL_MACHINE\\SOFTWARE\\Microsoft\\VisualStudio\\${ver}\\Setup\\VS;ProductDir]" ABSOLUTE)
    if(EXISTS ${MSVS_INSTALL_DIR} AND IS_DIRECTORY ${MSVS_INSTALL_DIR})
      set(MSVS_VERSION ${ver})
      break()
    endif()
  endforeach()

  if(MSVS_VERSION)
    message(STATUS "Found Visual Studio ${MSVS_VERSION} in ${MSVS_INSTALL_DIR}")
  else()
    message(STATUS "Failed to find Visual Studio")
  endif()
endif()
