# Note that there might be some issues with 64-bit versus 32-bit process
# registry access. If this doesn't work, you might need to use
# HKEY_LOCAL_MACHINE\\SOFTWARE\\Wow6432Node\\Microsoft ... instead.

# This next bit ought to make you cringe. If you know a better way of doing
# this in CMake's gimpy script language, replace it.
get_filename_component(MSVCDIR "[HKEY_LOCAL_MACHINE\\SOFTWARE\\Microsoft\\VisualStudio\\SxS\\VC7;10.0]" ABSOLUTE)
set(MSVCVER "10.0")

if(NOT EXISTS ${MSVCDIR})
  get_filename_component(MSVCDIR "[HKEY_LOCAL_MACHINE\\SOFTWARE\\Microsoft\\VisualStudio\\SxS\\VC7;9.0]" ABSOLUTE)
  set(MSVCVER "9.0")
endif(NOT EXISTS ${MSVCDIR})

if(NOT EXISTS ${MSVCDIR})
  get_filename_component(MSVCDIR "[HKEY_LOCAL_MACHINE\\SOFTWARE\\Microsoft\\VisualStudio\\SxS\\VC7;8.0]" ABSOLUTE CACHE)
  set(MSVCVER "8.0")
endif(NOT EXISTS ${MSVCDIR})

if(EXISTS ${MSVCDIR})
  message(STATUS "Found Visual Studio ${MSVCVER} in ${MSVCDIR}")
endif(EXISTS ${MSVCDIR})

unset(MSVC_INCLUDE_PATH)
find_path(MSVC_INCLUDE_PATH sal.h "${MSVCDIR}/include")

if(NOT MSVC_INCLUDE_PATH)
	message(FATAL_ERROR "Unable to deduce Visual Studio include directory! (${MSVCDIR} ${MSVCVER})")
endif(NOT MSVC_INCLUDE_PATH)
