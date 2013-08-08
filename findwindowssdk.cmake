# Note that there might be some issues with 64-bit versus 32-bit process
# registry access. If this doesn't work, you might need to use
# HKEY_LOCAL_MACHINE\\SOFTWARE\\Wow6432Node\\Microsoft ... instead.

get_filename_component(WINSDKDIR "[HKEY_LOCAL_MACHINE\\SOFTWARE\\Microsoft\\Microsoft SDKs\\Windows;CurrentInstallFolder]" ABSOLUTE)
get_filename_component(WINSDKVER "[HKEY_LOCAL_MACHINE\\SOFTWARE\\Microsoft\\Microsoft SDKs\\Windows;CurrentVersion]" NAME)

if(EXISTS ${WINSDKDIR})
  message(STATUS "Found Windows SDK ${WINSDKVER} in ${WINSDKDIR}")
endif(EXISTS ${WINSDKDIR})

unset(WINSDK_INCLUDE_PATH)
find_path(WINSDK_INCLUDE_PATH Windows.h "${WINSDKDIR}/Include")

if(NOT WINSDK_INCLUDE_PATH)
  message(FATAL_ERROR "Unable to deduce Windows SDK include directory!")
endif(NOT WINSDK_INCLUDE_PATH)
