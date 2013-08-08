get_filename_component(WINSDKDIR "[HKEY_LOCAL_MACHINE\\SOFTWARE\\Microsoft\\Microsoft SDKs\\Windows;CurrentInstallFolder]" ABSOLUTE CACHE)
get_filename_component(WINSDKVER "[HKEY_LOCAL_MACHINE\\SOFTWARE\\Microsoft\\Microsoft SDKs\\Windows;CurrentVersion]" NAME)

if(WINSDKDIR)
  message(STATUS "Found Windows SDK ${WINSDKVER} in ${WINSDKDIR}")
endif(WINSDKDIR)

unset(WINSDK_INCLUDE_PATH)
find_path(WINSDK_INCLUDE_PATH Windows.h ${WINSDKDIR}/Include)

if(NOT WINSDK_INCLUDE_PATH)
  message(FATAL_ERROR "Unable to deduce Windows SDK include directory!")
endif(NOT WINSDK_INCLUDE_PATH)
