#ifndef _WIN32_ERROR_H_
#define _WIN32_ERROR_H_

#ifndef _WIN32
#error win32_error.h is a Windows-specific header file.
#endif

#include <assert.h>
#include <stdio.h>
#include <windows.h>
#include <tchar.h>

/* Kind of like perror(), but takes an error code as a parameter as well, from
 * GetLastError(). Also, the msg string must be passed already wrapped in the
 * _T() macro, like so:
 *
 * DWORD err = GetLastError();
 * win32_error(_T("Ruh-roh!"), err);
 */
static inline void win32_error (LPCTSTR msg, DWORD errcode) {
    LPVOID errorText;
    FormatMessage(FORMAT_MESSAGE_FROM_SYSTEM
            | FORMAT_MESSAGE_ALLOCATE_BUFFER
            | FORMAT_MESSAGE_IGNORE_INSERTS,
            NULL, errcode,
            MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
            (LPTSTR)&errorText,
            0, NULL);
    assert(errorText);
    _ftprintf(stderr, _T("%s: %s\n"), msg, errorText);
    LocalFree(errorText);
}

#endif
