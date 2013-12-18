#ifndef _WIN32_GUIDS_H_
#define _WIN32_GUIDS_H_

#ifndef _WIN32
#error guid_devclass_ports.h is a Windows-specific header file.
#endif

/* Make the compiler shut up the "declared extern" warning. */
#if __GNUC__ >= 3
#pragma GCC system_header
#endif

#ifdef __cplusplus
extern "C" {
#endif

#include <initguid.h>

/* This value comes from here:
 * http://msdn.microsoft.com/en-us/library/windows/hardware/ff553426(v=vs.85).aspx
 * It's basically just a way to identify all serial and parallel ports in a
 * machine.
 */
DEFINE_GUID(GUID_DEVCLASS_PORTS, \
  0x4d36e978, 0xe325, 0x11ce, 0xbf, 0xc1, 0x08, 0x00, 0x2b, 0xe1, 0x03, 0x18);

#ifdef __cplusplus
}
#endif

#endif
