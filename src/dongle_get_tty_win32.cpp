/* Compile like so:
 *
 * gcc --std=c99 -o get_dongle_tty get_dongle_tty.c \
 *         -idirafter /path/to/visual/studio/headers \
 *         -idirafter /path/to/windows/sdk/headers \
 *         -lsetupapi
 */

#include "win32_guids.h"
#include "win32_error.h"

#include <assert.h>
#include <stdio.h>
#include <net_byte_order.h>
#include <stdlib.h>
#include <ctype.h>

#include <windows.h>
#include <tchar.h>
//#include <sal.h>

/* One more SAL define is required for setupapi.h */
#define __deref_out_range(a,b)
#include <setupapi.h>

#if _MSC_VER
#define snprintf _snprintf
#endif

static void printPropUnicodeStringList (PBYTE buf, DWORD bufsize) {
    assert(!(bufsize & 1));

    wchar_t *s = (wchar_t *)buf;
    size_t len = bufsize / 2;

    wprintf(L"{");
    wprintf(L" %s", s);
    s += wcslen(s) + 1;
    len -= wcslen(s) + 1;
    while (len) {
        wprintf(L", %s", s);
        s += wcslen(s) + 1;
        len -= wcslen(s) + 1;
    }
    wprintf(L" }");
}

static void printPropStringList (PTCHAR s) {
    _tprintf(_T("%s"), s);
    s += _tcslen(s) + 1;
    while (_tcslen(s)) {
        _tprintf(_T("\n\t%s"), s);
        s += _tcslen(s) + 1;
    }
    _tprintf(_T("\n"));
}

/* Allocate and return a buffer with property data in it. The size of the
 * buffer is returned in the output parameter size, the type of the property
 * is returned in the output parameter type. The caller must free the buffer
 * that is returned. Returns NULL on error. */
static PBYTE getPropertyBuf (HDEVINFO devices, PSP_DEVINFO_DATA dev,
        DWORD key, DWORD *size, DWORD *type) {
    assert(size && type);

    *size = 0;
    BOOL b = SetupDiGetDeviceRegistryProperty(devices, dev, key, NULL, NULL, 0, size);
    assert(!b);

    DWORD err = GetLastError();
    if (ERROR_INSUFFICIENT_BUFFER != err) {
        win32_error(_T("SetupDiGetDeviceRegistryProperty"), err);
        return NULL;
    }

    PBYTE buf = (PBYTE)malloc(sizeof(BYTE) * *size);
    assert(buf);
    memset(buf, 0, sizeof(BYTE) * *size);

    if (!SetupDiGetDeviceRegistryProperty(devices, dev, key, type, buf, *size, NULL)) {
        win32_error(_T("SetupDiGetDeviceRegistryProperty"), err);
        return NULL;
    }

    return buf;
}

static void dumpProperty (HDEVINFO devices, PSP_DEVINFO_DATA dev, DWORD key,
        PTCHAR keyname) {
    DWORD size = 0;
    DWORD type = 0;
    char *buf = (char*)getPropertyBuf(devices, dev, key, &size, &type);
    if (!buf) {
        exit(1);
    }

    _tprintf(_T("%s: "), keyname);

    switch (type) {
        case REG_BINARY:
            _tprintf(_T("(binary data)\n"));
            break;
        case REG_DWORD: /* little-endian */
            _tprintf(_T("%d\n"), (PDWORD)buf);
            break;
        case REG_QWORD: /* little-endian */
            _tprintf(_T("%ld\n"), (uint64_t *)buf);
            break;
        case REG_DWORD_BIG_ENDIAN:
            _tprintf(_T("(dword big endian)\n"));
            break;
        case REG_EXPAND_SZ:
            _tprintf(_T("%s\n"), (PTCHAR)buf);
            break;
        case REG_LINK:
            _tprintf(_T("(link)\n"));
            break;
        case REG_MULTI_SZ:
            printPropStringList((PTCHAR)buf);
            break;
        case REG_NONE:
            _tprintf(_T("(none)\n"));
            break;
        case REG_RESOURCE_LIST:
            _tprintf(_T("(resource list)\n"));
            break;
        case REG_SZ:
            _tprintf(_T("%s\n"), (PTCHAR)buf);
            break;
        default:
            _tprintf(_T("(unrecognized registry type %d)\n"), type);
            break;
    }

    free(buf);
}

#if 0
static void dumpProperties (HDEVINFO devices, PSP_DEVINFO_DATA dev) {
    dumpProperty(devices, dev, SPDRP_FRIENDLYNAME, _T("Friendly Name"));
    dumpProperty(devices, dev, SPDRP_ADDRESS, _T("Address"));
    dumpProperty(devices, dev, SPDRP_BUSNUMBER, _T("Bus Number"));
    dumpProperty(devices, dev, SPDRP_BUSTYPEGUID, _T("Bus Type GUID"));
    dumpProperty(devices, dev, SPDRP_CAPABILITIES, _T("Capabilities"));
    dumpProperty(devices, dev, SPDRP_CHARACTERISTICS, _T("Characteristics"));
    dumpProperty(devices, dev, SPDRP_CLASS, _T("Class"));
    dumpProperty(devices, dev, SPDRP_CLASSGUID, _T("Class GUID"));
    dumpProperty(devices, dev, SPDRP_COMPATIBLEIDS, _T("Compatible IDs"));
    dumpProperty(devices, dev, SPDRP_CONFIGFLAGS, _T("Config Flags"));
    dumpProperty(devices, dev, SPDRP_DEVICE_POWER_DATA, _T("Device Power Data"));
    dumpProperty(devices, dev, SPDRP_DEVICEDESC, _T("Device Description"));
    dumpProperty(devices, dev, SPDRP_DEVTYPE, _T("Device Type"));
    dumpProperty(devices, dev, SPDRP_DRIVER, _T("Driver"));
    dumpProperty(devices, dev, SPDRP_ENUMERATOR_NAME, _T("Enumerator Name"));
    dumpProperty(devices, dev, SPDRP_EXCLUSIVE, _T("Exclusive"));
    dumpProperty(devices, dev, SPDRP_HARDWAREID, _T("Hardware ID"));
    dumpProperty(devices, dev, SPDRP_INSTALL_STATE, _T("Install State"));
    dumpProperty(devices, dev, SPDRP_LEGACYBUSTYPE, _T("Legacy Bus Type"));
    dumpProperty(devices, dev, SPDRP_LOCATION_INFORMATION, _T("Location Information"));
    dumpProperty(devices, dev, SPDRP_LOCATION_PATHS, _T("Location Paths"));
    dumpProperty(devices, dev, SPDRP_LOWERFILTERS, _T("Lower Filters"));
    dumpProperty(devices, dev, SPDRP_MFG, _T("Manufacturer"));
    dumpProperty(devices, dev, SPDRP_PHYSICAL_DEVICE_OBJECT_NAME, _T("Physical Device Object Name"));
    dumpProperty(devices, dev, SPDRP_REMOVAL_POLICY, _T("Removal Policy"));
    dumpProperty(devices, dev, SPDRP_REMOVAL_POLICY_HW_DEFAULT, _T("Hardware Default Removal Policy"));
    dumpProperty(devices, dev, SPDRP_REMOVAL_POLICY_OVERRIDE, _T("Override Removal Policy"));
    dumpProperty(devices, dev, SPDRP_SECURITY, _T("Security"));
    dumpProperty(devices, dev, SPDRP_SECURITY_SDS, _T("Security Descriptor"));
    dumpProperty(devices, dev, SPDRP_UI_NUMBER, _T("UI Number"));
    dumpProperty(devices, dev, SPDRP_UI_NUMBER_DESC_FORMAT, _T("UI Number Format"));
    dumpProperty(devices, dev, SPDRP_UPPERFILTERS, _T("Upper Filters"));
    printf("\n");
}
#endif

static int isDongle (HDEVINFO devices, PSP_DEVINFO_DATA dev) {
  char manufacturer[64];
  char product[64];

  DWORD size = 0;
  DWORD type = 0;

  TCHAR *m = (TCHAR *)getPropertyBuf(devices, dev, SPDRP_MFG, &size, &type);
  if (!m) {
    return 0;
  }
  assert(REG_SZ == type);

  TCHAR *p= (TCHAR *)getPropertyBuf(devices, dev, SPDRP_DEVICEDESC, &size, &type);
  if (!p) {
    return 0;
  }
  assert(REG_SZ == type);

#if defined(UNICODE) || defined(_UNICODE)
  size_t retm = wcstombs(manufacturer, m, sizeof(manufacturer));
  size_t retp = wcstombs(product, p, sizeof(product));
#else
  strncpy(manufacturer, m, sizeof(manufacturer));
  strncpy(product, p, sizeof(product));
#endif

  free(m);
  free(p);
  m = NULL;
  p = NULL;

#if defined(UNICODE) || defined(_UNICODE)
  if (sizeof(manufacturer) <= retm || sizeof(product) <= retp) {
    fprintf(stderr, "(barobo) WARNING: buffer overflow or failed Unicode conversion in isDongle()\n");
    return 0;
  }
#else
  if (sizeof(manufacturer) == strlen(manufacturer)
      || sizeof(product) == strlen(product)) {
    fprintf(stderr, "(barobo) WARNING: buffer overflow in isDongle()\n");
    return 0;
  }
#endif

  size_t i;
  for (i = 0; i < NUM_BAROBO_USB_DONGLE_IDS; ++i) {
    if (!strcmp(manufacturer, g_barobo_usb_dongle_ids[i].manufacturer)
        || !strcmp(product, g_barobo_usb_dongle_ids[i].product)) {
      return 1;
    }
  }

  return 0;
}

static int getCOMPort (HDEVINFO devices, PSP_DEVINFO_DATA dev, char *comport, size_t len) {
    DWORD size = 0;
    DWORD type = 0;
    TCHAR *name = (TCHAR *)getPropertyBuf(devices, dev, SPDRP_FRIENDLYNAME, &size, &type);
    if (!name) {
        return -1;
    }

    assert(REG_SZ == type);

    char *buf = NULL;
#if defined(UNICODE) || defined(_UNICODE)
    size_t ret = wcstombs(NULL, name, 0);
    if ((size_t)-1 == ret) {
        fprintf(stderr, "Error converting Unicode FRIENDLYNAME to ANSI string\n");
        exit(1);
    }
    buf = malloc(sizeof(char) * ret);
    assert(buf);
    ret = wcstombs(buf, name, ret);
    if ((size_t)-1 == ret) {
        fprintf(stderr, "Error converting Unicode FRIENDLYNAME to ANSI string\n");
        exit(1);
    }
    free(name);
#else
    buf = name;
#endif

    char *lparen = strrchr(buf, '(');
    if (!lparen) {
        fprintf(stderr, "Dongle hardware does not appear to have a COM port\n");
        free(buf);
        return -1;
    }
    ++lparen;

    char *rparen = strchr(lparen, ')');
    if (!rparen) {
        fprintf(stderr, "Dongle hardware does not appear to have a COM port\n");
        free(buf);
        return -1;
    }
    *rparen = '\0';

    /* Finally got what we wanted. */
    snprintf(comport, len, "\\\\.\\%s", lparen);

    free(buf);
    return 0;
}

/* Find an attached dongle device and return the COM port name via the output
 * parameter tty. tty is a user-supplied buffer of size len. Return the COM
 * port number, if anyone cares. On error, return -1. */
int Mobot_dongleGetTTY (char *tty, size_t len) {
    /* Get all USB devices that provide a serial or parallel port interface. */
    HDEVINFO devices = SetupDiGetClassDevs(
            &GUID_DEVCLASS_PORTS,
            "USB",
            NULL,
            DIGCF_PRESENT);

    if (INVALID_HANDLE_VALUE == devices) {
        win32_error(_T("SetupDiGetClassDevs"), GetLastError());
        exit(1);
    }

    /* Now iterate over each device in the COM port interface class. */
    SP_DEVINFO_DATA dev;
    dev.cbSize = sizeof(SP_DEVINFO_DATA);
    DWORD i = 0;
    BOOL b = SetupDiEnumDeviceInfo(devices, i, &dev);
    int ret = -1;
    while (b) {
        if (isDongle(devices, &dev)) {
            ret = getCOMPort(devices, &dev, tty, len);
            if (-1 == ret) {
                fprintf(stderr, "Found dongle, but could not get COM port\n");
                exit(1);
            }
            /* Found the dongle. */
            break;
        }

        /* And get the next device. */
        dev.cbSize = sizeof(SP_DEVINFO_DATA);
        b = SetupDiEnumDeviceInfo(devices, ++i, &dev);
    }
    DWORD err = GetLastError();
    if (ERROR_SUCCESS != err && ERROR_NO_MORE_ITEMS != err) {
        win32_error(_T("SetupDiEnumDeviceInfo"), GetLastError());
        exit(1);
    }

    /* Done with our COM port devices. */
    if (!SetupDiDestroyDeviceInfoList(devices)) {
        win32_error(_T("SetupDiDestroyDeviceInfoList"), GetLastError());
        exit(1);
    }

    return ret;
}
