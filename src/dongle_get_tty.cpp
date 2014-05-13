#include "mobot.h"
/* Welcome to the hack. */

typedef struct usb_dongle_id {
  const char *manufacturer;
  const char *product;
} usb_dongle_id;

/* List of valid Barobo dongle manufacturer and product strings. The platform-
 * specific Mobot_dongleGetTTY() functions should depend on this data for
 * finding the dongle. Update this list as necessary. */
static const usb_dongle_id g_barobo_usb_dongle_ids[] = {
  { "Barobo, Inc.", "Mobot USB-Serial Adapter" },
  { "Barobo, Inc.", "Linkbot USB-Serial Adapter" },
  { "Barobo, Inc.", "Barobo USB-Serial Adapter" }
};

/* For convenience. */
#define NUM_BAROBO_USB_DONGLE_IDS \
  (sizeof(g_barobo_usb_dongle_ids) / sizeof(g_barobo_usb_dongle_ids[0]))

#ifdef _WIN32
#include "dongle_get_tty_win32.cpp"
#elif defined(__linux__)
#include "dongle_get_tty_linux.cpp"
#elif defined(__APPLE__) && defined(__MACH__)
#include "dongle_get_tty_osx.cpp"
#else
#error No dongle_get_tty.c available for this platform.
#endif
