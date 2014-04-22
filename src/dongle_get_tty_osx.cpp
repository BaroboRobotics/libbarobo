#include "logging.h"

#include <stdio.h>
#include <stdlib.h>

#include <unistd.h>

#include <CoreFoundation/CoreFoundation.h>
#include <IOKit/IOKitLib.h>
#include <IOKit/IOBSD.h>
#include <IOKit/usb/IOUSBLib.h>
#include <IOKit/IOReturn.h>

static CFTypeRef get_string_prop (io_object_t device, const char *prop);
static CFTypeRef get_string_prop_r (io_object_t device, const char *prop);

int Mobot_dongleGetTTY (char *buf, size_t len) {
  kern_return_t result;
  io_iterator_t it;

  result = IOServiceGetMatchingServices(kIOMasterPortDefault,
      IOServiceMatching(kIOUSBDeviceClassName), &it);

  if (kIOReturnSuccess != result) {
    fprintf(stderr, "(barobo) ERROR: IOServiceGetMatchingServices\n");
    abort();
  }

  /* Iterate through the serial ports */
  io_object_t device;
  CFTypeRef prod_cont, vend_cont;
  bool found_dongle = false;
  while (!found_dongle && (device = IOIteratorNext(it))) {
    vend_cont = get_string_prop(device, "USB Vendor Name");
    if (!vend_cont) {
      fprintf(stderr, "(barobo) ERROR: USB device has no vendor name\n");
      continue;
    }
    const char *usb_vendor_name
      = CFStringGetCStringPtr(vend_cont, kCFStringEncodingMacRoman);

    prod_cont = get_string_prop(device, "USB Product Name");
    if (!prod_cont) {
      fprintf(stderr, "(barobo) ERROR: USB device has no product name\n");
      continue;
    }
    const char *usb_product_name
      = CFStringGetCStringPtr(prod_cont, kCFStringEncodingMacRoman);

    int i;
    for (i = 0; i < NUM_BAROBO_USB_DONGLE_IDS; ++i) {
      if (!strcmp(usb_vendor_name, g_barobo_usb_dongle_ids[i].manufacturer) &&
          !strcmp(usb_product_name, g_barobo_usb_dongle_ids[i].product)) {
        /* Woohoo! */
        found_dongle = true;
        break;
      }
    }

    CFRelease(prod_cont);
    CFRelease(vend_cont);
  }

  IOObjectRelease(it);

  if (!found_dongle) {
    return -1;
  }

  /* Get the tty file path for this device */
  CFTypeRef tty_cont = get_string_prop_r(device, "IOCalloutDevice");
  if (!tty_cont) {
    fprintf(stderr, "(barobo) ERROR: dongle has no IOCalloutDevice\n");
    return -1;
  }

  /* Put the device path into buf */
  /* TODO figure out if CFStringGetLength() includes a null-terminator or not */
  CFIndex ttylen = CFStringGetLength(tty_cont);
  if (ttylen >= len) {
    fprintf(stderr, "(barobo) ERROR: buffer overflow in Mobot_dongleGetTTY()\n");
    return -1;
  }
  CFStringGetBytes(tty_cont, CFRangeMake(0, ttylen),
      kCFStringEncodingMacRoman, 0, false, buf, len, NULL);
  buf[ttylen] = 0;
  CFRelease(tty_cont);

  /* FIXME code duplication here with linux_dongle_get_tty.c */
  if (!access(buf, R_OK | W_OK)) {
    bInfo(stderr, "(barobo) INFO: dongle found at %s\n", buf);
    return 0;
  }

  /* access() must have failed */

  if (EACCES == errno) {
    fprintf(stderr, "(barobo) WARNING: dongle found at %s, but user does not have "
        "sufficient read/write permissions.\n", buf);
  }
  else {
    char errbuf[256];
    strerror_r(errno, errbuf, sizeof(errbuf));
    fprintf(stderr, "(barobo) WARNING: attempted to access %s: %s\n", buf, errbuf);
  }

  return -1;
}

static CFTypeRef get_string_prop (io_object_t device, const char *prop) {
  CFStringRef key = CFStringCreateWithCString(kCFAllocatorDefault,
      prop, kCFStringEncodingMacRoman);
  CFTypeRef ret = IORegistryEntryCreateCFProperty(device,
      key, kCFAllocatorDefault, 0);
  CFRelease(key);
  return ret;
}

static CFTypeRef get_string_prop_r (io_object_t device, const char *prop) {
  CFStringRef key = CFStringCreateWithCString(kCFAllocatorDefault,
      prop, kCFStringEncodingMacRoman);
  CFTypeRef ret = IORegistryEntrySearchCFProperty(device, kIOServicePlane,
      key, kCFAllocatorDefault, kIORegistryIterateRecursively);
  CFRelease(key);
  return ret;
}
