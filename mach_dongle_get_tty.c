#include <stdio.h>
#include <stdlib.h>

#include <CoreFoundation/CoreFoundation.h>
#include <IOKit/IOKitLib.h>
#include <IOKit/IOBSD.h>
#include <IOKit/usb/IOUSBLib.h>
#include <IOKit/IOReturn.h>

static CFTypeRef get_string_prop (io_object_t device, const char *prop);

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
      CFRelease(prod_cont);
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

    for (int i = 0; i < NUM_BAROBO_USB_DONGLE_IDS; ++i) {
      if (!strcmp(usb_vendor_name, g_barobo_usb_dongle_ids[i].manufacturer) &&
          !strcmp(usb_product_name, g_barobo_usb_dongle_ids[i].product)) {
        found_dongle = true;
        break;
      }
    }

    printf("found %s : %s\n", usb_vendor_name, usb_product_name);
    CFRelease(prod_cont);
    CFRelease(vend_cont);
  }

  if (found_dongle) {
    /* Get the tty file path for this device */
    CFStringRef key = CFStringCreateWithCString(kCFAllocatorDefault,
        "IOCalloutDevice", kCFStringEncodingMacRoman);
    CFTypeRef tty_cont = IORegistryEntrySearchCFProperty(device, kIOServicePlane, key,
        kCFAllocatorDefault, kIORegistryIterateRecursively);
    CFRelease(key);
    if (!tty_cont) {
      fprintf(stderr, "(barobo) ERROR: IORegistryEntrySearchCFProperty\n");
      abort();
    }
    const char *ttyname
      = CFStringGetCStringPtr(tty_cont, kCFStringEncodingMacRoman);
    printf("%s\n", ttyname);
    CFRelease(tty_cont); /* XXX does this invalidate ttyname? */
  }


  IOObjectRelease(it);
  return 0;
}

static CFTypeRef get_string_prop (io_object_t device, const char *prop) {
  CFStringRef key = CFStringCreateWithCString(kCFAllocatorDefault,
      prop, kCFStringEncodingMacRoman);
  CFTypeRef ret = IORegistryEntryCreateCFProperty(device,
      key, kCFAllocatorDefault, 0);
  CFRelease(key);
  return ret;
}

#if 0
int main () {
  char buf[256];
  Mobot_dongleGetTTY(buf, 256);
}
#endif
