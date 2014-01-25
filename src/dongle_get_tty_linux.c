#include "logging.h"
#include "popen3.h"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <ctype.h>
#include <unistd.h>

int Mobot_dongleGetTTY (char *buf, size_t len) {
  const char* sysfs = getenv("SYSFS_PATH");
  if (!sysfs) {
    sysfs = "/sys";
  }

  size_t i;
  int dongle_found = 0;
  for (i = 0; !dongle_found && i < NUM_BAROBO_USB_DONGLE_IDS; ++i) {
    char cmd[1024];

    snprintf(cmd, sizeof(cmd), "find -O3 %s/devices -type f -name manufacturer -print0"
      " | xargs -0 grep -lZ '^%s$'"
      " | xargs -0 dirname -z"
      " | xargs -0 -I{} find -O3 '{}' -maxdepth 1 -type f -name product -print0"
      " | xargs -0 grep -lZ '^%s$'"
      " | xargs -0 dirname -z"
      " | xargs -0 -I{} find -O3 '{}' -type l -name subsystem -lname '*/tty' -print0"
      " | xargs -0 dirname -z"
      " | xargs -0 -I{} grep DEVNAME '{}'/uevent"
      " | cut -d= -f2",
      sysfs, g_barobo_usb_dongle_ids[i].manufacturer, g_barobo_usb_dongle_ids[i].product);

    /* Using popen3() so that stderr is suppressed upon failure. */
    process_t cmd_proc = popen3(cmd);

    if (-1 == cmd_proc.pid) {
      char errbuf[256];
      strerror_r(errno, errbuf, sizeof(errbuf));
      fprintf(stderr, "(barobo) ERROR: unable to execute find dongle command: %s", errbuf);
      return -1;
    }

    char* line = NULL;
    size_t n = 0;
    long read;

    while (-1 != (read = getline(&line, &n, cmd_proc.out))) {
      line[read - 1] = '\0';  /* Overwrite newline */
      snprintf(buf, len, "/dev/%s", line);
      if (!access(buf, R_OK | W_OK)) {
        bInfo(stderr, "(barobo) INFO: dongle found at %s\n", buf);
        dongle_found = 1;
        break;
      }
      if (EACCES == errno) {
        fprintf(stderr, "(barobo) WARNING: dongle found at %s, but user does not have "
            "sufficient read/write permissions.\n", buf);
      }
      else {
        char errbuf[256];
        strerror_r(errno, errbuf, sizeof(errbuf));
        fprintf(stderr, "(barobo) WARNING: attempted to access %s: %s\n", buf, errbuf);
      }
    }

    free(line);
    if (-1 == pclose3(cmd_proc)) {
      char errbuf[256];
      strerror_r(errno, errbuf, sizeof(errbuf));
      fprintf(stderr, "(barobo) WARNING: error closing find dongle command: %s\n", errbuf);
      return -1;
    }
  }

  return dongle_found ? 0 : -1;
}
