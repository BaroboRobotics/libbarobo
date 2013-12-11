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

    FILE* cmdstdout = popen(cmd, "r");

    if (!cmdstdout) {
      char errbuf[256];
      strerror_r(errno, errbuf, sizeof(errbuf));
      fprintf(stderr, "(barobo) ERROR: unable to execute find dongle command: %s", errbuf);
      return -1;
    }

    char* line = NULL;
    size_t n = 0;
    ssize_t read;

    while (-1 != (read = getline(&line, &n, cmdstdout))) {
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
    if (-1 == pclose(cmdstdout)) {
      char errbuf[256];
      strerror_r(errno, errbuf, sizeof(errbuf));
      fprintf(stderr, "(barobo) WARNING: error closing find dongle command: %s\n", errbuf);
      return -1;
    }
  }

  return dongle_found ? 0 : -1;
}

#if 0
static int util_replace_whitespace(const char *str, char *to, size_t len);
static int util_replace_chars(char *str, const char *white);

int Mobot_dongleGetTTY (char *buf, size_t len) {
  char manufacturer[64];
  char product[64];

  size_t i;
  for (i = 0; i < NUM_BAROBO_USB_DONGLE_IDS; ++i) {
    const char *m = g_barobo_usb_dongle_ids[i].manufacturer;
    const char *p = g_barobo_usb_dongle_ids[i].product;

    /* Mangle the manufacturer and product names the exact same way that udev
     * does. */
    util_replace_whitespace(m, manufacturer, sizeof(manufacturer)-1);
    util_replace_chars(manufacturer, NULL);

    util_replace_whitespace(p, product, sizeof(product)-1);
    util_replace_chars(product, NULL);

    if (len <= snprintf(buf, len, "/dev/serial/by-id/usb-%s_%s-if00", manufacturer, product)) {
      fprintf(stderr, "(barobo) ERROR: buffer overflow in Mobot_dongleGetTTY()\n");
      return -1;
    }

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
  }

  return -1;
}

//////////////////////////////////////////////////////////////////////////////

/* 
 * Everything that follows was stol^H^H^H^Hborrowed from:
 * http://cgit.freedesktop.org/systemd/systemd/tree/src/libudev/libudev-util.c
 * It is licensed under the GNU General Public License version 2.1 or later.
 * I wanted to make sure we were mangling the manufacturer and product names
 * of our USB devices in exactly the same way as udev does.
 */

/* count of characters used to encode one unicode char */
static int utf8_encoded_expected_len(const char *str)
{
        unsigned char c = (unsigned char)str[0];

        if (c < 0x80)
                return 1;
        if ((c & 0xe0) == 0xc0)
                return 2;
        if ((c & 0xf0) == 0xe0)
                return 3;
        if ((c & 0xf8) == 0xf0)
                return 4;
        if ((c & 0xfc) == 0xf8)
                return 5;
        if ((c & 0xfe) == 0xfc)
                return 6;
        return 0;
}

/* decode one unicode char */
static int utf8_encoded_to_unichar(const char *str)
{
        int unichar;
        int len;
        int i;

        len = utf8_encoded_expected_len(str);
        switch (len) {
        case 1:
                return (int)str[0];
        case 2:
                unichar = str[0] & 0x1f;
                break;
        case 3:
                unichar = (int)str[0] & 0x0f;
                break;
        case 4:
                unichar = (int)str[0] & 0x07;
                break;
        case 5:
                unichar = (int)str[0] & 0x03;
                break;
        case 6:
                unichar = (int)str[0] & 0x01;
                break;
        default:
                return -1;
        }

        for (i = 1; i < len; i++) {
                if (((int)str[i] & 0xc0) != 0x80)
                        return -1;
                unichar <<= 6;
                unichar |= (int)str[i] & 0x3f;
        }

        return unichar;
}

/* expected size used to encode one unicode char */
static int utf8_unichar_to_encoded_len(int unichar)
{
        if (unichar < 0x80)
                return 1;
        if (unichar < 0x800)
                return 2;
        if (unichar < 0x10000)
                return 3;
        if (unichar < 0x200000)
                return 4;
        if (unichar < 0x4000000)
                return 5;
        return 6;
}

/* check if unicode char has a valid numeric range */
static int utf8_unichar_valid_range(int unichar)
{
        if (unichar > 0x10ffff)
                return 0;
        if ((unichar & 0xfffff800) == 0xd800)
                return 0;
        if ((unichar > 0xfdcf) && (unichar < 0xfdf0))
                return 0;
        if ((unichar & 0xffff) == 0xffff)
                return 0;
        return 1;
}

/* validate one encoded unicode char and return its length */
static int utf8_encoded_valid_unichar(const char *str)
{
        int len;
        int unichar;
        int i;

        len = utf8_encoded_expected_len(str);
        if (len == 0)
                return -1;

        /* ascii is valid */
        if (len == 1)
                return 1;

        /* check if expected encoded chars are available */
        for (i = 0; i < len; i++)
                if ((str[i] & 0x80) != 0x80)
                        return -1;

        unichar = utf8_encoded_to_unichar(str);

        /* check if encoded length matches encoded value */
        if (utf8_unichar_to_encoded_len(unichar) != len)
                return -1;

        /* check if value has valid range */
        if (!utf8_unichar_valid_range(unichar))
                return -1;

        return len;
}

static int util_replace_whitespace(const char *str, char *to, size_t len)
{
        size_t i, j;

        /* strip trailing whitespace */
        len = strnlen(str, len);
        while (len && isspace(str[len-1]))
                len--;

        /* strip leading whitespace */
        i = 0;
        while (isspace(str[i]) && (i < len))
                i++;

        j = 0;
        while (i < len) {
                /* substitute multiple whitespace with a single '_' */
                if (isspace(str[i])) {
                        while (isspace(str[i]))
                                i++;
                        to[j++] = '_';
                }
                to[j++] = str[i++];
        }
        to[j] = '\0';
        return 0;
}

static int is_whitelisted(char c, const char *white)
{
        if ((c >= '0' && c <= '9') ||
            (c >= 'A' && c <= 'Z') ||
            (c >= 'a' && c <= 'z') ||
            strchr("#+-.:=@_", c) != NULL ||
            (white != NULL && strchr(white, c) != NULL))
                return 1;
        return 0;
}

/* allow chars in whitelist, plain ascii, hex-escaping and valid utf8 */
static int util_replace_chars(char *str, const char *white)
{
        size_t i = 0;
        int replaced = 0;

        while (str[i] != '\0') {
                int len;

                if (is_whitelisted(str[i], white)) {
                        i++;
                        continue;
                }

                /* accept hex encoding */
                if (str[i] == '\\' && str[i+1] == 'x') {
                        i += 2;
                        continue;
                }

                /* accept valid utf8 */
                len = utf8_encoded_valid_unichar(&str[i]);
                if (len > 1) {
                        i += len;
                        continue;
                }

                /* if space is allowed, replace whitespace with ordinary space */
                if (isspace(str[i]) && white != NULL && strchr(white, ' ') != NULL) {
                        str[i] = ' ';
                        i++;
                        replaced++;
                        continue;
                }

                /* everything else is replaced with '_' */
                str[i] = '_';
                i++;
                replaced++;
        }
        return replaced;
}
#endif
