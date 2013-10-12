#ifndef _BAROBO_DONGLE_H_
#define _BAROBO_DONGLE_H_

#include "thread_macros.h"
#include "libsfp/serial_framing_protocol.h"

#include <sys/types.h>

#ifdef _WIN32
#include <Windows.h>
#endif

typedef enum MOBOTdongleFraming {
  MOBOT_DONGLE_FRAMING_UNKNOWN,
  MOBOT_DONGLE_FRAMING_NONE,
  MOBOT_DONGLE_FRAMING_SFP
} MOBOTdongleFraming;

typedef struct MOBOTdongle {
#ifdef _WIN32
  /* XXX hlh: there was some !defined(CH) bullshit here in this code's
   * original incarnation in mobot.h. The else case declared these identifiers
   * as void pointers. */
  HANDLE handle;
  LPOVERLAPPED ovIncoming;
  LPOVERLAPPED ovOutgoing;
  COMMTIMEOUTS oldCommTimeouts;
#else
  int fd;
#endif
  MOBOTdongleFraming framing;
  SFPcontext *sfpContext;
  MUTEX_T *sfpTxLock;
} MOBOTdongle;

int dongleOpen (MOBOTdongle *dongle, const char *ttyfilename, unsigned long baud);
void dongleClose (MOBOTdongle *dongle);

ssize_t dongleRead (MOBOTdongle *dongle, uint8_t *buf, size_t len);
ssize_t dongleWrite (MOBOTdongle *dongle, const uint8_t *buf, size_t len);

#endif