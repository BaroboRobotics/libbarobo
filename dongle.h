#ifndef _BAROBO_DONGLE_H_
#define _BAROBO_DONGLE_H_

#include "thread_macros.h"
#include "libsfp/serial_framing_protocol.h"

typedef enum MOBOTdongleFraming {
  MOBOT_DONGLE_FRAMING_UNKNOWN,
  MOBOT_DONGLE_FRAMING_NONE,
  MOBOT_DONGLE_FRAMING_SFP
} MOBOTdongleFraming;

typedef struct MOBOTdongle {
  int fd;
  MOBOTdongleFraming framing;
  SFPcontext *sfpContext;
  MUTEX_T *sfpTxLock;
} MOBOTdongle;

int dongleConnect (MOBOTdongle *dongle, const char *ttyfilename);
void dongleDisconnect (MOBOTdongle *dongle);

ssize_t dongleRead (MOBOTdongle *dongle, uint8_t *buf, size_t len);
ssize_t dongleWrite (MOBOTdongle *dongle, const uint8_t *buf, size_t len);

#endif
