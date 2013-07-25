#include "dongle.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include <unistd.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

static ssize_t dongleWriteRaw (MOBOTdongle *dongle, const uint8_t *buf, size_t len) {
  ssize_t err;
#ifdef _WIN32
  /* hlh: note that the original incarnation of this code segment in commsOutEngine
   * included the comment, "Write one whole message at a time". While libsfp
   * by default tries to call the write callback (i.e., this function) with
   * exactly one frame's worth of data, if the frames exceed its writebuffer
   * limit (SFP_CONFIG_WRITEBUF_SIZE), it will call the write callback more
   * frequently. I don't know if this is an issue or not. */
  if(!WriteFile(
        dongle->handle, 
        octets, 
        len,
        NULL,
        dongle->ovOutgoing)) {
    //printf("Error writing. %d \n", GetLastError());
  }
  DWORD bytesWritten;
  GetOverlappedResult(dongle->handle, dongle->ovOutgoing, &bytesWritten, TRUE);
  ResetEvent(comms->ovOutgoing->hEvent);
#else
  err = write(dongle->fd, buf, len);

  if (-1 == err) {
    char errbuf[256];
    strerror_r(errno, errbuf, sizeof(errbuf));
    fprintf(stderr, "(barobo) ERROR: in dongleWriteRaw, write(): %s\n", errbuf);
  }
#endif

  return err;
}

static ssize_t dongleReadRaw (MOBOTdongle *dongle, uint8_t *buf, size_t len) {
  ssize_t err;
#ifdef _WIN32
  DWORD readbytes = 0;
#if 0
  /* DEBUG */
    if(!SetCommMask(comms->commHandle, EV_RXFLAG)) {
      fprintf(stderr, "Could not set Comm flags to detect RX.\n");
    }
    if(WaitCommEvent(comms->commHandle, &mask, &comms->ovIncoming) == FALSE) {
      WaitForSingleObject(comms->ovIncoming.hEvent, INFINITE);
    }
    printf("Byte received\n");
    exit(0);
#endif
  /* Use CancelIOEx() to cancel this blocking read */
   /* 
  if(readbytes == 0) {
    if(!SetCommMask(comms->commHandle, EV_RXFLAG)) {
      fprintf(stderr, "Could not set Comm flags to detect RX.\n");
    }
    if(WaitCommEvent(comms->commHandle, &mask, &comms->ovIncoming) == FALSE) {
      printf("Watiing...\n");
      WaitForSingleObject(comms->ovIncoming.hEvent, INFINITE);
    }
  }
  */
  ReadFile(
      dongle->handle,
      buf,
      len,
      NULL,
      dongle->ovIncoming);
  /*
  WaitForMultipleObjects(
      2,
      events,
      false,
      INFINITE);
      */
  GetOverlappedResult(dongle->handle, comms->ovIncoming, &readbytes, TRUE);
  //ResetEvent(events[0]);
  //ResetEvent(events[1]);
  //CloseHandle(comms->ovIncoming.hEvent);
  err = readbytes;
#else
  err = read(dongle->fd, buf, len);

  if (-1 == err) {
    char errbuf[256];
    strerror_r(errno, errbuf, sizeof(errbuf));
    fprintf(stderr, "(barobo) ERROR: in dongleReadRaw, read(): %s\n", errbuf);
  }
#endif

  return err;
}

static ssize_t dongleReadNonBlockRaw (MOBOTdongle *dongle, uint8_t *buf, size_t len) {
  int flags = fcntl(dongle->fd, F_GETFL);
  if (-1 == flags) {
    char errbuf[256];
    strerror_r(errno, errbuf, sizeof(errbuf));
    fprintf(stderr, "(barobo) ERROR: in dongleReadNonBlockRaw, fcntl(): %s\n", errbuf);
    return -1;
  }

  /* Turn on non-blocking. */
  int err = fcntl(dongle->fd, F_SETFL, O_NONBLOCK | flags);
  if (-1 == err) {
    char errbuf[256];
    strerror_r(errno, errbuf, sizeof(errbuf));
    fprintf(stderr, "(barobo) ERROR: in dongleReadNonBlockRaw, fcntl(): %s\n", errbuf);
    return -1;
  }

  /* Perform the read. */
  ssize_t ret = dongleReadRaw(dongle, buf, len);

  /* Restore the flags. */
  err = fcntl(dongle->fd, F_SETFL, flags);
  if (-1 == err) {
    char errbuf[256];
    strerror_r(errno, errbuf, sizeof(errbuf));
    fprintf(stderr, "(barobo) ERROR: in dongleReadNonBlockRaw, fcntl(): %s\n", errbuf);
    return -1;
  }

  return ret;
}

static int dongleWaitUntilReadable (MOBOTdongle *dongle, long us_delay) {
  fd_set rfds;
  FD_ZERO(&rfds);
  FD_SET(dongle->fd, &rfds);

  struct timeval timeout;
  timeout.tv_sec = 0;
  timeout.tv_usec = us_delay;

  int err = select(dongle->fd + 1, &rfds, NULL, NULL, &timeout);

  if (-1 == err) {
    char errbuf[256];
    strerror_r(errno, errbuf, sizeof(errbuf));
    fprintf(stderr, "(barobo) ERROR: in dongleIsReadyToRead, select(): %s\n", errbuf);
    return 0;
  }

  return FD_ISSET(dongle->fd, &rfds);
}

static int dongleDetectFraming (MOBOTdongle *dongle) {
  /* The following is a magic octet string which the old firmware, which used
   * no framing, should parse correctly, but which will look like one
   * complete, though corrupt, frame to libsfp. */
  static const uint8_t detection[] = {
    0x7e, // flag in command location--the old firmware never reads this octet
    0x09, // length, including the terminating flag
    0x00, 0x00, 0x01, // ZigBee stuff
    0x30, 0x03, 0x00, // CMD_GETSTATUS
    0x7e  // flag--the old firmware never asserts that this has to be 0x00
  };

  ssize_t err = dongleWriteRaw(dongle, detection, sizeof(detection));
  if (-1 == err) {
    return -1;
  }
  else if (sizeof(detection) != err) {
    fprintf(stderr, "(barobo) ERROR: in dongleDetectFraming, unable to write "
        "complete detection string.\n");
    return -1;
  }

  /* Give the dongle a little time to think about it. */
  long us_delay = 500000; // half a second
  if (!dongleWaitUntilReadable(dongle, us_delay)) {
    fprintf(stderr, "(barobo) ERROR: in dongleDetectFraming, timed out "
        "waiting for response.\n");
    return -1;
  }

  uint8_t response[256];
  err = dongleReadNonBlockRaw(dongle, response, sizeof(response));
  if (-1 == err) {
    return -1;
  }

  assert(err <= sizeof(response));

  fprintf(stderr, "(barobo) DEBUG: dongleDetectFraming received:");
  for (size_t i = 0; i < err; ++i) {
    fprintf(stderr, " %02x", response[i]);
  }
  fprintf(stderr, "\n");

  static const uint8_t old_response[]
    = { 0x10, 0x09, 0x00, 0x00, 0x01, 0x10, 0x03, 0x11, 0x11 };

  if (sizeof(old_response) <= err
      && !memcmp(response, old_response, sizeof(old_response))) {
    fprintf(stderr, "(barobo) INFO: old (unframed serial protocol) firmware detected.\n");
    dongle->framing = MOBOT_DONGLE_FRAMING_NONE;
  }
  else {
    fprintf(stderr, "(barobo) INFO: new (framed serial protocol) firmware detected.\n");
    dongle->framing = MOBOT_DONGLE_FRAMING_SFP;
  }

  return 0;
}

static void sfp_lock (void *data) {
  MUTEX_LOCK((MUTEX_T *)data);
}

static void sfp_unlock (void *data) {
  MUTEX_UNLOCK((MUTEX_T *)data);
}

static ssize_t sfp_write (uint8_t *octets, size_t len, void *data) {
  MOBOTdongle *dongle = (MOBOTdongle *)data;
  return dongleWriteRaw(dongle, octets, len);
}

ssize_t dongleWrite (MOBOTdongle *dongle, const uint8_t *buf, size_t len) {
  assert(dongle);
  assert(buf);

  if (MOBOT_DONGLE_FRAMING_SFP == dongle->framing) {
    sfpWritePacket(dongle->sfpContext, buf, len);
    return len;
  }
  else {
    return dongleWriteRaw(dongle, buf, len);
  }
}

ssize_t dongleRead (MOBOTdongle *dongle, uint8_t *buf, size_t len) {
  assert(dongle);
  assert(buf);

  size_t i = 0;

  while (1) {
    uint8_t byte;
    int err = dongleReadRaw(dongle, &byte, 1);
    if (1 != err) {
      return -1;
    }

    if (MOBOT_DONGLE_FRAMING_SFP == dongle->framing) {
      ssize_t ret = sfpDeliverOctet(dongle->sfpContext, byte, buf, len);
      if (ret) {
        return ret;
      }
    }
    else {
      /* The old way, where we rely on the length, in the second octet of a packet. */
      if (i >= len) {
        return -1;
      }
      buf[i++] = byte;
      if (i > 1 && buf[1] == i) {
        return i;
      }
    }
  }

  return -1;
}

static int dongleSetupSFP (MOBOTdongle *dongle) {
  MUTEX_NEW(dongle->sfpTxLock);
  MUTEX_INIT(dongle->sfpTxLock);

  dongle->sfpContext = malloc(sizeof(SFPcontext));
  assert(dongle->sfpContext);

  sfpInit(dongle->sfpContext);
  
  sfpSetWriteCallback(dongle->sfpContext, SFP_WRITE_MULTIPLE, sfp_write, dongle);
  //sfpSetDeliverCallback(dongle->sfpContext, sfp_deliver, dongle);
  sfpSetLockCallback(dongle->sfpContext, sfp_lock, dongle->sfpTxLock);
  sfpSetUnlockCallback(dongle->sfpContext, sfp_unlock, dongle->sfpTxLock);

  sfpConnect(dongle->sfpContext);

  while (!sfpIsConnected(dongle->sfpContext)) {
    // nasty hax, this could block or die :(
    uint8_t byte;
    int err = dongleReadRaw(dongle, &byte, 1);
    if (-1 == err) {
      return -1;
    }
    assert(1 == err);
    sfpDeliverOctet(dongle->sfpContext, byte, NULL, 0);
  }

  return 0;
}

void dongleDisconnect (MOBOTdongle *dongle) {
  if (!dongle) {
    return;
  }

  if (MOBOT_DONGLE_FRAMING_SFP == dongle->framing) {
    /* sfpInit effectively disconnects */
    sfpInit(dongle->sfpContext);
    MUTEX_DESTROY(dongle->sfpTxLock);
    free(dongle->sfpTxLock);
    free(dongle->sfpContext);
  }
}

#ifdef __MACH__
#define BAUD 500000
#else
#define BAUD B230400
#endif

int dongleConnect (MOBOTdongle *dongle, const char *ttyfilename) {
  assert(dongle);
  assert(ttyfilename);

  dongle->framing = MOBOT_DONGLE_FRAMING_UNKNOWN;
  dongle->sfpContext = NULL;
  dongle->fd = -1;

  dongle->fd = open(ttyfilename, O_RDWR | O_NOCTTY | O_ASYNC);
  if(-1 == dongle->fd) {
    char errbuf[256];
    strerror_r(errno, errbuf, sizeof(errbuf));
    fprintf(stderr, "(barobo) ERROR: in dongleConnect, open(): %s\n", errbuf);
    return -1;
  }

  /* Change the baud rate to 57600 */
  struct termios term;
  tcgetattr(dongle->fd, &term);

  // Input flags - Turn off input processing
  // convert break to null byte, no CR to NL translation,
  // no NL to CR translation, don't mark parity errors or breaks
  // no input parity check, don't strip high bit off,
  // no XON/XOFF software flow control
  //
  term.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
      INLCR | PARMRK | INPCK | ISTRIP | IXON);
  //
  // Output flags - Turn off output processing
  // no CR to NL translation, no NL to CR-NL translation,
  // no NL to CR translation, no column 0 CR suppression,
  // no Ctrl-D suppression, no fill characters, no case mapping,
  // no local output processing
  //
  // term.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
  //                     ONOCR | ONOEOT| OFILL | OLCUC | OPOST);
  term.c_oflag = 0;
  //
  // No line processing:
  // echo off, echo newline off, canonical mode off, 
  // extended input processing off, signal chars off
  //
  term.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
  //
  // Turn off character processing
  // clear current char size mask, no parity checking,
  // no output processing, force 8 bit input
  //
  term.c_cflag &= ~(CSIZE | PARENB);
  term.c_cflag |= CS8;
  //
  // One input byte is enough to return from read()
  // Inter-character timer off
  //
  term.c_cc[VMIN]  = 1;
  term.c_cc[VTIME] = 0;

  // Communication speed

  cfsetspeed(&term, BAUD);
  cfsetispeed(&term, BAUD);
  cfsetospeed(&term, BAUD);
  int status;

  if(status = tcsetattr(dongle->fd, TCSANOW, &term)) {
    char errbuf[256];
    strerror_r(errno, errbuf, sizeof(errbuf));
    fprintf(stderr, "(barobo) ERROR: Configuring %s: %s.\n", ttyfilename, errbuf);
  }
  tcgetattr(dongle->fd, &term);
  if(cfgetispeed(&term) != BAUD) {
    fprintf(stderr, "(barobo) ERROR: Unable to set %s input speed.\n", ttyfilename);
    exit(0);
  }
  if(cfgetospeed(&term) != BAUD) {
    fprintf(stderr, "(barobo) ERROR: Unable to set %s output speed.\n", ttyfilename);
    exit(0);
  }

  tcflush(dongle->fd, TCIOFLUSH);
  dongleDetectFraming(dongle);
  tcflush(dongle->fd, TCIOFLUSH);

  if (MOBOT_DONGLE_FRAMING_SFP == dongle->framing) {
    status = dongleSetupSFP(dongle);
    if (-1 == status) {
      dongleDisconnect(dongle);
      return -1;
    }
  }

  //dongle->status = MOBOT_CONNECTION_STATUS_UP;

  return 0;
}
