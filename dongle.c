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

/* NULL ms_delay means wait forever, otherwise it is a pointer to the number
 * of milliseconds to wait.
 *
 * Returns 0 on timeout (or read() returning zero), -1 on error, number of
 * bytes read otherwise. */
static ssize_t dongleTimedReadRaw (MOBOTdongle *dongle, uint8_t *buf,
    size_t len, const long *ms_delay);

#ifdef _WIN32

/* WIN32 implementation of dongleTimedReadRaw. */

/* A Win32-only auxiliary function for dongleTimedReadRaw. */
static DWORD dongleTimedReadRawWin32GetResult (MOBOTdongle *dongle) {
  DWORD readbytes = 0;
  BOOL b = GetOverlappedResult(dongle->handle, dongle->ovIncoming, &readbytes, FALSE);

  if (!b) {
    err = GetLastError();
    if (ERROR_IO_INCOMPLETE == err) {
      /* I suppose this is analagous to the POSIX select() false positive
       * case. It may be an impossible state in Windows, but I'll treat it
       * similarly for the sake of symmetry. */
      fprintf(stderr, "(barobo) WARNING: in dongleTimedReadRaw, "
          "MsgWaitForMultipleObjects() returned false positive\n");
      readbytes = 0;
    }
    else {
      fprintf(stderr, "(barobo) ERROR: in dongleTimedReadRaw, "
          "GetOverlappedResult(): %d\n", GetLastError());
      readbytes = -1;
    }
  }

  return readbytes;
}

static ssize_t dongleTimedReadRaw (MOBOTdongle *dongle, uint8_t *buf, size_t len, const long *ms_delay) {
  DWORD delay = ms_delay ? *ms_delay : INFINITE;

  DWORD readbytes = 0;
  BOOL b = ReadFile(dongle->handle, buf, len, &readbytes, dongle->ovIncoming);

  if (b) {
    /* ReadFile completed synchronously. */
    return readbytes;
  }

  DWORD err = GetLastError();
  if (ERROR_IO_PENDING != err) {
    fprintf(stderr, "(barobo) ERROR: in dongleTimedReadRaw, "
        "ReadFile(): %d\n", err);
    return -1;
  }

  DWORD code = MsgWaitForMultipleObjects(1, &dongle->ovIncoming.hEvent, FALSE,
      delay, QS_ALLINPUT);

  switch (code) {
    case WAIT_OBJECT_0:
      /* Success! */
      readbytes = dongleTimedReadRawWin32GetResult(dongle);
      break;
    case WAIT_TIMEOUT:
      b = CancelIo(dongle->handle);
      if (!b) {
        fprintf(stderr, "(barobo) WARNING: in dongleTimedReadRaw, "
            "CancelIo(): %d\n", GetLastError());
      }
      readbytes = 0;
      break;
    case WAIT_FAILED:
      fprintf(stderr, "(barobo) ERROR: in dongleTimedReadraw, "
          "MsgWaitForMultipleObjects(): %d\n", GetLastError());
      readbytes = -1;
      break;
    default:
      /* Here's the deal: I do not grok the semantics of MsgWaitForMultipleObjects,
       * but I gather that it's safer to use than WaitFor*Object(s), which can
       * deadlock in a windowed environment. So I'm using the safer (apparently)
       * MsgWaitForMultipleObjects, and just forcing a crash if something
       * unexpected happens. */
      assert(0);
      break;
  }

  return readbytes;
}

#else /* _WIN32 */

/* POSIX implementation of dongleTimedReadRaw. */

static ssize_t dongleTimedReadRaw (MOBOTdongle *dongle, uint8_t *buf, size_t len, const long *ms_delay) {
  fd_set rfds;
  FD_ZERO(&rfds);
  FD_SET(dongle->fd, &rfds);

  struct timeval *ptimeout = NULL;
  struct timeval timeout;

  if (ms_delay) {
    timeout.tv_sec = 0;
    timeout.tv_usec = *ms_delay * 1000;
    ptimeout = &timeout;
  }

  int err = select(dongle->fd + 1, &rfds, NULL, NULL, ptimeout);

  if (-1 == err) {
    char errbuf[256];
    strerror_r(errno, errbuf, sizeof(errbuf));
    fprintf(stderr, "(barobo) ERROR: in dongleIsReadyToRead, select(): %s\n", errbuf);
    return -1;
  }

  if (!err || !FD_ISSET(dongle->fd, &rfds)) {
    /* We timed out. */
    return 0;
  }

  /* Perform the read. */
  err = read(dongle->fd, buf, len);

  if (-1 == err) {
    if (EAGAIN == errno || EWOULDBLOCK == errno) {
      /* The select() man page warns that this can happen: select() thinks a
       * file descriptor is ready for reading, but read() disagrees. In this
       * case, the proper way to handle it is probably to call select() again
       * with a delay of the amount of time remaining. This is easy to do
       * using the clock_gettime() POSIX call, which apparently doesn't exist
       * on OS X. So instead, let's just claim to have timed out--this case
       * should be exceedingly rare and probably harmless, anyway. */
      err = 0;
      fprintf(stderr, "(barobo) WARNING: in dongleTimedReadRaw, select() returned false positive\n");
    }
    else {
      char errbuf[256];
      strerror_r(errno, errbuf, sizeof(errbuf));
      fprintf(stderr, "(barobo) ERROR: in dongleReadRaw, read(): %s\n", errbuf);
    }
  }

  return err;
}

#endif

static ssize_t dongleReadRaw (MOBOTdongle *dongle, uint8_t *buf, size_t len) {
  return dongleTimedReadRaw(dongle, buf, len, NULL);
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
  long ms_delay = 500; // half a second
  uint8_t response[256];
  ssize_t bytesread = dongleTimedReadRaw(dongle, response, sizeof(response), &ms_delay);
  if (!bytesread) {
    fprintf(stderr, "(barobo) ERROR: in dongleDetectFraming, timed out "
        "waiting for response.\n");
    return -1;
  }

  assert(bytesread <= sizeof(response));

  fprintf(stderr, "(barobo) DEBUG: dongleDetectFraming received:");
  for (size_t i = 0; i < bytesread; ++i) {
    fprintf(stderr, " %02x", response[i]);
  }
  fprintf(stderr, "\n");

  static const uint8_t old_response[]
    = { 0x10, 0x09, 0x00, 0x00, 0x01, 0x10, 0x03, 0x11, 0x11 };

  if (sizeof(old_response) <= bytesread
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
  /* We do not currently use the libsfp deliver callback in libbarobo, but
   * rather use sfpDeliverOctet's output parameters to get the complete
   * packets. */
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

#ifdef _WIN32
  BOOL b = CloseHandle(dongle->handle);
  if (!b) {
    fprintf(stderr, "(barobo) ERROR: in dongleDisconnect, "
        "CloseHandle(): %d\n", GetLastError());
  }
#else
  int err = close(dongle->fd);
  if (-1 == err) {
    char errbuf[256];
    strerror_r(errno, errbuf, sizeof(errbuf));
    fprintf(stderr, "(barobo) ERROR: in dongleDisconnect, close(): %s\n", errbuf);
  }
#endif
}

void dongleInit (MOBOTdongle *dongle) {
  dongle->framing = MOBOT_DONGLE_FRAMING_UNKNOWN;
  dongle->sfpContext = NULL;
#ifdef _WIN32
  /* Halp! Is this right? */
  dongle->handle = NULL;

  /* Initialize overlapped communication shit */
  if (!dongle->ovIncoming) {
    comms->ovIncoming = (LPOVERLAPPED)malloc(sizeof(OVERLAPPED));
    memset(comms->ovIncoming, 0, sizeof(OVERLAPPED));
    comms->ovIncoming->hEvent = CreateEvent(0, 1, 0, 0);
  }
  if (!dongle->ovOutgoing) {
    comms->ovOutgoing = (LPOVERLAPPED)malloc(sizeof(OVERLAPPED));
    memset(comms->ovOutgoing, 0, sizeof(OVERLAPPED));
    comms->ovOutgoing->hEvent = CreateEvent(0, 1, 0, 0);
  }

#else
  dongle->fd = -1;
#endif
}

#ifdef _WIN32

/* WIN32 implementation of dongleConnect. */

int dongleConnect (MOBOTdongle *dongle, const char *ttyfilename) {
  assert(dongle);
  assert(ttyfilename);

  dongle->framing = MOBOT_DONGLE_FRAMING_UNKNOWN;
  dongle->sfpContext = NULL;

  /* Check the file name. If we receive something like "COM45", we want to
   * change it to "\\.\COM45" */
  char *tty;
  tty = (char*)malloc((sizeof(char)*strlen(ttyfilename))+20);
  tty[0] = '\0';
  if(ttyfilename[0] != '\\') {
    strcpy(tty, "\\\\.\\");
  }
  strcat(tty, ttyfilename);
  /* For windows, we should connect to a com port */
  dongle->handle = CreateFile(
      tty, 
      GENERIC_READ | GENERIC_WRITE,
      0,
      0,
      OPEN_EXISTING,
      FILE_FLAG_OVERLAPPED,
      0 );
  free(tty);
  if(dongle->handle == INVALID_HANDLE_VALUE) {
    //fprintf(stderr, "Error connecting to COM port: %s\n", ttyfilename);
    return -1;
  }
  /* Adjust settings */
  DCB dcb;
  FillMemory(&dcb, sizeof(dcb), 0);
  dcb.DCBlength = sizeof(dcb);
  if (!BuildCommDCB("230400,n,8,1", &dcb)) {
    fprintf(stderr, "Could not build DCB.\n");
    return -1;
  }
  dcb.BaudRate = 230400;

  if (!SetCommState(dongle->handle, &dcb)) {
    fprintf(stderr, "Could not set Comm State to new DCB settings.\n");
    return -1;
  }
  
  return 0;
}

#else /* WIN32 */

/* POSIX implementation of dongleConnect. */

/* FIXME figure out what this is all about */
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

  /* We use non-blocking I/O here so we can support the POSIX implementation
   * of dongleTimedReadRaw. dongleTimedReadRaw uses select, which can, per the
   * man page, return false positives. A false positive would then cause the
   * subsequent read call to block. */
  dongle->fd = open(ttyfilename, O_NONBLOCK | O_RDWR | O_NOCTTY | O_ASYNC);
  if(-1 == dongle->fd) {
    char errbuf[256];
    strerror_r(errno, errbuf, sizeof(errbuf));
    fprintf(stderr, "(barobo) ERROR: in dongleConnect, open(): %s\n", errbuf);
    return -1;
  }

  struct termios term;
  tcgetattr(dongle->fd, &term);

  /* The following code appears to be from:
   * http://en.wikibooks.org/wiki/Serial_Programming/termios
   * Refer there for appropriate comments--the gist is that we're setting up
   * a serial link suitable for passing binary data, rather than a traditional
   * ASCII terminal. The flag settings are very similar to cfmakeraw. */

  // Input flags - Turn off input processing
  term.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
      INLCR | PARMRK | INPCK | ISTRIP | IXON);

  // Output flags - Turn off output processing
  term.c_oflag = 0;

  // No line processing:
  term.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

  // Turn off character processing
  term.c_cflag &= ~(CSIZE | PARENB);
  term.c_cflag |= CS8;

  // One input byte is enough to return from read()
  // Inter-character timer off
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

  //dongle->status = MOBOT_LINK_STATUS_UP;

  return 0;
}
