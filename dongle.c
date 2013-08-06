#include "dongle.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#ifndef _WIN32
#include <unistd.h>
#include <termios.h>
#include <sys/stat.h>
#include <fcntl.h>
#else
#include <tchar.h>
#endif


#ifdef _WIN32
static void dongleErrorWin32 (LPCTSTR msg, DWORD err) {
  LPVOID errorText;

  FormatMessage(FORMAT_MESSAGE_FROM_SYSTEM
      | FORMAT_MESSAGE_ALLOCATE_BUFFER
      | FORMAT_MESSAGE_IGNORE_INSERTS,
      NULL,
      err,
      MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
      (LPTSTR)&errorText,
      0,
      NULL);

  assert(errorText);
  _ftprintf(stderr, _T("%s: %s\n"), msg, (LPCTSTR)errorText);
  LocalFree(errorText);
}

#endif

static void dongleInit (MOBOTdongle *dongle);
static void dongleFini (MOBOTdongle *dongle);

static ssize_t dongleWriteRaw (MOBOTdongle *dongle, const uint8_t *buf, size_t len) {
  assert(dongle && buf);
  ssize_t ret;
#ifdef _WIN32
  /* hlh: note that the original incarnation of this code segment in commsOutEngine
   * included the comment, "Write one whole message at a time". While libsfp
   * by default tries to call the write callback (i.e., this function) with
   * exactly one frame's worth of data, if the frames exceed its writebuffer
   * limit (SFP_CONFIG_WRITEBUF_SIZE), it will call the write callback more
   * frequently. I don't know if this is an issue or not. */

  DWORD bytesWritten;
  BOOL b = WriteFile(
        dongle->handle, 
        buf, 
        len,
        &bytesWritten,
        dongle->ovOutgoing);
  if (b) {
    return bytesWritten;
  }

  DWORD err = GetLastError();
  if (ERROR_IO_PENDING != err) {
    dongleErrorWin32(_T("(barobo) ERROR: in dongleWriteRaw, "
        "WriteFile()"), err);
    return -1;
  }

  if (!GetOverlappedResult(dongle->handle, dongle->ovOutgoing, &bytesWritten, TRUE)) {
    dongleErrorWin32(_T("(barobo) ERROR: in dongleWriteRaw, "
          "GetOverlappedResult()"), GetLastError());
    return -1;
  }

  if (!ResetEvent(dongle->ovOutgoing->hEvent)) {
    dongleErrorWin32(_T("(barobo) ERROR: in dongleWriteRaw, "
          "ResetEvent()"), GetLastError());
    return -1;
  }

  ret = bytesWritten;
#else
  ret = write(dongle->fd, buf, len);

  if (-1 == ret) {
    char errbuf[256];
    strerror_r(errno, errbuf, sizeof(errbuf));
    fprintf(stderr, "(barobo) ERROR: in dongleWriteRaw, write(): %s\n", errbuf);
  }
#endif

  return ret;
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
  BOOL b = GetOverlappedResult(dongle->handle, dongle->ovIncoming, &readbytes, TRUE);

  if (!b) {
    int err = GetLastError();
    if (ERROR_IO_INCOMPLETE == err) {
      /* I suppose this is analagous to the POSIX select() false positive
       * case. It may be an impossible state in Windows, but I'll treat it
       * similarly for the sake of symmetry. */
      fprintf(stderr, "(barobo) WARNING: in dongleTimedReadRaw, "
          "MsgWaitForMultipleObjects() returned false positive\n");
      readbytes = 0;
    }
    else {
      dongleErrorWin32(_T("(barobo) ERROR: in dongleTimedReadRaw, "
          "GetOverlappedResult()\n"), err);
      readbytes = -1;
    }
  }

  if (!ResetEvent(dongle->ovIncoming->hEvent)) {
    dongleErrorWin32(_T("(barobo) ERROR: in dongleTimedReadRaw, "
          "ResetEvent()\n"), GetLastError());
    readbytes = -1;
  }

  return readbytes;
}

/* Returns -1 on error, 0 on timeout, otherwise the number of bytes that are
 * ready to be read. */
static int dongleTimedReadRawWin32WaitForInput (MOBOTdongle* dongle, const long *ms_delay) {
  DWORD errmask = 0;
  COMSTAT comstat;
  if (!ClearCommError(dongle->handle, &errmask, &comstat)) {
    dongleErrorWin32(_T("(barobo) ERROR: in dongleTimedReadRaw, "
          "ClearCommError()"), GetLastError());
    return -1;
  }

  if (errmask) {
    fprintf(stderr, "(barobo) ERROR: Communications error: 0x%04x\n", errmask);
  }

  if (comstat.cbInQue) {
    return comstat.cbInQue;
  }

  fprintf(stderr, "Waiting for a communications event...\n");

  /* Nothing in the buffer yet, so wait for it... */
  DWORD commEvent = 0;
  if (!WaitCommEvent(dongle->handle, &commEvent, dongle->ovIncoming)) {
    DWORD err = GetLastError();
    if (ERROR_IO_PENDING != err) {
      dongleErrorWin32(_T("(barobo) ERROR: in dongleTimedReadRaw, "
            "WaitCommEvent()"), err);
      return -1;
    }

    DWORD code = WaitForSingleObject(dongle->ovIncoming->hEvent,
        ms_delay ? *ms_delay : INFINITE);

    switch (code) {
      case WAIT_OBJECT_0:
        break;
      case WAIT_ABANDONED:
        assert(0);
      case WAIT_TIMEOUT:
        fprintf(stderr, "Timed out (for real)\n");
        return 0;
      case WAIT_FAILED:
        dongleErrorWin32(_T("(barobo) ERROR: in dongleTimedReadRaw, "
              "WaitForSingleObject()"), GetLastError());
        return -1;
    }

    DWORD unused;
    if (!GetOverlappedResult(dongle->handle, dongle->ovIncoming, &unused, TRUE)) {
      dongleErrorWin32(_T("(barobo) ERROR: in dongleTimedReadRaw, "
            "GetOverlappedResult()"), GetLastError());
      return -1;
    }

    if (!ResetEvent(dongle->ovIncoming->hEvent)) {
      dongleErrorWin32(_T("(barobo) ERROR: in dongleTimedReadRaw, "
            "ResetEvent()"), GetLastError());
      return -1;
    }
  }

  /* Even though dongleOpen calls SetCommMask with only EV_RXCHAR, Windows
   * sometimes spits other events at us. For now, ignore them and pretend it
   * was a timeout. They don't happen that often. */
  if (!(EV_RXCHAR & commEvent)) {
    fprintf(stderr, "Received unwanted comm event: 0x%04x\n", commEvent);
    return 0;
  }

  /* Get the number of bytes that are in the buffer so far. */
  if (!ClearCommError(dongle->handle, NULL, &comstat)) {
    dongleErrorWin32(_T("(barobo) ERROR: in dongleTimedReadRaw, "
          "ClearCommError()"), GetLastError());
    return -1;
  }

  fprintf(stderr, "Returning buffer size: %d\n", comstat.cbInQue);

  return comstat.cbInQue;
}

static ssize_t dongleTimedReadRaw (MOBOTdongle *dongle, uint8_t *buf, size_t len, const long *ms_delay) {
  assert(dongle && buf);

  int bytesToBeRead = dongleTimedReadRawWin32WaitForInput(dongle, ms_delay);
  if (bytesToBeRead < 1) {
    /* Either timed out, or error. */
    fprintf(stderr, "dongleTimedReadRawWin32WaitForInput returned %d\n", bytesToBeRead);
    return bytesToBeRead;
  }

  DWORD readbytes = 0;
  BOOL b = ReadFile(dongle->handle, buf, len, &readbytes, dongle->ovIncoming);

  if (b) {
    /* ReadFile completed synchronously. */
    return readbytes;
  }

  DWORD err = GetLastError();
  if (ERROR_IO_PENDING != err) {
    dongleErrorWin32(_T("(barobo) ERROR: in dongleTimedReadRaw, "
        "ReadFile()"), err);
    return -1;
  }

  return dongleTimedReadRawWin32GetResult(dongle);
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
  long ms_delay = 1000;
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

static int sfp_write (uint8_t *octets, size_t len, size_t *outlen, void *data) {
  MOBOTdongle *dongle = (MOBOTdongle *)data;
  ssize_t ret = dongleWriteRaw(dongle, octets, len);
  if (outlen && ret >= 0) {
    *outlen = ret;
    ret = 0;
  }
  return ret;
}

ssize_t dongleWrite (MOBOTdongle *dongle, const uint8_t *buf, size_t len) {
  assert(dongle);
  assert(buf);

  if (MOBOT_DONGLE_FRAMING_SFP == dongle->framing) {
    sfpWritePacket(dongle->sfpContext, buf, len, NULL);
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
    if (-1 == err) {
      return -1;
    }
    if (1 != err) {
      continue;
    }

    if (MOBOT_DONGLE_FRAMING_SFP == dongle->framing) {
      size_t outlen = 0;
      int ret = sfpDeliverOctet(dongle->sfpContext, byte, buf, len, &outlen);
      if (ret > 0) {
        /* We got a packet. :) */
        return outlen;
      }
      else if (ret < 0) {
        /* We got an error. :( */
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
    sfpDeliverOctet(dongle->sfpContext, byte, NULL, 0, NULL);
  }

  return 0;
}

static void dongleInit (MOBOTdongle *dongle) {
  dongle->framing = MOBOT_DONGLE_FRAMING_UNKNOWN;
  /* We don't malloc() sfpContext yet, because we don't know if we'll actually
   * need it until we detect the framing used by the firmware. */
  dongle->sfpContext = NULL;

#ifdef _WIN32
  /* Halp! Is this right? */
  dongle->handle = NULL;

  /* Initialize overlapped communication shit */
  dongle->ovIncoming = malloc(sizeof(OVERLAPPED));
  memset(dongle->ovIncoming, 0, sizeof(OVERLAPPED));
  dongle->ovIncoming->hEvent = CreateEvent(0, 1, 0, 0);

  dongle->ovOutgoing = malloc(sizeof(OVERLAPPED));
  memset(dongle->ovOutgoing, 0, sizeof(OVERLAPPED));
  dongle->ovOutgoing->hEvent = CreateEvent(0, 1, 0, 0);

#else
  dongle->fd = -1;
#endif
}

static void dongleFini (MOBOTdongle *dongle) {
  dongle->framing = MOBOT_DONGLE_FRAMING_UNKNOWN;
  if (dongle->sfpContext) {
    free(dongle->sfpContext);
    dongle->sfpContext = NULL;
  }

#ifdef _WIN32
  if (dongle->ovIncoming) {
    CloseHandle(dongle->ovIncoming->hEvent);
    free(dongle->ovIncoming);
    dongle->ovIncoming = NULL;
  }
  if (dongle->ovOutgoing) {
    CloseHandle(dongle->ovOutgoing->hEvent);
    free(dongle->ovOutgoing);
    dongle->ovOutgoing = NULL;
  }
#endif
}

#ifdef _WIN32

/* WIN32 implementation of dongleOpen. */

int dongleOpen (MOBOTdongle *dongle, const char *ttyfilename, unsigned long baud) {
  assert(dongle);
  assert(ttyfilename);

  dongleInit(dongle);

  /* hlh: for some reason, when I print to stdout and then to stderr, there
   * unicode issues */
  _ftprintf(stderr, _T("(barobo) INFO: opening %s with baud<%d>\n"), ttyfilename, baud);

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
  if(dongle->handle == INVALID_HANDLE_VALUE) {
    dongleErrorWin32(_T("(barobo) ERROR: in dongleOpen, CreateFile()"), GetLastError());
    return -1;
  }
  free(tty);
  _ftprintf(stderr, _T("(barobo) INFO: opened %s\n"), ttyfilename);
  /* Adjust settings */
  DCB dcb;
  FillMemory(&dcb, sizeof(dcb), 0);
  dcb.DCBlength = sizeof(dcb);
  char dcbconf[64];
  snprintf(dcbconf, sizeof(dcbconf), "%lu,n,8,1", baud);
  if (!BuildCommDCB(dcbconf, &dcb)) {
    dongleErrorWin32(_T("(barobo) ERROR: in dongleOpen, BuildCommDCB()"), GetLastError());
    dongleClose(dongle);
    return -1;
  }
  dcb.BaudRate = baud;
  if (!SetCommState(dongle->handle, &dcb)) {
    dongleErrorWin32(_T("(barobo) ERROR: in dongleOpen, SetCommState()"), GetLastError());
    dongleClose(dongle);
    return -1;
  }

  if (!GetCommTimeouts(dongle->handle, &dongle->oldCommTimeouts)) {
    dongleErrorWin32(_T("(barobo) ERROR: in dongleOpen, GetCommTimeouts()"),
        GetLastError());
    dongleClose(dongle);
    return -1;
  }

  COMMTIMEOUTS timeouts;
  memcpy(&timeouts, &dongle->oldCommTimeouts, sizeof(timeouts));
  /* This should make ReadFile() return immediately, even when using non-
   * overlapped (synchronous) I/O. We still have to use asynchronous I/O for
   * WaitCommEvent(), however. The reason I'm doing this is so that I can
   * implement a Win32 version of dongleTimedReadRaw() that closely mimics the
   * behavior of the POSIX version, specifically: if dongleTimedReadRaw() is
   * called with a large buffer of size n bytes, and a timeout of t
   * milliseconds, and the kernel receives k bytes, where k < n, within the
   * timeout interval t, the function should return those k bytes in the
   * caller's buffer. Without setting the timeouts as follows, the Win32
   * behavior would be to claim to be unable to fulfill the request, and
   * return zero bytes, which is not the answer we want. */
  timeouts.ReadIntervalTimeout = MAXDWORD;
  timeouts.ReadTotalTimeoutMultiplier = 0;
  timeouts.ReadTotalTimeoutConstant = 0;

  if (!SetCommTimeouts(dongle->handle, &timeouts)) {
    dongleErrorWin32(_T("(barobo) ERROR: in dongleOpen, SetCommTimeouts()"),
        GetLastError());
    dongleClose(dongle);
    return -1;
  }

  SetCommMask(dongle->handle, 0);
  if (!SetCommMask(dongle->handle, EV_RXCHAR)) {
    dongleErrorWin32(_T("(barobo) ERROR: in dongleOpen, SetCommMask()"),
        GetLastError());
    dongleClose(dongle);
    return -1;
  }

  if (-1 == dongleDetectFraming(dongle)) {
    fprintf(stderr, "(barobo) INFO: could not detect dongle framing, giving up\n");
    dongleClose(dongle);
    return -1;
  }

  if (MOBOT_DONGLE_FRAMING_SFP == dongle->framing) {
    int status = dongleSetupSFP(dongle);
    if (-1 == status) {
      dongleClose(dongle);
      return -1;
    }
  }

  return 0;
}

#else /* WIN32 */

/* POSIX implementation of dongleOpen. */

static speed_t baud_to_speed_t (unsigned long baud) {
  /* Oh, the humanity */
  switch (baud) {
    case 230400:
      return B230400;
    case 500000:
      return B500000;
    default:
      return B0;
  }
}

int dongleOpen (MOBOTdongle *dongle, const char *ttyfilename, unsigned long baud) {
  assert(dongle);
  assert(ttyfilename);

  dongleInit(dongle);

  printf("(barobo) INFO: opening %s with baud<%d>\n", ttyfilename, baud);

  /* We use non-blocking I/O here so we can support the POSIX implementation
   * of dongleTimedReadRaw. dongleTimedReadRaw uses select, which can, per the
   * man page, return false positives. A false positive would then cause the
   * subsequent read call to block. */
  dongle->fd = open(ttyfilename, O_NONBLOCK | O_RDWR | O_NOCTTY);
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

  speed_t speed = baud_to_speed_t(baud);
  assert(B0 != speed);

  cfsetspeed(&term, speed);
  cfsetispeed(&term, speed);
  cfsetospeed(&term, speed);
  int status;

  if(status = tcsetattr(dongle->fd, TCSANOW, &term)) {
    char errbuf[256];
    strerror_r(errno, errbuf, sizeof(errbuf));
    fprintf(stderr, "(barobo) ERROR: Configuring %s: %s.\n", ttyfilename, errbuf);
  }
  tcgetattr(dongle->fd, &term);
  if(cfgetispeed(&term) != speed) {
    fprintf(stderr, "(barobo) ERROR: Unable to set %s input speed.\n", ttyfilename);
    exit(0);
  }
  if(cfgetospeed(&term) != speed) {
    fprintf(stderr, "(barobo) ERROR: Unable to set %s output speed.\n", ttyfilename);
    exit(0);
  }

  tcflush(dongle->fd, TCIOFLUSH);
  dongleDetectFraming(dongle);
  tcflush(dongle->fd, TCIOFLUSH);

  if (MOBOT_DONGLE_FRAMING_SFP == dongle->framing) {
    status = dongleSetupSFP(dongle);
    if (-1 == status) {
      dongleClose(dongle);
      return -1;
    }
  }

  //dongle->status = MOBOT_LINK_STATUS_UP;

  return 0;
}

#endif

void dongleClose (MOBOTdongle *dongle) {
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
  /* I dunno, MSDN said you're supposed to reset these to the way you found
   * them. Whatever. */
  if (!SetCommTimeouts(dongle->handle, &dongle->oldCommTimeouts)) {
    dongleErrorWin32(_T("(barobo) ERROR: in dongleClose, SetCommTimeouts()"),
        GetLastError());
  }

  BOOL b = CloseHandle(dongle->handle);
  if (!b) {
    dongleErrorWin32(_T("(barobo) ERROR: in dongleClose, CloseHandle()"), GetLastError());
  }
#else
  int err = close(dongle->fd);
  if (-1 == err) {
    char errbuf[256];
    strerror_r(errno, errbuf, sizeof(errbuf));
    fprintf(stderr, "(barobo) ERROR: in dongleClose, close(): %s\n", errbuf);
  }
#endif

  dongleFini(dongle);
}
