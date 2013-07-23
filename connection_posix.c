#include <termios.h>
#include <unistd.h>

typedef struct MOBOTconnection {
  MOBOTconnectionType type;
  MOBOTconnectionStatus status;
  MOBOTconnectionFraming framing;
  int fd;
} MOBOTconnection;

static int connectionTTYConnect (MOBOTconnection *conn, const char *address);
static int connectionTCPConnect (MOBOTconnection *conn, const char *address);
static int connectionBluetoothConnect (MOBOTconnection *conn, const char *address);
static int connectionZigBeeConnect (MOBOTconnection *conn, const char *address);

void Mobot_connectionInit (MOBOTconnection *conn, MOBOTconnectionType type) {
  conn->type = type;
  conn->status = MOBOT_CONNECTION_STATUS_DOWN;
  conn->framing = MOBOT_CONNECTION_FRAMING_NONE;
  conn->fd = 0;
}

int Mobot_connectionConnect (MOBOTconnection *conn, const char *address) {
  assert(conn);
  switch (conn->type) {
    case MOBOT_CONNECTION_TYPE_TTY:
      return connectionTTYConnect(conn, address);
    case MOBOT_CONNECTION_TYPE_TCP:
      return connectionTCPConnect(conn, address);
    case MOBOT_CONNECTION_TYPE_BLUETOOTH:
      return connectionBluetoothConnect(conn, address);
    case MOBOT_CONNECTION_TYPE_ZIGBEE:
      return connectionZigBeeConnect(conn, address);
    default:
      assert(0);
      return -1;
  }
}

static int connectionWriteRaw (MOBOTconnection *conn, uint8_t *octets, size_t len) {
  int err = write(conn->fd, octets, len);
  if (-1 == err) {
    char barf[256];
    strerror_r(errno, barf, 256);
    fprintf(stderr, "(barobo) ERROR: write(): %s\n", barf);
  }
}

static void connectionDetectFraming (MOBOTconnection *conn) {
  connectionWrite
}

#ifdef __MACH__
#define BAUD 500000
#else
#define BAUD B230400
#endif

/* address is "/dev/tty*" */
static int connectonTTYConnect (MOBOTconnection *conn, const char* address) {
  conn->fd = open(address, O_RDWR | O_NOCTTY | O_ASYNC);
  if(-1 == conn->fd) {
    char barf[256];
    strerror_r(errno, barf, 256);
    fprintf(stderr, "(barobo) ERROR: open(): %s\n", barf);
    return -1;
  }

  /* Change the baud rate to 57600 */
  struct termios term;
  tcgetattr(conn->fd, &term);

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
  if(status = tcsetattr(conn->fd, TCSANOW, &term)) {
    char barf[256];
    strerror_r(errno, barf, 256);
    fprintf(stderr, "(barobo) ERROR: Configuring %s: %s.\n", address, barf);
  }
  tcgetattr(conn->fd, &term);
  if(cfgetispeed(&term) != BAUD) {
    fprintf(stderr, "(barobo) ERROR: Unable to set %s input speed.\n", address);
    exit(0);
  }
  if(cfgetospeed(&term) != BAUD) {
    fprintf(stderr, "(barobo) ERROR: Unable to set %s output speed.\n", address);
    exit(0);
  }

  tcflush(comms->socket, TCIOFLUSH);

  conn->status = MOBOT_CONNECTION_STATUS_UP;

  connectionDetectFraming(conn);

  return 0;
}
