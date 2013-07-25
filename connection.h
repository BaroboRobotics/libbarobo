#ifndef _BAROBO_CONNECTION_H_
#define _BAROBO_CONNECTION_H_

typedef enum MOBOTconnectionType {
  MOBOT_CONNECTION_TYPE_NONE,
  MOBOT_CONNECTION_TYPE_TTY,
  MOBOT_CONNECTION_TYPE_TCP,
  MOBOT_CONNECTION_TYPE_BLUETOOTH,
  MOBOT_CONNECTION_TYPE_ZIGBEE
} MOBOTconnectionType;

typedef enum MOBOTconnectionStatus {
  MOBOT_CONNECTION_STATUS_DOWN,
  MOBOT_CONNECTION_STATUS_UP
} MOBOTconnectionStatus;

typedef struct MOBOTconnection MOBOTconnection;

int Mobot_connectionInit (MOBOTconnection *conn);
int Mobot_connectionConnect (MOBOTconnection *conn, );
int Mobot_connectionDisconnect (MOBOTconnection *conn);
int Mobot_serviceInput

#endif
