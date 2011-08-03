
#include <stdio.h>
#include <stdlib.h>
#include "imobotcomms.h"


int iMobotComms_init(br_comms_t* comms)
{
  memset(&comms->addr, 0, sizeof(sockaddr_t));
  comms->connected = 0;
#ifdef _WIN32
  WSADATA wsd;
  WSAStartup (MAKEWORD(1,1), &wsd);
#endif
  return 0;
}

int iMobotComms_connect(br_comms_t* comms)
{
#ifndef _WIN32
  fprintf(stderr, 
      "ERROR; Function iMobotComms_connect() is not yet implemented "
      "on non-Windows systems. Please use iMobotComms_connectAddress() "
      "instead.");
  return -1;
#else
  char buf[80];

/* NUM_PORTS indicates the number of Windows COM ports to check for connections
 * to the robot. */
#define NUM_PORTS 30
  int i;
  for(i = 1; i < NUM_PORTS; i++) {
    sprintf(buf, "\\\\.\\COM%d", i);
    printf("Trying %s...\n", buf);
    comms->hSerial = CreateFile(buf, 
        GENERIC_READ | GENERIC_WRITE,
        0,
        0,
        OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL,
        0);
    if(comms->hSerial == INVALID_HANDLE_VALUE) {
      printf("Could not open serial handle.\n");
      continue;
    }
    /* Set up the properties */
    DCB dcbSerialParams = {0};
    dcbSerialParams.DCBlength=sizeof(dcbSerialParams);
    if(!GetCommState(comms->hSerial, &dcbSerialParams)) {
      // error getting state
      printf("Could not get serial properties.\n");
      continue;
    }
    dcbSerialParams.BaudRate = CBR_115200;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;
    if(!SetCommState(comms->hSerial, &dcbSerialParams)) {
      // error setting state
      printf("Could not set serial properties.\n");
      continue;
    }

    /* Set up timeouts */
    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutConstant = 50;
    timeouts.ReadTotalTimeoutMultiplier = 10;
    timeouts.WriteTotalTimeoutConstant = 50;
    timeouts.WriteTotalTimeoutMultiplier = 10;
    if(!SetCommTimeouts(comms->hSerial, &timeouts)) {
      // Error setting timeouts
      printf("Could not set timeouts.\n");
      continue;
    }

    /* Send status request message to the iMobot */
    DWORD bytes;
    if(!WriteFile(comms->hSerial, "GET_IMOBOT_STATUS", strlen("GET_IMOBOT_STATUS")+1, &bytes, NULL)) {
      printf("Could not send status request message.\n");
      CloseHandle(comms->hSerial);
      continue;
    }
    printf("Wrote %d bytes.\n", bytes);
    Sleep(250);
    /* Check response message */
    if(!ReadFile(comms->hSerial, buf, 79, &bytes, NULL)) {
      printf("Could not receive response message.\n");
      CloseHandle(comms->hSerial);
      continue;
    }
    if(strcmp(buf, "IMOBOT READY")) {
      printf("Incorrect response message: %s.\n", buf);
      CloseHandle(comms->hSerial);
      continue;
    } else {
      comms->connected = 2;
      break;
    }
  }
  /* At this point, we _should_ be connected */
  if(i != NUM_PORTS) {
    return 0;
  } else {
    return -1;
  }
#endif
}

int iMobotComms_connectAddress(br_comms_t* comms, const char* address, int channel)
{
  int status;
  comms->socket = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

  // set the connection parameters (who to connect to)
#ifndef _WIN32
  comms->addr.rc_family = AF_BLUETOOTH;
  comms->addr.rc_channel = (uint8_t) channel;
  str2ba( address, &comms->addr.rc_bdaddr );
#else
  comms->addr.addressFamily = AF_BTH;
  str2ba( address, (bdaddr_t*)&comms->addr.btAddr);
  comms->addr.port = channel;
#endif

  // connect to server
  status = connect(comms->socket, (const struct sockaddr *)&comms->addr, sizeof(comms->addr));
  if(status == 0) {
    comms->connected = 1;
  }
  return status;
}

int iMobotComms_disconnect(br_comms_t* comms)
{
#ifndef _WIN32
  close(comms->socket);
#else
  closesocket(comms->socket);
  CloseHandle((LPVOID)comms->socket);
#endif
  comms->connected = 0;
  return 0;
}

int iMobotComms_isConnected(br_comms_t* comms)
{
  return comms->connected;
}

int iMobotComms_setMotorDirection(br_comms_t* comms, int id, int dir)
{
  char buf[160];
  int status;
  int bytes_read;
  sprintf(buf, "SET_MOTOR_DIRECTION %d %d", id, dir);
  status = SendToIMobot(comms, buf, strlen(buf)+1);
  if(status < 0) return status;
  bytes_read = RecvFromIMobot(comms, buf, sizeof(buf));
  if(strcmp(buf, "OK")) return -1;
  return 0;
}

int iMobotComms_getMotorDirection(br_comms_t* comms, int id, int *dir)
{
  char buf[160];
  int status;
  int bytes_read;
  sprintf(buf, "GET_MOTOR_DIRECTION %d", id);
  status = SendToIMobot(comms, buf, strlen(buf)+1);
  if(status < 0) return status;
  bytes_read = RecvFromIMobot(comms, buf, sizeof(buf));
  if(!strcmp(buf, "ERROR")) return -1;
  sscanf(buf, "%d", dir);
  return 0;
}

int iMobotComms_setMotorSpeed(br_comms_t* comms, int id, int speed)
{
  char buf[160];
  int status;
  int bytes_read;
  sprintf(buf, "SET_MOTOR_SPEED %d %d", id, speed);
  status = SendToIMobot(comms, buf, strlen(buf)+1);
  if(status < 0) return status;
  bytes_read = RecvFromIMobot(comms, buf, sizeof(buf));
  if(strcmp(buf, "OK")) return -1;
  return 0;
}

int iMobotComms_getMotorSpeed(br_comms_t* comms, int id, int *speed)
{
  char buf[160];
  int status;
  int bytes_read;
  sprintf(buf, "GET_MOTOR_SPEED %d", id);
  status = SendToIMobot(comms, buf, strlen(buf)+1);
  if(status < 0) return status;
  bytes_read = RecvFromIMobot(comms, buf, sizeof(buf));
  if(!strcmp(buf, "ERROR")) return -1;
  sscanf(buf, "%d", speed);
  return 0;
}

int iMobotComms_setMotorPosition(br_comms_t* comms, int id, double position)
{
  char buf[160];
  int status;
  int bytes_read;
  sprintf(buf, "SET_MOTOR_POSITION %d %lf", id, position);
  status = SendToIMobot(comms, buf, strlen(buf)+1);
  if(status < 0) return status;
  bytes_read = RecvFromIMobot(comms, buf, sizeof(buf));
  if(strcmp(buf, "OK")) return -1;
  return 0;
}

int iMobotComms_getMotorPosition(br_comms_t* comms, int id, double *position)
{
  char buf[160];
  int status;
  int bytes_read;
  sprintf(buf, "GET_MOTOR_POSITION %d", id);
  status = SendToIMobot(comms, buf, strlen(buf)+1);
  if(status < 0) return status;
  bytes_read = RecvFromIMobot(comms, buf, sizeof(buf));
  if(!strcmp(buf, "ERROR")) return -1;
  sscanf(buf, "%lf", position);
  return 0;
}

int iMobotComms_getMotorState(br_comms_t* comms, int id, int *state)
{
  char buf[160];
  int status;
  int bytes_read;
  sprintf(buf, "GET_MOTOR_STATE %d", id);
  status = SendToIMobot(comms, buf, strlen(buf)+1);
  if(status < 0) return status;
  bytes_read = RecvFromIMobot(comms, buf, sizeof(buf));
  if(!strcmp(buf, "ERROR")) return -1;
  sscanf(buf, "%d", state);
  return 0;
}

int iMobotComms_poseZero(br_comms_t* comms)
{
  int i;
  for(i = 0; i < 4; i++) {
    if(iMobotComms_setMotorPosition(comms, i, 0)) {
      return -1;
    }
  }
  return 0;
}

int iMobotComms_waitMotor(br_comms_t* comms, int id)
{
  int state;
  while(1)
  {
    if(iMobotComms_getMotorState(comms, id, &state)) {
      return -1;
    }
    if(state == 0) {
      return 0;
    } else {
#ifndef _WIN32
      usleep(200000);
#else
      Sleep(200);
#endif
    }
  }
}

int iMobotComms_moveWait(br_comms_t* comms)
{
  int i;
  for(i = 0; i < 4; i++) {
    if(iMobotComms_waitMotor(comms, i)) {
      return -1;
    }
  }
  return 0;
}

int iMobotComms_stop(br_comms_t* comms)
{
  char buf[160];
  int status;
  int bytes_read;
  status = SendToIMobot(comms, "STOP", 5);
  if(status < 0) return status;
  bytes_read = RecvFromIMobot(comms, buf, sizeof(buf));
  if(!strcmp(buf, "ERROR")) return -1;
  return 0;
}

#ifdef _WIN32
void baswap(bdaddr_t *dst, const bdaddr_t *src)
{
	register unsigned char *d = (unsigned char *) dst;
	register const unsigned char *s = (const unsigned char *) src;
	register int i;

	for (i = 0; i < 6; i++)
		d[i] = s[5-i];
}

int str2ba(const char *str, bdaddr_t *ba)
{
	UINT8 b[6];
	const char *ptr = str;
	int i;

	for (i = 0; i < 6; i++) {
		b[i] = (UINT8) strtol(ptr, NULL, 16);
		if (i != 5 && !(ptr = strchr(ptr, ':')))
			ptr = ":00:00:00:00:00";
		ptr++;
	}

	baswap(ba, (bdaddr_t *) b);

	return 0;
}
#endif

int SendToIMobot(br_comms_t* comms, const char* str, int len)
{
  if(comms->connected == 1) {
    return write(comms->socket, str, len);
  } else if (comms->connected == 2) {
#ifdef _WIN32
    DWORD bytes;
    if(!WriteFile(comms->hSerial, str, len, &bytes, NULL)) {
      return -1;
    }
#else
    return -1;
#endif
  } else {
    return -1;
  }
  return 0;
}

int RecvFromIMobot(br_comms_t* comms, char* buf, int size)
{
#ifdef _WIN32
  DWORD bytes;
#endif
  if(comms->connected == 1) {
    return read(comms->socket, buf, size);
  } else if (comms->connected == 2) {
#ifdef _WIN32
    if(!ReadFile(comms->hSerial, buf, size, &bytes, NULL)) {
      return -1;
    }
    return bytes;
#else
    return -1;
#endif
  } else {
    return -1;
  }
}

#ifndef C_ONLY

CiMobotComms::CiMobotComms()
{
  iMobotComms_init(&_comms);
}

CiMobotComms::~CiMobotComms()
{
}


int CiMobotComms::connect()
{
  return iMobotComms_connect(&_comms);
}

int CiMobotComms::connectAddress(const char* address, int channel)
{
  return iMobotComms_connectAddress(&_comms, address, channel);
}

int CiMobotComms::disconnect()
{
  return iMobotComms_disconnect(&_comms);
}

int CiMobotComms::isConnected()
{
  return iMobotComms_isConnected(&_comms);
}

int CiMobotComms::setMotorDirection(int id, int dir)
{
  return iMobotComms_setMotorDirection(&_comms, id, dir);
}

int CiMobotComms::getMotorDirection(int id, int &dir)
{
  return iMobotComms_getMotorDirection(&_comms, id, &dir);
}

int CiMobotComms::setMotorSpeed(int id, int speed)
{
  return iMobotComms_setMotorSpeed(&_comms, id, speed);
}

int CiMobotComms::getMotorSpeed(int id, int &speed)
{
  return iMobotComms_getMotorSpeed(&_comms, id, &speed);
}

int CiMobotComms::setMotorPosition(int id, double position)
{
  return iMobotComms_setMotorPosition(&_comms, id, position);
}

int CiMobotComms::getMotorPosition(int id, double &position)
{
  return iMobotComms_getMotorPosition(&_comms, id, &position);
}

int CiMobotComms::getMotorState(int id, int &state)
{
  return iMobotComms_getMotorState(&_comms, id, &state);
}

int CiMobotComms::poseZero()
{
  return iMobotComms_poseZero(&_comms);
}

int CiMobotComms::waitMotor(int id)
{
  return iMobotComms_waitMotor(&_comms, id);
}

int CiMobotComms::moveWait()
{
  return iMobotComms_moveWait(&_comms);
}

int CiMobotComms::stop()
{
  return iMobotComms_stop(&_comms);
}
#endif
