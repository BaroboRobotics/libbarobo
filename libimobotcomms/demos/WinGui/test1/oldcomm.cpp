
#include <stdio.h>
#include <stdlib.h>
#include "imobotcomms.h"

#ifdef _WIN32
typedef struct bdaddr_s {
  UINT8 b[6];
} bdaddr_t;
int str2ba(const char *str, bdaddr_t *ba);
#define write(sock, buf, len) send(sock, buf, len, 0)
#define read(sock, buf, len) \
  recvfrom(sock, buf, len, 0, (struct sockaddr*)0, 0)
#endif

CiMobotComms::CiMobotComms()
{
  memset(&_addr, 0, sizeof(sockaddr_t));
  _connect_state = COMM_VOID;
#ifdef _WIN32
  WSADATA wsd;
  WSAStartup (MAKEWORD(1,1), &wsd);
#endif
}

int CiMobotComms::connectSlave(const char* address, int channel)
{
#ifndef _WIN32
  _socket = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
#else
  _socket = socket(AF_BTH, SOCK_STREAM, BTHPROTO_RFCOMM);
#endif
  if(_socket < 0) {
    printf("Error: Could not create socket. Error: %d\n", WSAGetLastError());
    return -1;
  }

  // set the connection parameters (who to connect to)
#ifndef _WIN32
  _addr.rc_family = AF_BLUETOOTH;
  _addr.rc_channel = (uint8_t) channel;
  str2ba( address, &_addr.rc_bdaddr );
#else
  _addr.addressFamily = AF_BTH ; 
  str2ba( address, (bdaddr_t*)&_addr.btAddr);
  _addr.port = channel & 0xFF;
#endif

  // connect to server
  int status = connect(_socket, (const sockaddr *)&_addr, sizeof(_addr));
  if(status == 0) {
    _connect_state = COMM_CONNECTED;
  } else {
      printf("Connect error: %d\n", WSAGetLastError());
  }
  return status;
}

int CiMobotComms::disconnect()
{
  if(_connect_state != COMM_VOID) {
#ifndef _WIN32
    close(_socket);
#else
    closesocket(_socket);
    CloseHandle((LPVOID)_socket);
#endif
    _connect_state = COMM_VOID;
  }
  return 0;
}

int CiMobotComms::pose(int m[5]) 
{
  char buf[160];
  int status;
  int bytes_read;
  sprintf(buf, "SET_POS %d %d %d %d %d", m[0], m[1], m[2], m[3], m[4]);
  status = write(_socket, buf, strlen(buf) + 1);
  if(status < 0) {return status;}
  bytes_read = read(_socket, buf, sizeof(buf));
  if(strcmp(buf, "OK")) { return -1; }

  sprintf(buf, "POSE %d", m[0]);
  status = write(_socket, buf, strlen(buf) + 1);
  bytes_read = read(_socket, buf, sizeof(buf));
  if(strcmp(buf, "OK")) { return -1; }

  return 0;
}

int CiMobotComms::gotopose(int poseno) 
{
  char buf[160];
  int status;
  int bytes_read;
  
  sprintf(buf, "POSE %d", poseno);
  status = write(_socket, buf, strlen(buf) + 1);
  bytes_read = read(_socket, buf, sizeof(buf));
  if(strcmp(buf, "OK")) { return -1; }

  return 0;

}

int CiMobotComms::move(int m[4])
{
  char buf[160];
  int status;
  int bytes_read;
  int _m[4] = {0,0,0,0};
  /* First get current encoder positions */
  sprintf(buf, "GET_POS");
  status = write(_socket, buf, strlen(buf)+1);
  if(status<0) {return status;}
  bytes_read = read(_socket, buf, sizeof(buf));
  if(bytes_read < 0) { return -1; }
  sscanf(buf, "%d %d %d %d", &_m[0], &_m[1], &_m[2], &_m[3]);

  /* Now adjust to desired motor encoder counts */
  int i;
  for(i = 0; i < 4; i++) {
    _m[i] += m[i];
  }

  sprintf(buf, "SET_POS 0 %d %d %d %d", _m[0], _m[1], _m[2], _m[3]);
  status = write(_socket, buf, strlen(buf)+1);
  if(status<0) {return status;}
  bytes_read = read(_socket, buf, sizeof(buf));
  if(bytes_read < 0) { return -1; }
  if(strcmp(buf, "OK")) { return -1; }

  sprintf(buf, "POSE 0");
  status = write(_socket, buf, strlen(buf)+1);
  if(status<0) {return status;}
  bytes_read = read(_socket, buf, sizeof(buf));
  if(bytes_read < 0) { return -1; }
  if(strcmp(buf, "OK")) { return -1; }

  return 0;
}


int CiMobotComms::stop()
{
  char buf[80];
  int status;
  int bytes_read;
  sprintf(buf, "STOP");
  status = write(_socket, buf, strlen(buf)+1);
  if(status<0) {return status;}
  bytes_read = read(_socket, buf, sizeof(buf));
  if(bytes_read < 0) { return -1; }
  if(!strcmp(buf, "ERROR")) {return -1;}
  return 0;
}

int CiMobotComms::play()
{
  char buf[80];
  int status;
  int bytes_read;
  sprintf(buf, "PLAY");
  status = write(_socket, buf, strlen(buf)+1);
  if(status<0) {return status;}
  bytes_read = read(_socket, buf, sizeof(buf));
  if(bytes_read < 0) { return -1; }
  if(!strcmp(buf, "ERROR")) {return -1;}
  return 0;
}

int CiMobotComms::pause()
{
  char buf[80];
  int status;
  int bytes_read;
  sprintf(buf, "PAUSE");
  status = write(_socket, buf, strlen(buf)+1);
  if(status<0) {return status;}
  bytes_read = read(_socket, buf, sizeof(buf));
  if(bytes_read < 0) { return -1; }
  if(!strcmp(buf, "ERROR")) {return -1;}
  return 0;
}

int CiMobotComms::quit()
{
  char buf[80];
  int status;
  int bytes_read;
  sprintf(buf, "QUIT");
  status = write(_socket, buf, strlen(buf)+1);
  if(status<0) {return status;}
  bytes_read = read(_socket, buf, sizeof(buf));
  if(bytes_read < 0) { return -1; }
  if(!strcmp(buf, "ERROR")) {return -1;}
  return 0;
}

int CiMobotComms::moveWait()
{
  char buf[160];
  int status;
  int bytes_read;
  int usec_wait = 50000;
  while(1) {
    sprintf(buf, "CHK");
    status = write(_socket, buf, strlen(buf) + 1);
    if(status<0) {return status;}
    bytes_read = read(_socket, buf, sizeof(buf));
    if(!strcmp(buf, "OK")) { break; }
    else {
#ifndef _WIN32
      usleep(usec_wait);
#else
      Sleep(usec_wait / 1000 );
#endif
    }
  }
  return 0;
}

int CiMobotComms::setJointPower(int motor, uint8_t speed)
{
  int status, bytes_read;
  char buf[160];
  sprintf(buf, "SET_MOTOR_SPEED %d %d", motor, speed);
  status = write(_socket, buf, strlen(buf) + 1);
  if(status<0) {return status;}
  bytes_read = read(_socket, buf, sizeof(buf));
  if(strcmp(buf, "OK")) { return -1; }
  return 0;
}

int CiMobotComms::getJointEncoderCount(int jointID)
{
  char buf[160];
  int status;
  int bytes_read;
  sprintf(buf, "GET_ENC %d", jointID);
  status = write(_socket, buf, strlen(buf)+1);
  if(status<0) {return status;}
  bytes_read = read(_socket, buf, sizeof(buf));
  if(bytes_read < 0) { return -1; }
  if(!strcmp(buf, "ERROR")) {return -1;}
  sscanf(buf, "%d", &status);
  return status;
}

int CiMobotComms::sendMessage(const char* message, char* reply, size_t sizeReply)
{
  int status;
  int bytes_read;
  status = write(_socket, message, strlen(message)+1);
  if(status<0) {return status;}
  bytes_read = read(_socket, reply, sizeReply);
  if(bytes_read < 0) { return -1; }
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
