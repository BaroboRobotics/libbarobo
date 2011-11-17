
#include <stdio.h>
#include <stdlib.h>
#include "mobot.h"
#include "mobot_internal.h"

#ifdef _CH_
#include <stdarg.h>
#endif


int Mobot_init(br_comms_t* comms)
{
  int i;
  memset(&comms->addr, 0, sizeof(sockaddr_t));
  comms->connected = 0;
#ifdef _WIN32
  WSADATA wsd;
  WSAStartup (MAKEWORD(1,1), &wsd);
#endif
  for(i = 0; i < 4; i++) {
    comms->jointSpeeds[i] = DEF_MOTOR_SPEED;
  }
  return 0;
}

int Mobot_connect(br_comms_t* comms)
{
#ifndef _WIN32
  fprintf(stderr, 
      "ERROR; Function Mobot_connect() is not yet implemented \n"
      "on non-Windows systems. Please use Mobot_connectWithAddress() \n"
      "instead.\n");
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
    comms->hSerial = CreateFileA(buf, 
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

    /* Send status request message to the mobot */
    DWORD bytes;
    if(!WriteFile(comms->hSerial, "GET_MOBOT_STATUS", strlen("GET_MOBOT_STATUS")+1, &bytes, NULL)) {
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
    if(strcmp(buf, "MOBOT READY")) {
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

int Mobot_connectWithAddress(br_comms_t* comms, const char* address, int channel)
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

int Mobot_disconnect(br_comms_t* comms)
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

int Mobot_isConnected(br_comms_t* comms)
{
  return comms->connected;
}

int Mobot_setJointDirection(br_comms_t* comms, mobotJointId_t id, mobotJointDirection_t dir)
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

int Mobot_getJointDirection(br_comms_t* comms, mobotJointId_t id, mobotJointDirection_t *dir)
{
  char buf[160];
  int status;
  int bytes_read;
  sprintf(buf, "GET_MOTOR_DIRECTION %d", id);
  status = SendToIMobot(comms, buf, strlen(buf)+1);
  if(status < 0) return status;
  bytes_read = RecvFromIMobot(comms, buf, sizeof(buf));
  if(!strcmp(buf, "ERROR")) return -1;
  sscanf(buf, "%d", (int*)dir);
  return 0;
}

int Mobot_setJointSpeed(br_comms_t* comms, mobotJointId_t id, double speed)
{
  char buf[160];
  int status;
  int bytes_read;
  sprintf(buf, "SET_MOTOR_SPEED %d %d", id, (int)(speed*100));
  status = SendToIMobot(comms, buf, strlen(buf)+1);
  if(status < 0) return status;
  bytes_read = RecvFromIMobot(comms, buf, sizeof(buf));
  if(strcmp(buf, "OK")) return -1;
  comms->jointSpeeds[id-1] = speed;
  return 0;
}

int Mobot_getJointSpeed(br_comms_t* comms, mobotJointId_t id, double *speed)
{
  char buf[160];
  int status;
  int bytes_read;
  int ispeed;
  sprintf(buf, "GET_MOTOR_SPEED %d", id);
  status = SendToIMobot(comms, buf, strlen(buf)+1);
  if(status < 0) return status;
  bytes_read = RecvFromIMobot(comms, buf, sizeof(buf));
  if(!strcmp(buf, "ERROR")) return -1;
  sscanf(buf, "%d", &ispeed);
  *speed = (double)ispeed/100.0;
  return 0;
}

int Mobot_getJointAngle(br_comms_t* comms, mobotJointId_t id, double *angle)
{
  char buf[160];
  int status;
  int bytes_read;
  sprintf(buf, "GET_MOTOR_POSITION %d", id);
  status = SendToIMobot(comms, buf, strlen(buf)+1);
  if(status < 0) return status;
  bytes_read = RecvFromIMobot(comms, buf, sizeof(buf));
  if(!strcmp(buf, "ERROR")) return -1;
  sscanf(buf, "%lf", angle);
  return 0;
}

int Mobot_getJointState(br_comms_t* comms, mobotJointId_t id, mobotJointState_t *state)
{
  char buf[160];
  int status;
  int bytes_read;
  sprintf(buf, "GET_MOTOR_STATE %d", id);
  status = SendToIMobot(comms, buf, strlen(buf)+1);
  if(status < 0) return status;
  bytes_read = RecvFromIMobot(comms, buf, sizeof(buf));
  if(!strcmp(buf, "ERROR")) return -1;
  sscanf(buf, "%d", (int*)state);
  return 0;
}

int Mobot_moveJointTo(br_comms_t* comms, mobotJointId_t id, double angle)
{
  char buf[160];
  int status;
  int bytes_read;
  sprintf(buf, "SET_MOTOR_POSITION %d %lf", id, angle);
  status = SendToIMobot(comms, buf, strlen(buf)+1);
  if(status < 0) return status;
  bytes_read = RecvFromIMobot(comms, buf, sizeof(buf));
  if(strcmp(buf, "OK")) return -1;
  /* Wait for the motion to finish */
  return Mobot_moveJointWait(comms, id);
}

int Mobot_moveJointToNB(br_comms_t* comms, mobotJointId_t id, double angle)
{
  char buf[160];
  int status;
  int bytes_read;
  sprintf(buf, "SET_MOTOR_POSITION %d %lf", id, angle);
  status = SendToIMobot(comms, buf, strlen(buf)+1);
  if(status < 0) return status;
  bytes_read = RecvFromIMobot(comms, buf, sizeof(buf));
  if(strcmp(buf, "OK")) return -1;
  return 0;
}

int Mobot_moveToZero(br_comms_t* comms)
{
  int i;
  for(i = 1; i < 5; i++) {
    if(Mobot_moveJointTo(comms, (mobotJointId_t)i, 0)) {
      return -1;
    }
  }
  /* Wait for the motion to finish */
  return Mobot_moveWait(comms);
}

int Mobot_moveToZeroNB(br_comms_t* comms)
{
  int i;
  for(i = 1; i < 5; i++) {
    if(Mobot_moveJointTo(comms, (mobotJointId_t)i, 0)) {
      return -1;
    }
  }
  return 0;
}

int Mobot_move(br_comms_t* comms,
                               double angle1,
                               double angle2,
                               double angle3,
                               double angle4)
{
  double angles[4];
  double curAngles[4];
  int i;
  angles[0] = angle1;
  angles[1] = angle2;
  angles[2] = angle3;
  angles[3] = angle4;
  for(i = 0; i < 4; i++) {
    if(Mobot_getJointAngle(comms, (mobotJointId_t)(i+1), &curAngles[i])) {
      return -1;
    }
  }
  for(i = 0; i < 4; i++) {
    if(angles[i] == 0) {
      continue;
    }
    if(Mobot_moveJointTo(comms, (mobotJointId_t)(i+1), angles[i] + curAngles[i])) {
      return -1;
    }
  }
  /* Wait for the motion to complete */
  return Mobot_moveWait(comms);
}

int Mobot_moveNB(br_comms_t* comms,
                               double angle1,
                               double angle2,
                               double angle3,
                               double angle4)
{
  double angles[4];
  double curAngles[4];
  int i;
  angles[0] = angle1;
  angles[1] = angle2;
  angles[2] = angle3;
  angles[3] = angle4;
  for(i = 0; i < 4; i++) {
    if(Mobot_getJointAngle(comms, (mobotJointId_t)(i+1), &curAngles[i])) {
      return -1;
    }
  }
  for(i = 0; i < 4; i++) {
    if(angles[i] == 0) {
      continue;
    }
    if(Mobot_moveJointTo(comms, (mobotJointId_t)(i+1), angles[i] + curAngles[i])) {
      return -1;
    }
  }
  return 0;
}

int Mobot_moveContinuousNB(br_comms_t* comms,
                                  mobotJointDirection_t dir1,
                                  mobotJointDirection_t dir2,
                                  mobotJointDirection_t dir3,
                                  mobotJointDirection_t dir4)
{
  mobotJointDirection_t dirs[4];
  int i;
  dirs[0] = dir1;
  dirs[1] = dir2;
  dirs[2] = dir3;
  dirs[3] = dir4;
  for(i = 0; i < 4; i++) {
    if(dirs[i] == MOBOT_FORWARD) {
      Mobot_setJointSpeed(comms, (mobotJointId_t)(i+1), comms->jointSpeeds[i]);
      Mobot_setJointDirection(comms, (mobotJointId_t)(i+1), MOBOT_FORWARD);
    } else if (dirs[i] == MOBOT_BACKWARD) {
      Mobot_setJointSpeed(comms, (mobotJointId_t)(i+1), comms->jointSpeeds[i]);
      Mobot_setJointDirection(comms, (mobotJointId_t)(i+1), MOBOT_BACKWARD);
    }
  }
  return 0;
}

int Mobot_moveContinuousTime(br_comms_t* comms,
                                  mobotJointDirection_t dir1,
                                  mobotJointDirection_t dir2,
                                  mobotJointDirection_t dir3,
                                  mobotJointDirection_t dir4,
                                  int msecs)
{
  int i;
  Mobot_moveContinuousNB(comms, dir1, dir2, dir3, dir4);
#ifdef _WIN32
  Sleep(msecs);
#else
  usleep(msecs * 1000);
#endif
  /* Stop the motors */
  for(i = 0; i < 4; i++) {
    Mobot_setJointDirection(comms, (mobotJointId_t)(i+1), MOBOT_NEUTRAL);
  }
  return 0;
}

int Mobot_moveTo(br_comms_t* comms,
                               double angle1,
                               double angle2,
                               double angle3,
                               double angle4)
{
  if(Mobot_moveJointTo(comms, MOBOT_JOINT1, angle1)) {
    return -1;
  }
  if(Mobot_moveJointTo(comms, MOBOT_JOINT2, angle2)) {
    return -1;
  }
  if(Mobot_moveJointTo(comms, MOBOT_JOINT3, angle3)) {
    return -1;
  }
  if(Mobot_moveJointTo(comms, MOBOT_JOINT4, angle4)) {
    return -1;
  }
  /* Wait for motion to finish */
  return Mobot_moveWait(comms);
}

int Mobot_moveToNB(br_comms_t* comms,
                               double angle1,
                               double angle2,
                               double angle3,
                               double angle4)
{
  if(Mobot_moveJointTo(comms, MOBOT_JOINT1, angle1)) {
    return -1;
  }
  if(Mobot_moveJointTo(comms, MOBOT_JOINT2, angle2)) {
    return -1;
  }
  if(Mobot_moveJointTo(comms, MOBOT_JOINT3, angle3)) {
    return -1;
  }
  if(Mobot_moveJointTo(comms, MOBOT_JOINT4, angle4)) {
    return -1;
  }
  return 0;
}

int Mobot_moveJointWait(br_comms_t* comms, mobotJointId_t id)
{
  mobotJointState_t state;
  /* Make sure there is no non-blocking function running */
  THREAD_JOIN(comms->thread);

  while(1)
  {
    if(Mobot_getJointState(comms, id, &state)) {
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
  return 0;
}

int Mobot_moveWait(br_comms_t* comms)
{
  int i;
  /* Make sure there is no non-blocking function running */
  THREAD_JOIN(comms->thread);
  for(i = 0; i < 4; i++) {
    if(Mobot_moveJointWait(comms, (mobotJointId_t)(i+1))) {
      return -1;
    }
  }
  return 0;
}

int Mobot_stop(br_comms_t* comms)
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

int Mobot_motionRollForward(br_comms_t* comms)
{
  double motorPosition[2];
  Mobot_getJointAngle(comms, MOBOT_JOINT3, &motorPosition[0]);
  Mobot_getJointAngle(comms, MOBOT_JOINT4, &motorPosition[1]);
  Mobot_moveJointTo(comms, MOBOT_JOINT3, motorPosition[0] + 90);
  Mobot_moveJointTo(comms, MOBOT_JOINT4, motorPosition[1] - 90);
  Mobot_moveWait(comms);
  return 0;
}

int Mobot_motionRollBackward(br_comms_t* comms)
{
  double motorPosition[2];
  Mobot_getJointAngle(comms, MOBOT_JOINT3, &motorPosition[0]);
  Mobot_getJointAngle(comms, MOBOT_JOINT4, &motorPosition[1]);
  Mobot_moveJointTo(comms, MOBOT_JOINT3, motorPosition[0] - 90);
  Mobot_moveJointTo(comms, MOBOT_JOINT4, motorPosition[1] + 90);
  Mobot_moveWait(comms);
  return 0;
}

int Mobot_motionTurnLeft(br_comms_t* comms)
{
  double motorPosition[2];
  Mobot_getJointAngle(comms, MOBOT_JOINT3, &motorPosition[0]);
  Mobot_getJointAngle(comms, MOBOT_JOINT4, &motorPosition[1]);
  Mobot_moveJointTo(comms, MOBOT_JOINT3, motorPosition[0] + 90);
  Mobot_moveJointTo(comms, MOBOT_JOINT4, motorPosition[1] + 90);
  Mobot_moveWait(comms);
  return 0;
}

int Mobot_motionTurnRight(br_comms_t* comms)
{
  double motorPosition[2];
  Mobot_getJointAngle(comms, MOBOT_JOINT3, &motorPosition[0]);
  Mobot_getJointAngle(comms, MOBOT_JOINT4, &motorPosition[1]);
  Mobot_moveJointTo(comms, MOBOT_JOINT3, motorPosition[0] - 90);
  Mobot_moveJointTo(comms, MOBOT_JOINT4, motorPosition[1] - 90);
  Mobot_moveWait(comms);
  return 0;
}

int Mobot_motionInchwormLeft(br_comms_t* comms)
{
  Mobot_moveJointTo(comms, MOBOT_JOINT1, 0);
  Mobot_moveJointTo(comms, MOBOT_JOINT2, 0);
  Mobot_moveWait(comms);

  Mobot_moveJointTo(comms, MOBOT_JOINT2, 50);
  Mobot_moveWait(comms);
  Mobot_moveJointTo(comms, MOBOT_JOINT1, -50);
  Mobot_moveWait(comms);
  Mobot_moveJointTo(comms, MOBOT_JOINT2, 0);
  Mobot_moveWait(comms);
  Mobot_moveJointTo(comms, MOBOT_JOINT1, 0);
  Mobot_moveWait(comms);

  return 0;
}

int Mobot_motionInchwormRight(br_comms_t* comms)
{
  Mobot_moveJointTo(comms, MOBOT_JOINT1, 0);
  Mobot_moveJointTo(comms, MOBOT_JOINT2, 0);
  Mobot_moveWait(comms);

  Mobot_moveJointTo(comms, MOBOT_JOINT1, -50);
  Mobot_moveWait(comms);
  Mobot_moveJointTo(comms, MOBOT_JOINT2, 50);
  Mobot_moveWait(comms);
  Mobot_moveJointTo(comms, MOBOT_JOINT1, 0);
  Mobot_moveWait(comms);
  Mobot_moveJointTo(comms, MOBOT_JOINT2, 0);
  Mobot_moveWait(comms);

  return 0;
}

int Mobot_motionStand(br_comms_t* comms)
{
  Mobot_moveToZero(comms);
  Mobot_moveWait(comms);
  Mobot_moveJointTo(comms, MOBOT_JOINT1, -85);
  Mobot_moveJointTo(comms, MOBOT_JOINT2, 80);
  Mobot_moveWait(comms);
  Mobot_moveJointTo(comms, MOBOT_JOINT3, 45);
  Mobot_moveWait(comms);
  Mobot_moveJointTo(comms, MOBOT_JOINT1, 20);
  Mobot_moveWait(comms);
  return 0;
}

#ifndef _WIN32
void* motionInchwormLeftThread(void* arg)
#else
DWORD WINAPI motionInchwormLeftThread(LPVOID arg)
#endif
{
  br_comms_t* comms = (br_comms_t*)arg;
  Mobot_motionInchwormLeft(comms);
  return NULL;
}

int Mobot_motionInchwormLeftNB(br_comms_t* comms)
{
  /* Make sure the old thread has joined */
  THREAD_JOIN(comms->thread);
  THREAD_CREATE(&comms->thread, motionInchwormLeftThread, comms);
  return 0;
}

#ifndef _WIN32
void* motionInchwormRightThread(void* arg)
#else
DWORD WINAPI motionInchwormRightThread(LPVOID arg)
#endif
{
  Mobot_motionInchwormRight((br_comms_t*)arg);
  return NULL;
}

int Mobot_motionInchwormRightNB(br_comms_t* comms)
{
  THREAD_JOIN(comms->thread);
  THREAD_CREATE(&comms->thread, motionInchwormRightThread, comms);
  return 0;
}

#ifndef _WIN32
void* motionRollBackwardThread(void* arg)
#else
DWORD WINAPI motionRollBackwardThread(LPVOID arg)
#endif
{
  Mobot_motionRollBackward((br_comms_t*)arg);
  return NULL;
}

int Mobot_motionRollBackwardNB(br_comms_t* comms)
{
  THREAD_JOIN(comms->thread);
  THREAD_CREATE(&comms->thread, motionRollBackwardThread, comms);
  return 0;
}

#ifndef _WIN32
void* motionRollForwardThread(void* arg)
#else
DWORD WINAPI motionRollForwardThread(LPVOID arg)
#endif
{
  Mobot_motionRollForward((br_comms_t*)arg);
  return NULL;
}

int Mobot_motionRollForwardNB(br_comms_t* comms)
{
  THREAD_JOIN(comms->thread);
  THREAD_CREATE(&comms->thread, motionRollForwardThread, comms);
  return 0;
}

#ifndef _WIN32
void* motionStandThread(void* arg)
#else
DWORD WINAPI motionStandThread(LPVOID arg)
#endif
{
  Mobot_motionStand((br_comms_t*)arg);
  return NULL;
}

int Mobot_motionStandNB(br_comms_t* comms)
{
  THREAD_JOIN(comms->thread);
  THREAD_CREATE(&comms->thread, motionStandThread, comms);
  return 0;
}

#ifndef _WIN32
void* motionTurnLeftThread(void* arg)
#else
DWORD WINAPI motionTurnLeftThread(LPVOID arg)
#endif
{
  Mobot_motionTurnLeft((br_comms_t*)arg);
  return NULL;
}

int Mobot_motionTurnLeftNB(br_comms_t* comms)
{
  THREAD_JOIN(comms->thread);
  THREAD_CREATE(&comms->thread, motionTurnLeftThread, comms);
  return 0;
}

#ifndef _WIN32
void* motionTurnRightThread(void* arg)
#else
DWORD WINAPI motionTurnRightThread(LPVOID arg)
#endif
{
  Mobot_motionTurnRight((br_comms_t*)arg);
  return NULL;
}

int Mobot_motionTurnRightNB(br_comms_t* comms)
{
  THREAD_JOIN(comms->thread);
  THREAD_CREATE(&comms->thread, motionTurnRightThread, comms);
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
#ifdef _WIN32
	return send(comms->socket, str, len, 0);
#else
    return write(comms->socket, str, len);
#endif
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
#ifdef _WIN32
	return recvfrom(comms->socket, buf, size, 0, (struct sockaddr*)0, 0);
#else
    return read(comms->socket, buf, size);
#endif
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

CMobot::CMobot()
{
  Mobot_init(&_comms);
}

CMobot::~CMobot()
{
}

int CMobot::connect()
{
  return Mobot_connect(&_comms);
}

int CMobot::connectWithAddress(const char* address, int channel)
{
  return Mobot_connectWithAddress(&_comms, address, channel);
}

int CMobot::disconnect()
{
  return Mobot_disconnect(&_comms);
}

int CMobot::isConnected()
{
  return Mobot_isConnected(&_comms);
}

int CMobot::setJointDirection(mobotJointId_t id, mobotJointDirection_t dir)
{
  return Mobot_setJointDirection(&_comms, id, dir);
}

int CMobot::getJointDirection(mobotJointId_t id, mobotJointDirection_t &dir)
{
  return Mobot_getJointDirection(&_comms, id, &dir);
}

int CMobot::setJointSpeed(mobotJointId_t id, double speed)
{
  return Mobot_setJointSpeed(&_comms, id, speed);
}

int CMobot::getJointSpeed(mobotJointId_t id, double &speed)
{
  return Mobot_getJointSpeed(&_comms, id, &speed);
}

int CMobot::moveJointTo(mobotJointId_t id, double angle)
{
  return Mobot_moveJointTo(&_comms, id, angle);
}

int CMobot::moveJointToNB(mobotJointId_t id, double angle)
{
  return Mobot_moveJointToNB(&_comms, id, angle);
}

int CMobot::getJointAngle(mobotJointId_t id, double &angle)
{
  return Mobot_getJointAngle(&_comms, id, &angle);
}

int CMobot::getJointState(mobotJointId_t id, mobotJointState_t &state)
{
  return Mobot_getJointState(&_comms, id, &state);
}

int CMobot::move( double angle1,
                        double angle2,
                        double angle3,
                        double angle4)
{
  return Mobot_move(&_comms, angle1, angle2, angle3, angle4);
}

int CMobot::moveNB( double angle1,
                        double angle2,
                        double angle3,
                        double angle4)
{
  return Mobot_moveNB(&_comms, angle1, angle2, angle3, angle4);
}

int CMobot::moveContinuousNB( mobotJointDirection_t dir1, mobotJointDirection_t dir2, mobotJointDirection_t dir3, mobotJointDirection_t dir4)
{
  return Mobot_moveContinuousNB(&_comms, dir1, dir2, dir3, dir4);
}

int CMobot::moveContinuousTime( mobotJointDirection_t dir1, mobotJointDirection_t dir2, mobotJointDirection_t dir3, mobotJointDirection_t dir4, int msecs)
{
  return Mobot_moveContinuousTime(&_comms, dir1, dir2, dir3, dir4, msecs);
}

int CMobot::moveTo( double angle1,
                          double angle2,
                          double angle3,
                          double angle4)
{
  return Mobot_moveTo(&_comms, angle1, angle2, angle3, angle4);
}

int CMobot::moveToNB( double angle1,
                          double angle2,
                          double angle3,
                          double angle4)
{
  return Mobot_moveToNB(&_comms, angle1, angle2, angle3, angle4);
}

int CMobot::moveToZero()
{
  return Mobot_moveToZero(&_comms);
}

int CMobot::moveToZeroNB()
{
  return Mobot_moveToZeroNB(&_comms);
}

int CMobot::moveJointWait(mobotJointId_t id)
{
  return Mobot_moveJointWait(&_comms, id);
}

int CMobot::moveWait()
{
  return Mobot_moveWait(&_comms);
}

int CMobot::stop()
{
  return Mobot_stop(&_comms);
}

int CMobot::motionRollForward()
{
  return Mobot_motionRollForward(&_comms);
}

int CMobot::motionRollBackward()
{
  return Mobot_motionRollBackward(&_comms);
}

int CMobot::motionTurnLeft()
{
  return Mobot_motionTurnLeft(&_comms);
}

int CMobot::motionTurnRight()
{
  return Mobot_motionTurnRight(&_comms);
}

int CMobot::motionInchwormLeft()
{
  return Mobot_motionInchwormLeft(&_comms);
}

int CMobot::motionInchwormRight()
{
  return Mobot_motionInchwormRight(&_comms);
}

int CMobot::motionStand()
{
  return Mobot_motionStand(&_comms);
}

int CMobot::motionInchwormLeftNB()
{
  return Mobot_motionInchwormLeftNB(&_comms);
}

int CMobot::motionInchwormRightNB()
{
  return Mobot_motionInchwormRightNB(&_comms);
}

int CMobot::motionRollBackwardNB()
{
  return Mobot_motionRollBackwardNB(&_comms);
}

int CMobot::motionRollForwardNB()
{
  return Mobot_motionRollForwardNB(&_comms);
}

int CMobot::motionStandNB()
{
  return Mobot_motionStandNB(&_comms);
}

int CMobot::motionTurnLeftNB()
{
  return Mobot_motionTurnLeftNB(&_comms);
}

int CMobot::motionTurnRightNB()
{
  return Mobot_motionTurnRightNB(&_comms);
}

#endif
