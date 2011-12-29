
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "mobot.h"
#include "mobot_internal.h"
#ifndef _WIN32
#include <unistd.h>
#include <fcntl.h>
#else
#include <windows.h>
#include <shlobj.h>
#endif

#ifdef _CH_
#include <stdarg.h>
#endif

int g_numConnected = 0;

double deg2rad(double deg)
{
  return deg * M_PI / 180.0;
}

double rad2deg(double rad)
{
  return rad * 180.0 / M_PI;
}

void* nullThread(void* arg)
{ 
  return NULL;
}

int Mobot_init(br_comms_t* comms)
{
  int i;
  memset(&comms->addr, 0, sizeof(sockaddr_t));
  comms->connected = 0;
#ifdef _WIN32
  printf("Starting WSA...\n");
  WSADATA wsd;
  if(WSAStartup (MAKEWORD(2,2), &wsd) != 0) {
    printf("WSAStartup failed with error %d\n", WSAGetLastError());
  }
#endif
  for(i = 0; i < 4; i++) {
    comms->jointSpeeds[i] = DEF_MOTOR_SPEED;
  }
  THREAD_CREATE(&comms->thread, nullThread, NULL);
  return 0;
}

int finishConnect(br_comms_t* comms);
int Mobot_connect(br_comms_t* comms)
{
#ifdef _WIN32
  /* Find the user's local appdata directory */
  int i;
  FILE *fp;
  char path[MAX_PATH];
  if(SHGetFolderPathA(NULL, CSIDL_LOCAL_APPDATA, NULL, 0, path) != S_OK) 
  {
    /* Could not get the user's app data directory */
  } else {
    //MessageBox((LPCTSTR)path, (LPCTSTR)"Test");
    //fprintf(fp, "%s", path); 
  }
  strcat(path, "\\Barobo.config");
  fp = fopen(path, "r");
  if(fp == NULL) {
    /* The file doesn't exist. Gotta make the file */
    fprintf(stderr, 
        "ERROR: Your Barobo configuration file does not exist.\n"
        "Please create one by opening the MoBot remote control, clicking on\n"
        "the 'Robot' menu entry, and selecting 'Configure Robot Bluetooth'.\n");
    return -1;
  }
  /* Read the correct line */
  for(i = 0; i < g_numConnected+1; i++) {
    if(fgets(path, MAX_PATH, fp) == NULL) {
      return -1;
    }
  }
  /* Get rid of trailing newline and/or carriage return */
  while(
      (path[strlen(path)-1] == '\r' ) ||
      (path[strlen(path)-1] == '\n' ) 
      )
  {
    path[strlen(path)-1] = '\0';
  }
  /* Pass it on to connectWithAddress() */
  if(i = Mobot_connectWithAddress(comms, path, 1)) {
    return i;
  } else {
    g_numConnected++;
    return i;
  }
#else
  return -1;
#endif
}
int Mobot_connect_old(br_comms_t* comms)
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
    dcbSerialParams.BaudRate = CBR_57600;
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

    /* Send status request message to the robot*/
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
    finishConnect(comms);
    return 0;
  } else {
    return -1;
  }
#endif
}

int Mobot_connectWithAddress(br_comms_t* comms, const char* address, int channel)
{
  int status;
  int flags;
  char buf[256];
#ifndef _WIN32
  comms->socket = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
#else
  comms->socket = socket(AF_BTH, SOCK_STREAM, BTHPROTO_RFCOMM);
#endif

#ifdef _WIN32
  if(comms->socket == INVALID_SOCKET) {
    printf("Could not bind to socket. Error %d\n", WSAGetLastError());
    return -1;
  }
#endif

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
#ifndef _WIN32
  /* Make the socket non-blocking */
  flags = fcntl(comms->socket, F_GETFL, 0);
  fcntl(comms->socket, F_SETFL, flags | O_NONBLOCK);
  /* Wait for the MoBot to get ready */
  sleep(1);
  read(comms->socket, buf, 255);
  finishConnect(comms);
#endif
  return status;
}

/* finishConnect():
 * Perform final connecting tasks common to all connection methods */
int finishConnect(br_comms_t* comms)
{
  int i;
  char buf[256];
  while(1) {
    SendToIMobot(comms, "GET_IMOBOT_STATUS", strlen("GET_IMOBOT_STATUS")+1);
    RecvFromIMobot(comms, buf, 255);
    if(strcmp(buf, "IMOBOT READY")==0) {
      break;
    }
  }
  /* Get the joint max speeds */
  for(i = 4; i >= 1; i--) {
    Mobot_getJointMaxSpeed(comms, (robotJointId_t)i, &(comms->maxSpeed[i-1]));
  }
  return 0;
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

int Mobot_isMoving(br_comms_t* comms)
{
  int moving = 0;
  robotJointState_t state;
  int i;
  for(i = 1; i <= 4; i++) {
    Mobot_getJointState(comms, (robotJointId_t)i, &state);
    if(state != ROBOT_JOINT_IDLE) {
      moving = 1;
      break;
    }
  }
  return moving;
}

int Mobot_setJointDirection(br_comms_t* comms, robotJointId_t id, robotJointDirection_t dir)
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

int Mobot_getJointDirection(br_comms_t* comms, robotJointId_t id, robotJointDirection_t *dir)
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

int Mobot_getJointMaxSpeed(br_comms_t* comms, robotJointId_t id, double *maxSpeed)
{
  char buf[160];
  int status;
  int bytes_read;
  sprintf(buf, "GET_JOINT_MAX_SPEED %d", id);
  status = SendToIMobot(comms, buf, strlen(buf)+1);
  if(status < 0) return status;
  bytes_read = RecvFromIMobot(comms, buf, sizeof(buf));
  if(!strcmp(buf, "ERROR")) return -1;
  sscanf(buf, "%lf", maxSpeed);
  comms->maxSpeed[(int)id-1] = *maxSpeed;
  return 0;
}

int Mobot_setJointSpeed(br_comms_t* comms, robotJointId_t id, double speed)
{
  char buf[160];
  int status;
  int bytes_read;
  sprintf(buf, "SET_MOTOR_SPEED %d %lf", id, speed);
  status = SendToIMobot(comms, buf, strlen(buf)+1);
  if(status < 0) return status;
  bytes_read = RecvFromIMobot(comms, buf, sizeof(buf));
  if(strcmp(buf, "OK")) return -1;
  comms->jointSpeeds[id-1] = speed;
  return 0;
}

int Mobot_setJointSpeedRatio(br_comms_t* comms, robotJointId_t id, double ratio)
{
  if((ratio < 0) || (ratio > 1)) {
    return -1;
  }
  return Mobot_setJointSpeed(comms, id, ratio * comms->maxSpeed[(int)id-1]);
}

int Mobot_getJointSpeed(br_comms_t* comms, robotJointId_t id, double *speed)
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
  sscanf(buf, "%lf", speed);
  return 0;
}

int Mobot_getJointSpeedRatio(br_comms_t* comms, robotJointId_t id, double *ratio)
{
  double speed;
  Mobot_getJointSpeed(comms, id, &speed);
  *ratio = speed / comms->maxSpeed[(int)id-1];
  return 0;
}

int Mobot_getJointAngle(br_comms_t* comms, robotJointId_t id, double *angle)
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

int Mobot_getJointState(br_comms_t* comms, robotJointId_t id, robotJointState_t *state)
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

int Mobot_getJointSpeeds(br_comms_t* comms, double speeds[4])
{
  for(int i = 0; i < 4; i++) {
    Mobot_getJointSpeed(comms, (robotJointId_t)(i+1), &speeds[i]);
  }
  return 0;
}

int Mobot_getJointSpeedRatios(br_comms_t* comms, double ratios[4])
{
  for(int i = 0; i < 4; i++) {
    Mobot_getJointSpeedRatio(comms, (robotJointId_t)(i+1), &ratios[i]);
  }
  return 0;
}

int Mobot_moveJointContinuousNB(br_comms_t* comms, robotJointId_t id, robotJointDirection_t dir)
{
  Mobot_setJointSpeed(comms, id, comms->jointSpeeds[(int)id-1]);
  Mobot_setJointDirection(comms, id, dir);
  return 0;
}

int Mobot_moveJointContinuousTime(br_comms_t* comms, robotJointId_t id, robotJointDirection_t dir, int msecs)
{
  Mobot_moveJointContinuousNB(comms, id, dir);
#ifdef _WIN32
  Sleep(msecs);
#else
  usleep(msecs * 1000);
#endif
  return 0;
}

int Mobot_moveJointTo(br_comms_t* comms, robotJointId_t id, double angle)
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

int Mobot_moveJointToPIDNB(br_comms_t* comms, robotJointId_t id, double angle)
{
  char buf[160];
  int status;
  int bytes_read;
  sprintf(buf, "SET_MOTOR_POSITION_PID %d %lf", id, angle);
  status = SendToIMobot(comms, buf, strlen(buf)+1);
  if(status < 0) return status;
  bytes_read = RecvFromIMobot(comms, buf, sizeof(buf));
  if(strcmp(buf, "OK")) return -1;
  return 0;
}

int Mobot_moveJointToNB(br_comms_t* comms, robotJointId_t id, double angle)
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
    if(Mobot_moveJointTo(comms, (robotJointId_t)i, 0)) {
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
    if(Mobot_moveJointTo(comms, (robotJointId_t)i, 0)) {
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
    if(Mobot_getJointAngle(comms, (robotJointId_t)(i+1), &curAngles[i])) {
      return -1;
    }
  }
  for(i = 0; i < 4; i++) {
    if(angles[i] == 0) {
      continue;
    }
    if(Mobot_moveJointTo(comms, (robotJointId_t)(i+1), angles[i] + curAngles[i])) {
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
    if(Mobot_getJointAngle(comms, (robotJointId_t)(i+1), &curAngles[i])) {
      return -1;
    }
  }
  for(i = 0; i < 4; i++) {
    if(angles[i] == 0) {
      continue;
    }
    if(Mobot_moveJointTo(comms, (robotJointId_t)(i+1), angles[i] + curAngles[i])) {
      return -1;
    }
  }
  return 0;
}

int Mobot_moveContinuousNB(br_comms_t* comms,
                                  robotJointDirection_t dir1,
                                  robotJointDirection_t dir2,
                                  robotJointDirection_t dir3,
                                  robotJointDirection_t dir4)
{
  robotJointDirection_t dirs[4];
  int i;
  dirs[0] = dir1;
  dirs[1] = dir2;
  dirs[2] = dir3;
  dirs[3] = dir4;
  for(i = 0; i < 4; i++) {
    if(dirs[i] == ROBOT_FORWARD) {
      Mobot_setJointSpeed(comms, (robotJointId_t)(i+1), comms->jointSpeeds[i]);
      Mobot_setJointDirection(comms, (robotJointId_t)(i+1), ROBOT_FORWARD);
    } else if (dirs[i] == ROBOT_BACKWARD) {
      Mobot_setJointSpeed(comms, (robotJointId_t)(i+1), comms->jointSpeeds[i]);
      Mobot_setJointDirection(comms, (robotJointId_t)(i+1), ROBOT_BACKWARD);
    }
  }
  return 0;
}

int Mobot_moveContinuousTime(br_comms_t* comms,
                                  robotJointDirection_t dir1,
                                  robotJointDirection_t dir2,
                                  robotJointDirection_t dir3,
                                  robotJointDirection_t dir4,
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
    Mobot_setJointDirection(comms, (robotJointId_t)(i+1), ROBOT_NEUTRAL);
  }
  return 0;
}

int Mobot_moveTo(br_comms_t* comms,
                               double angle1,
                               double angle2,
                               double angle3,
                               double angle4)
{
  if(Mobot_moveJointTo(comms, ROBOT_JOINT1, angle1)) {
    return -1;
  }
  if(Mobot_moveJointTo(comms, ROBOT_JOINT2, angle2)) {
    return -1;
  }
  if(Mobot_moveJointTo(comms, ROBOT_JOINT3, angle3)) {
    return -1;
  }
  if(Mobot_moveJointTo(comms, ROBOT_JOINT4, angle4)) {
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
  if(Mobot_moveJointTo(comms, ROBOT_JOINT1, angle1)) {
    return -1;
  }
  if(Mobot_moveJointTo(comms, ROBOT_JOINT2, angle2)) {
    return -1;
  }
  if(Mobot_moveJointTo(comms, ROBOT_JOINT3, angle3)) {
    return -1;
  }
  if(Mobot_moveJointTo(comms, ROBOT_JOINT4, angle4)) {
    return -1;
  }
  return 0;
}

int Mobot_moveJointWait(br_comms_t* comms, robotJointId_t id)
{
  robotJointState_t state;
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
    if(Mobot_moveJointWait(comms, (robotJointId_t)(i+1))) {
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
  Mobot_getJointAngle(comms, ROBOT_JOINT1, &motorPosition[0]);
  Mobot_getJointAngle(comms, ROBOT_JOINT4, &motorPosition[1]);
  Mobot_moveJointTo(comms, ROBOT_JOINT1, motorPosition[0] + DEG2RAD(90));
  Mobot_moveJointTo(comms, ROBOT_JOINT4, motorPosition[1] - DEG2RAD(90));
  Mobot_moveWait(comms);
  return 0;
}

int Mobot_motionRollBackward(br_comms_t* comms)
{
  double motorPosition[2];
  Mobot_getJointAngle(comms, ROBOT_JOINT1, &motorPosition[0]);
  Mobot_getJointAngle(comms, ROBOT_JOINT4, &motorPosition[1]);
  Mobot_moveJointTo(comms, ROBOT_JOINT1, motorPosition[0] - DEG2RAD(90));
  Mobot_moveJointTo(comms, ROBOT_JOINT4, motorPosition[1] + DEG2RAD(90));
  Mobot_moveWait(comms);
  return 0;
}

int Mobot_motionTurnLeft(br_comms_t* comms)
{
  double motorPosition[2];
  Mobot_getJointAngle(comms, ROBOT_JOINT1, &motorPosition[0]);
  Mobot_getJointAngle(comms, ROBOT_JOINT4, &motorPosition[1]);
  Mobot_moveJointTo(comms, ROBOT_JOINT1, motorPosition[0] + DEG2RAD(90));
  Mobot_moveJointTo(comms, ROBOT_JOINT4, motorPosition[1] + DEG2RAD(90));
  Mobot_moveWait(comms);
  return 0;
}

int Mobot_motionTurnRight(br_comms_t* comms)
{
  double motorPosition[2];
  Mobot_getJointAngle(comms, ROBOT_JOINT1, &motorPosition[0]);
  Mobot_getJointAngle(comms, ROBOT_JOINT4, &motorPosition[1]);
  Mobot_moveJointTo(comms, ROBOT_JOINT1, motorPosition[0] - DEG2RAD(90));
  Mobot_moveJointTo(comms, ROBOT_JOINT4, motorPosition[1] - DEG2RAD(90));
  Mobot_moveWait(comms);
  return 0;
}

int Mobot_motionInchwormLeft(br_comms_t* comms)
{
  Mobot_moveJointTo(comms, ROBOT_JOINT2, 0);
  Mobot_moveJointTo(comms, ROBOT_JOINT3, 0);
  Mobot_moveWait(comms);

  Mobot_moveJointTo(comms, ROBOT_JOINT3, DEG2RAD(50));
  Mobot_moveWait(comms);
  Mobot_moveJointTo(comms, ROBOT_JOINT2, DEG2RAD(-50));
  Mobot_moveWait(comms);
  Mobot_moveJointTo(comms, ROBOT_JOINT3, 0);
  Mobot_moveWait(comms);
  Mobot_moveJointTo(comms, ROBOT_JOINT2, 0);
  Mobot_moveWait(comms);

  return 0;
}

int Mobot_motionInchwormRight(br_comms_t* comms)
{
  Mobot_moveJointTo(comms, ROBOT_JOINT2, 0);
  Mobot_moveJointTo(comms, ROBOT_JOINT3, 0);
  Mobot_moveWait(comms);

  Mobot_moveJointTo(comms, ROBOT_JOINT2, DEG2RAD(-50));
  Mobot_moveWait(comms);
  Mobot_moveJointTo(comms, ROBOT_JOINT3, DEG2RAD(50));
  Mobot_moveWait(comms);
  Mobot_moveJointTo(comms, ROBOT_JOINT2, 0);
  Mobot_moveWait(comms);
  Mobot_moveJointTo(comms, ROBOT_JOINT3, 0);
  Mobot_moveWait(comms);

  return 0;
}

int Mobot_motionStand(br_comms_t* comms)
{
  Mobot_moveToZero(comms);
  Mobot_moveWait(comms);
  Mobot_moveJointTo(comms, ROBOT_JOINT2, DEG2RAD(-85));
  Mobot_moveJointTo(comms, ROBOT_JOINT3, DEG2RAD(80));
  Mobot_moveWait(comms);
  Mobot_moveJointTo(comms, ROBOT_JOINT1, DEG2RAD(45));
  Mobot_moveWait(comms);
  Mobot_moveJointTo(comms, ROBOT_JOINT2, DEG2RAD(20));
  Mobot_moveWait(comms);
  return 0;
}

void* motionInchwormLeftThread(void* arg)
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

void* motionInchwormRightThread(void* arg)
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

void* motionRollBackwardThread(void* arg)
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

void* motionRollForwardThread(void* arg)
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

void* motionStandThread(void* arg)
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

void* motionTurnLeftThread(void* arg)
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

void* motionTurnRightThread(void* arg)
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
  int err = 0;
#if 0
  char* str;
  str = (char*)malloc(strlen(buf)+2);
  strcpy(str, buf);
  strcat(str, "$");
  len++;
#endif
  //printf("SEND: <<%s>>\n", str);
  /* To send to the iMobot, we need to append the terminating character, '$' */
  if(comms->connected == 1) {
#ifdef _WIN32
    err = send(comms->socket, str, len, 0);
#else
    err = write(comms->socket, str, len);
#endif
  } else if (comms->connected == 2) {
#ifdef _WIN32
    DWORD bytes;
    if(!WriteFile(comms->hSerial, str, len, &bytes, NULL)) {
      err = -1;
    }
#else
    err = -1;
#endif
  } else {
    err = -1;
  }
  return err;
}

int RecvFromIMobot(br_comms_t* comms, char* buf, int size)
{
  int err = 0;
  int i = 0;
  int j = 0;
  int done = 0;
  int tries = 100;
  char tmp[256];
  buf[0] = '\0';
#ifdef _WIN32
  DWORD bytes;
#endif
  while(done == 0) {
    if(comms->connected == 1) {
#ifdef _WIN32
      fd_set fds;
      int n;
      struct timeval tv;
      /* Set up file descriptor set */
      FD_ZERO(&fds);
      FD_SET(comms->socket, &fds);
      /* Set up timeval for the timeout */
      tv.tv_sec = 1;
      tv.tv_usec = 0;
      /* Wait until timeout or data received */
      n = select(comms->socket, &fds, NULL, NULL, &tv);
      if(n == 0) {
        /* Timeout */
        return -1;
      }
      if(n == -1) {
        /* Error */
        return -1;
      }
      err = recvfrom(comms->socket, tmp, 256, 0, (struct sockaddr*)0, 0);
#else
      while(tries >= 0) {
        err = read(comms->socket, tmp, 255);
        if(err < 0) {
          tries--;
          //printf("*");
          usleep(1000);
        } else {
          break;
        }
      }
#endif
    } else if (comms->connected == 2) {
#ifdef _WIN32
      if(!ReadFile(comms->hSerial, tmp, 256, &bytes, NULL)) {
        err = -1;
      }
      err = bytes;
#else
      err = -1;
#endif
    } else {
      err = -1;
    }
    if(err < 0) break;
    for(i = 0; i < err; i++, j++) {
      buf[j] = tmp[i];
      if(tmp[i] == '\0') 
      {
        done = 1;
        break;
      }
    }
    tries = 100;
  }
  //printf("RECV: <<%s>>\n", buf);
  return err;
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

int CMobot::isMoving()
{
  return Mobot_isMoving(&_comms);
}

int CMobot::setJointDirection(robotJointId_t id, robotJointDirection_t dir)
{
  return Mobot_setJointDirection(&_comms, id, dir);
}

int CMobot::getJointDirection(robotJointId_t id, robotJointDirection_t &dir)
{
  return Mobot_getJointDirection(&_comms, id, &dir);
}

int CMobot::setJointSpeed(robotJointId_t id, double speed)
{
  return Mobot_setJointSpeed(&_comms, id, speed);
}

int CMobot::setJointSpeedRatio(robotJointId_t id, double ratio)
{
  return Mobot_setJointSpeedRatio(&_comms, id, ratio);
}

int CMobot::getJointSpeed(robotJointId_t id, double &speed)
{
  return Mobot_getJointSpeed(&_comms, id, &speed);
}

int CMobot::getJointSpeeds(robotJointId_t id, double speeds[4])
{
  return Mobot_getJointSpeeds(&_comms, speeds);
}

int CMobot::getJointSpeedRatios(robotJointId_t id, double ratios[4])
{
  return Mobot_getJointSpeeds(&_comms, ratios);
}

int CMobot::moveJointContinuousNB(robotJointId_t id, robotJointDirection_t dir)
{
  return Mobot_moveJointContinuousNB(&_comms, id, dir);
}

int CMobot::moveJointContinuousTime(robotJointId_t id, robotJointDirection_t dir, int msecs)
{
  return Mobot_moveJointContinuousTime(&_comms, id, dir, msecs);
}

int CMobot::moveJointTo(robotJointId_t id, double angle)
{
  return Mobot_moveJointTo(&_comms, id, angle);
}

int CMobot::moveJointToNB(robotJointId_t id, double angle)
{
  return Mobot_moveJointToNB(&_comms, id, angle);
}

int CMobot::moveJointToPIDNB(robotJointId_t id, double angle)
{
  return Mobot_moveJointToPIDNB(&_comms, id, angle);
}

int CMobot::getJointAngle(robotJointId_t id, double &angle)
{
  return Mobot_getJointAngle(&_comms, id, &angle);
}

int CMobot::getJointMaxSpeed(robotJointId_t id, double &maxSpeed)
{
  return Mobot_getJointMaxSpeed(&_comms, id, &maxSpeed);
}

int CMobot::getJointState(robotJointId_t id, robotJointState_t &state)
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

int CMobot::moveContinuousNB( robotJointDirection_t dir1, robotJointDirection_t dir2, robotJointDirection_t dir3, robotJointDirection_t dir4)
{
  return Mobot_moveContinuousNB(&_comms, dir1, dir2, dir3, dir4);
}

int CMobot::moveContinuousTime( robotJointDirection_t dir1, robotJointDirection_t dir2, robotJointDirection_t dir3, robotJointDirection_t dir4, int msecs)
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

int CMobot::moveJointWait(robotJointId_t id)
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

int CMobotGroup::add(CMobot& robot)
{
  _robots[_numRobots] = &robot;
  _numRobots++;
  return 0;
}

int CMobotGroup::move(double angle1, double angle2, double angle3, double angle4)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->moveNB(angle1, angle2, angle3, angle4);
  }
  return 0;
} 

int CMobotGroup::moveContinuousNB(robotJointDirection_t dir1, 
                       robotJointDirection_t dir2, 
                       robotJointDirection_t dir3, 
                       robotJointDirection_t dir4)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->moveContinuousNB(dir1, dir2, dir3, dir4);
  }
  return 0;
}

int CMobotGroup::moveContinuousTime(robotJointDirection_t dir1, 
                           robotJointDirection_t dir2, 
                           robotJointDirection_t dir3, 
                           robotJointDirection_t dir4, 
                           int msecs)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->moveContinuousNB(dir1, dir2, dir3, dir4);
  }
#ifdef _WIN32
  Sleep(msecs);
#else
  usleep(msecs*1000);
#endif
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->stop();
  }
  return 0;
}

int CMobotGroup::moveJointContinuousNB(robotJointId_t id, robotJointDirection_t dir)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->moveJointContinuousNB(id, dir);
  }
  return 0;
}

int CMobotGroup::moveJointContinuousTime(robotJointId_t id, robotJointDirection_t dir, int msecs)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->moveJointContinuousNB(id, dir);
  }
#ifdef _WIN32
  Sleep(msecs);
#else
  usleep(msecs * 1000);
#endif
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->stop();
  }
  return 0;
}

int CMobotGroup::moveJointTo(robotJointId_t id, double angle)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->moveJointToNB(id, angle);
  }
  return 0;
}

int CMobotGroup::moveJointWait(robotJointId_t id)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->moveJointWait(id);
  }
  return 0;
}

int CMobotGroup::moveTo(double angle1, double angle2, double angle3, double angle4)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->moveToNB(angle1, angle2, angle3, angle4);
  }
  moveWait();
  return 0;
}

int CMobotGroup::moveWait()
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->moveWait();
  }
  return 0;
}

int CMobotGroup::moveToZero()
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->moveToZero();
  }
  return 0;
}

int CMobotGroup::setJointSpeed(robotJointId_t id, double speed)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->setJointSpeed(id, speed);
  }
  return 0;
}

int CMobotGroup::setJointSpeedRatio(robotJointId_t id, double ratio)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->setJointSpeedRatio(id, ratio);
  }
  return 0;
}

int CMobotGroup::stop()
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->stop();
  }
  return 0;
}

int CMobotGroup::motionInchwormLeft()
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->motionInchwormLeftNB();
  }
  moveWait();
  return 0;
}

int CMobotGroup::motionInchwormRight()
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->motionInchwormRightNB();
  }
  moveWait();
  return 0;
}

int CMobotGroup::motionRollBackward()
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->motionRollBackwardNB();
  }
  moveWait();
  return 0;
}

int CMobotGroup::motionRollForward()
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->motionRollForwardNB();
  }
  moveWait();
  return 0;
}

int CMobotGroup::motionStand()
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->motionStandNB();
  }
  moveWait();
  return 0;
}

int CMobotGroup::motionTurnLeft()
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->motionTurnLeftNB();
  }
  moveWait();
  return 0;
}

int CMobotGroup::motionTurnRight()
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->motionTurnRightNB();
  }
  moveWait();
  return 0;
}
#endif
