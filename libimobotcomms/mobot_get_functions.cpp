#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <ctype.h>
#include "mobot.h"
#include "mobot_internal.h"
#ifndef _WIN32
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <libgen.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <termios.h>
#else
#include <windows.h>
#include <shlobj.h>
#endif

#ifdef _CH_
#include <stdarg.h>
#endif

#ifdef __MACH__
#include <mach/clock.h>
#include <mach/mach.h>
#endif

#ifndef _WIN32
#include <netdb.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <arpa/inet.h>
#else
#include <ws2tcpip.h>
#endif

#include "commands.h"

#define ABS(x) ((x)<0?-(x):(x))

#define DEPRECATED(from, to) \
  fprintf(stderr, "Warning: The function \"%s()\" is deprecated. Please use \"%s()\"\n" , from, to)

int Mobot_getQueriedAddresses(mobot_t* comms)
{
  int status;
  uint8_t buf[256];
  uint16_t addr;
  int i;
  status = SendToIMobot(comms, BTCMD(CMD_GETQUERIEDADDRESSES), NULL, 0);
  if(status < 0) return status;
  if(RecvFromIMobot(comms, buf, sizeof(buf))) {
    return -1;
  }

  for(i = 0; i < (buf[1]-3)/6; i++) {
    addr = buf[2+i*6] << 8;
    addr |= buf[3+i*6] & 0xff;
    //memcpy(&addr, &buf[2+i*2], 2);
    printf("Got address: 0x%4X\n", addr);
    
  }
  return 0;
}

int Mobot_getID(mobot_t* comms)
{
  int status;
  uint8_t buf[8];
  int addr;
  status = SendToIMobot(comms, BTCMD(CMD_GETSERIALID), NULL, 0);
  if(status < 0) return status;
  if(RecvFromIMobot(comms, buf, sizeof(buf))) {
    return -1;
  }
  /* Make sure the buf size is correct */
  if(buf[1] != 7) {
    return -1;
  }
  memcpy(comms->serialID, &buf[2], 4);
  return 0;
}

int Mobot_getBatteryVoltage(mobot_t* comms, double *voltage)
{
  uint8_t buf[32];
  float f;
  int status;
  status = SendToIMobot(comms, BTCMD(CMD_GETBATTERYVOLTAGE), NULL, 0);
  if(status < 0) return status;
  if(RecvFromIMobot(comms, buf, sizeof(buf))) {
    return -1;
  }
  /* Make sure the data size is correct */
  if(buf[1] != 7) {
    return -1;
  }
  /* Copy the data */
  memcpy(&f, &buf[2], 4);
  *voltage = f;
  return 0;
}

int Mobot_getButtonVoltage(mobot_t* comms, double *voltage)
{
  uint8_t buf[32];
  float f;
  int status;
  status = SendToIMobot(comms, BTCMD(CMD_GETBUTTONVOLTAGE), NULL, 0);
  if(status < 0) return status;
  if(RecvFromIMobot(comms, buf, sizeof(buf))) {
    return -1;
  }
  /* Make sure the data size is correct */
  if(buf[1] != 7) {
    return -1;
  }
  /* Copy the data */
  memcpy(&f, &buf[2], 4);
  *voltage = f;
  return 0;
}

const char* Mobot_getConfigFilePath()
{
  static char path[512];
  /* Find the configuration file path */
#ifdef _WIN32
  /* Find the user's local appdata directory */
  if(SHGetFolderPathA(NULL, CSIDL_LOCAL_APPDATA, NULL, 0, path) != S_OK) 
  {
    /* Could not get the user's app data directory */
  } else {
    //MessageBox((LPCTSTR)path, (LPCTSTR)"Test");
    //fprintf(fp, "%s", path); 
  }
  strcat(path, "\\Barobo.config");
#else
  /* Try to open the barobo configuration file. */
#define MAX_PATH 512
  strcpy(path, getenv("HOME"));
  strcat(path, "/.Barobo.config");
#endif
  return path;
}

int Mobot_getEncoderVoltage(mobot_t* comms, int pinNumber, double *voltage)
{
  uint8_t buf[32];
  float f;
  int status;
  buf[0] = pinNumber;
  status = SendToIMobot(comms, BTCMD(CMD_GETENCODERVOLTAGE), buf, 1);
  if(status < 0) return status;
  if(RecvFromIMobot(comms, buf, sizeof(buf))) {
    return -1;
  }
  /* Make sure the data size is correct */
  if(buf[1] != 7) {
    return -1;
  }
  /* Copy the data */
  memcpy(&f, &buf[2], 4);
  *voltage = f;
  return 0;
}

int Mobot_getHWRev(mobot_t* comms, int* rev)
{
  uint8_t buf[20];
  int status;
  if(status = SendToIMobot(comms, BTCMD(CMD_GETHWREV), NULL, 0)) {
    return status;
  }
  if(RecvFromIMobot(comms, buf, sizeof(buf))) {
    return -1;
  }
  if(buf[0] != RESP_OK) {
    return -1;
  }
  *rev = buf[2];
  return 0;
}

int Mobot_getJointAngle(mobot_t* comms, mobotJointId_t id, double *angle)
{
  uint8_t buf[32];
  float f;
  int status;
  buf[0] = (uint8_t)id-1;
  status = SendToIMobot(comms, BTCMD(CMD_GETMOTORANGLEABS), buf, 1);
  if(status < 0) return status;
  if(RecvFromIMobot(comms, buf, sizeof(buf))) {
    return -1;
  }
  /* Make sure the data size is correct */
  if(buf[1] != 7) {
    return -1;
  }
  /* Copy the data */
  memcpy(&f, &buf[2], 4);
  *angle = f;
  return 0;
}

int Mobot_getJointAngleAverage(mobot_t* comms, mobotJointId_t id, double *angle, int numReadings)
{
  int i;
  double d;
  *angle = 0;
  for(i = 0; i < numReadings; i++) {
    if(Mobot_getJointAngle(comms, id, &d)) {
      return -1;
    }
    *angle += d;
  }
  *angle = *angle / numReadings;
  return 0;
}

int Mobot_getJointAngles(mobot_t* comms, 
                             double *angle1,
                             double *angle2,
                             double *angle3,
                             double *angle4)
{
  uint8_t buf[32];
  float f;
  uint32_t millis;
  int status;
  status = SendToIMobot(comms, BTCMD(CMD_GETMOTORANGLESTIMESTAMPABS), NULL, 0);
  if(status < 0) return status;
  if(RecvFromIMobot(comms, buf, sizeof(buf))) {
    return -1;
  }
  /* Make sure the data size is correct */
  if(buf[1] != 0x17) {
    return -1;
  }
  /* Copy the data */
  /* Copy 4 joint angles */
  memcpy(&f, &buf[6], 4);
  *angle1 = f;
  memcpy(&f, &buf[10], 4);
  *angle2 = f;
  memcpy(&f, &buf[14], 4);
  *angle3 = f;
  memcpy(&f, &buf[18], 4);
  *angle4 = f;
  return 0;
}

int Mobot_getJointAnglesAverage(mobot_t* comms, 
                             double *angle1,
                             double *angle2,
                             double *angle3,
                             double *angle4,
                             int numReadings)
{
  double d[4] = {0, 0, 0, 0};
  int i;
  *angle1 = 0;
  *angle2 = 0;
  *angle3 = 0;
  *angle4 = 0;
  for(i = 0; i < numReadings; i++) {
    Mobot_getJointAngles(comms, 
        &d[0],
        &d[1],
        &d[2],
        &d[3]);
    *angle1 += d[0];
    *angle2 += d[1];
    *angle3 += d[2];
    *angle4 += d[3];
  }
  *angle1 = *angle1 / numReadings;
  *angle2 = *angle2 / numReadings;
  *angle3 = *angle3 / numReadings;
  *angle4 = *angle4 / numReadings;
  return 0;
}

int Mobot_getJointAnglesTime(mobot_t* comms, 
                             double *time, 
                             double *angle1,
                             double *angle2,
                             double *angle3,
                             double *angle4)
{
  uint8_t buf[32];
  float f;
  uint32_t millis;
  int status;
  status = SendToIMobot(comms, BTCMD(CMD_GETMOTORANGLESTIMESTAMPABS), NULL, 0);
  if(status < 0) return status;
  if(RecvFromIMobot(comms, buf, sizeof(buf))) {
    return -1;
  }
  /* Make sure the data size is correct */
  if(buf[1] != 0x17) {
    return -1;
  }
  /* Copy the data */
  memcpy(&millis, &buf[2], 4);
  *time = millis / 1000.0;
  /* Copy 4 joint angles */
  memcpy(&f, &buf[6], 4);
  *angle1 = f;
  memcpy(&f, &buf[10], 4);
  *angle2 = f;
  memcpy(&f, &buf[14], 4);
  *angle3 = f;
  memcpy(&f, &buf[18], 4);
  *angle4 = f;
  return 0;
}

int Mobot_getJointAnglesTimeState(mobot_t* comms,
                             double *time, 
                             double *angle1,
                             double *angle2,
                             double *angle3,
                             double *angle4,
                             mobotJointState_t* state1,
                             mobotJointState_t* state2,
                             mobotJointState_t* state3,
                             mobotJointState_t* state4)
{
  uint32_t millis;
  uint8_t buf[64];
  int status;
  int i;
  float angles[4];
  mobotJointState_t* states[4];
  states[0] = state1;
  states[1] = state2;
  states[2] = state3;
  states[3] = state4;
  status = SendToIMobot(comms, BTCMD(CMD_GETBIGSTATE), NULL, 0);
  if(status < 0) return status;
  if(RecvFromIMobot(comms, buf, sizeof(buf))) {
    return -1;
  }
  /* Make sure the data size is correct */
  if(buf[1] != 27) {
    return -1;
  }
  /* Copy the timestamp */
  memcpy(&millis, &buf[2], 4);
  *time = millis/1000.0;
  for(i = 0; i < 4; i++) {
    /* Copy the motor angles */
    memcpy(&angles[i], &buf[6 + i*4], 4);
    /* Copy the motor states */
    *states[i] = (mobotJointState_t)buf[22+i];
  }
  *angle1 = angles[0];
  *angle2 = angles[1];
  *angle3 = angles[2];
  *angle4 = angles[3];
  return 0;
}

int Mobot_getJointAnglesTimeIsMoving(mobot_t* comms,
                             double *time, 
                             double *angle1,
                             double *angle2,
                             double *angle3,
                             double *angle4,
                             int *isMoving)
{
  int i, rc;
  mobotJointState_t states[4];
  rc = Mobot_getJointAnglesTimeState( comms,
     time, 
     angle1,
     angle2,
     angle3,
     angle4,
     &states[0],
     &states[1],
     &states[2],
     &states[3]); 
  if(rc) {
    return rc;
  }
  *isMoving = 0;
  for(i = 0; i < 4; i++) {
    if(states[i] == MOBOT_FORWARD ||
        states[i] == MOBOT_BACKWARD)
    {
      *isMoving = 1;
    }
  }
  return 0;
}

int Mobot_getJointDirection(mobot_t* comms, mobotJointId_t id, mobotJointState_t *dir)
{
  uint8_t buf[32];
  int status;
  buf[0] = (uint8_t)id-1;
  status = SendToIMobot(comms, BTCMD(CMD_GETMOTORDIR), buf, 1);
  if(status < 0) return status;
  if(RecvFromIMobot(comms, buf, sizeof(buf))) {
    return -1;
  }
  /* Make sure the data size is correct */
  if(buf[1] != 0x04) {
    return -1;
  }
  *dir = (mobotJointState_t)buf[2];
  return 0;
}

int Mobot_getJointMaxSpeed(mobot_t* comms, mobotJointId_t id, double *maxSpeed)
{
  float f;
  uint8_t buf[64];
  int status;
  int bytes_read;
  buf[0] = (uint8_t) id-1;
  status = SendToIMobot(comms, BTCMD(CMD_GETMOTORMAXSPEED), &buf[0], 1);
  if(status < 0) return status;
  if(RecvFromIMobot(comms, buf, sizeof(buf))) {
    return -1;
  }
  /* Make sure the data size is correct */
  if(buf[1] != 7) {
    return -1;
  }
  /* Copy the data */
  memcpy(&f, &buf[2], 4);
  *maxSpeed = f;
  return 0;
}

int Mobot_getJointSafetyAngle(mobot_t* comms, double *angle) 
{
  uint8_t buf[32];
  float f;
  int status;
  status = SendToIMobot(comms, BTCMD(CMD_GETMOTORSAFETYLIMIT), NULL, 0);
  if(status < 0) return status;
  if(RecvFromIMobot(comms, buf, sizeof(buf))) {
    return -1;
  }
  /* Make sure the data size is correct */
  if(buf[1] != 0x07) {
    return -1;
  }
  memcpy(&f, &buf[2], 4);
  *angle = f;
  return 0;
}

int Mobot_getJointSafetyAngleTimeout(mobot_t* comms, double *seconds) 
{
  uint8_t buf[32];
  float f;
  int status;
  status = SendToIMobot(comms, BTCMD(CMD_GETMOTORSAFETYTIMEOUT), NULL, 0);
  if(status < 0) return status;
  if(RecvFromIMobot(comms, buf, sizeof(buf))) {
    return -1;
  }
  /* Make sure the data size is correct */
  if(buf[1] != 0x07) {
    return -1;
  }
  memcpy(&f, &buf[2], 4);
  *seconds = f;
  return 0;
}

int Mobot_getJointSpeed(mobot_t* comms, mobotJointId_t id, double *speed)
{
  uint8_t buf[32];
  float f;
  int status;
  buf[0] = (uint8_t)id-1;
  status = SendToIMobot(comms, BTCMD(CMD_GETMOTORSPEED), buf, 1);
  if(status < 0) return status;
  if(RecvFromIMobot(comms, buf, sizeof(buf))) {
    return -1;
  }
  /* Make sure the data size is correct */
  if(buf[1] != 0x07) {
    return -1;
  }
  memcpy(&f, &buf[2], 4);
  *speed = ABS(f);
  comms->jointSpeeds[id-1] = *speed;
  return 0;
}

int Mobot_getJointSpeedRatio(mobot_t* comms, mobotJointId_t id, double *ratio)
{
  double speed;
  Mobot_getJointSpeed(comms, id, &speed);
  *ratio = speed / comms->maxSpeed[(int)id-1];
  return 0;
}

int Mobot_getJointSpeedRatios(mobot_t* comms, double *ratio1, double *ratio2, double *ratio3, double *ratio4)
{
  double *ratios[4];
  ratios[0] = ratio1;
  ratios[1] = ratio2;
  ratios[2] = ratio3;
  ratios[3] = ratio4;
  for(int i = 0; i < 4; i++) {
    Mobot_getJointSpeedRatio(comms, (mobotJointId_t)(i+1), ratios[i]);
  }
  return 0;
}

int Mobot_getJointState(mobot_t* comms, mobotJointId_t id, mobotJointState_t *state)
{
  uint8_t buf[32];
  int status;
  buf[0] = (uint8_t)id-1;
  status = SendToIMobot(comms, BTCMD(CMD_GETMOTORSTATE), buf, 1);
  if(status < 0) return status;
  if(RecvFromIMobot(comms, buf, sizeof(buf))) {
    return -1;
  }
  /* Make sure the data size is correct */
  if(buf[1] != 0x04) {
    return -1;
  }
  *state = (mobotJointState_t)buf[2];
  return 0;
}

int Mobot_getJointSpeeds(mobot_t* comms, double *speed1, double *speed2, double *speed3, double *speed4)
{
  double *speeds[4];
  speeds[0] = speed1;
  speeds[1] = speed2;
  speeds[2] = speed3;
  speeds[3] = speed4;
  for(int i = 0; i < 4; i++) {
    Mobot_getJointSpeed(comms, (mobotJointId_t)(i+1), speeds[i]);
  }
  return 0;
}

int Mobot_getStatus(mobot_t* comms)
{
  uint8_t buf[64];
  SendToIMobot(comms, BTCMD(CMD_STATUS), NULL, 0);
  RecvFromIMobot(comms, buf, 64);
  if(buf[0] != RESP_OK) {
    return -1;
  }
  if(buf[1] != 3 ) {
    return -1;
  }
  if(buf[2] != RESP_END) {
    return -1;
  }
  return 0;
}

int Mobot_getVersion(mobot_t* comms)
{
  uint8_t buf[16];
  int version;
  SendToIMobot(comms, BTCMD(CMD_GETVERSION), NULL, 0);
  RecvFromIMobot(comms, buf, 16);
  if(buf[0] != RESP_OK) {
    return -1;
  }
  if(buf[1] != 4 ) {
    return -1;
  }
  if(buf[3] != RESP_END) {
    return -1;
  }
  version = buf[2];
  return version;
}

const char* CMobot::getConfigFilePath()
{
  return Mobot_getConfigFilePath();
}

int CMobot::getJointAngle(mobotJointId_t id, double &angle)
{
  int err;
  err = Mobot_getJointAngle(_comms, id, &angle);
  angle = RAD2DEG(angle);
  return err;
}

int CMobot::getJointAngleAverage(mobotJointId_t id, double &angle, int numReadings)
{
  int err;
  err = Mobot_getJointAngleAverage(_comms, id, &angle, numReadings);
  angle = RAD2DEG(angle);
  return err;
}

int CMobot::getJointAngles(
    double &angle1,
    double &angle2,
    double &angle3,
    double &angle4)
{
  double time;
  int err;
  err = Mobot_getJointAnglesTime(
      _comms, 
      &time,
      &angle1,
      &angle2,
      &angle3,
      &angle4);
  if(err) return err;
  angle1 = RAD2DEG(angle1);
  angle2 = RAD2DEG(angle2);
  angle3 = RAD2DEG(angle3);
  angle4 = RAD2DEG(angle4);
  return 0;
}

int CMobot::getJointAnglesAverage(
    double &angle1,
    double &angle2,
    double &angle3,
    double &angle4,
    int numReadings)
{
  int err;
  err = Mobot_getJointAnglesAverage(
      _comms, 
      &angle1,
      &angle2,
      &angle3,
      &angle4,
      numReadings);
  if(err) return err;
  angle1 = RAD2DEG(angle1);
  angle2 = RAD2DEG(angle2);
  angle3 = RAD2DEG(angle3);
  angle4 = RAD2DEG(angle4);
  return 0;
}

int CMobot::getJointDirection(mobotJointId_t id, mobotJointState_t &dir)
{
  return Mobot_getJointDirection(_comms, id, &dir);
}

int CMobot::getJointMaxSpeed(mobotJointId_t id, double &maxSpeed)
{
  int err = Mobot_getJointMaxSpeed(_comms, id, &maxSpeed);
  maxSpeed = RAD2DEG(maxSpeed);
  return err;
}

int CMobot::getJointSafetyAngle(double &angle)
{
  int r;
  double a;
  r = Mobot_getJointSafetyAngle(_comms, &a);
  angle = RAD2DEG(a);
  return r;
}

int CMobot::getJointSafetyAngleTimeout(double &seconds)
{
  return Mobot_getJointSafetyAngleTimeout(_comms, &seconds);
}

int CMobot::getJointSpeed(mobotJointId_t id, double &speed)
{
  int err;
  err = Mobot_getJointSpeed(_comms, id, &speed);
  speed = RAD2DEG(speed);
  return err;
}

int CMobot::getJointSpeedRatio(mobotJointId_t id, double &ratio)
{
  return Mobot_getJointSpeedRatio(_comms, id, &ratio);
}

int CMobot::getJointSpeeds(double &speed1, double &speed2, double &speed3, double &speed4)
{
  int i;
  int err = Mobot_getJointSpeeds(_comms, &speed1, &speed2, &speed3, &speed4);
  speed1 = RAD2DEG(speed1);
  speed2 = RAD2DEG(speed2);
  speed3 = RAD2DEG(speed3);
  speed4 = RAD2DEG(speed4);
  return err;
}

int CMobot::getJointSpeedRatios(double &ratio1, double &ratio2, double &ratio3, double &ratio4)
{
  return Mobot_getJointSpeedRatios(_comms, &ratio1, &ratio2, &ratio3, &ratio4);
}

int CMobot::getJointState(mobotJointId_t id, mobotJointState_t &state)
{
  return Mobot_getJointState(_comms, id, &state);
}

mobot_t* CMobot::getMobotObject()
{
  return _comms;
}

