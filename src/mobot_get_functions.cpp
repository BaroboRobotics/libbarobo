/*
   Copyright 2013 Barobo, Inc.

   This file is part of libbarobo.

   BaroboLink is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   BaroboLink is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with BaroboLink.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "logging.h"

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

#include "rgbhashtable.h"

#define ABS(x) ((x)<0?-(x):(x))

#define DEPRECATED(from, to) \
  fprintf(stderr, "Warning: The function \"%s()\" is deprecated. Please use \"%s()\"\n" , from, to)

int Mobot_LinkPodAnalogRead(mobot_t* comms, int adc, int* value)
{
  uint8_t buf[32];
  uint16_t incoming;
  int rc;
  buf[0] = MSG_REGACCESS;
  buf[1] = TWIMSG_ANALOGREADPIN;
  buf[2] = adc;
  rc = Mobot_twiSendRecv(comms, 0x02, buf, 3, &incoming, 2);
  if(rc) return rc;
  *value = incoming; 
  return 0;
}

int Mobot_LinkPodAnalogReadVolts(mobot_t* comms, int adc, double *volts)
{
  int value;
  int rc;
  rc = Mobot_LinkPodAnalogRead(comms, adc, &value);
  if(rc) return rc;
  *volts = ((double)value)/1024.0 * 5.0;
  return 0;
}

int Mobot_LinkPodDigitalRead(mobot_t* comms, int pin, int *value)
{
  uint8_t incoming;
  uint8_t buf[32];
  int rc;
  buf[0] = MSG_REGACCESS;
  buf[1] = TWIMSG_DIGITALREADPIN;
  buf[2] = pin;
  rc = Mobot_twiSendRecv(comms, 0x02, buf, 3, &incoming, 1);
  if(rc) return rc;
  *value = incoming; 
  return 0;
}

int Mobot_getQueriedAddresses(mobot_t* comms)
{
  int status;
  uint8_t buf[256];
  uint16_t addr;
  int i;
  status = MobotMsgTransaction(comms, BTCMD(CMD_GETQUERIEDADDRESSES), buf, 0);
  if(status < 0) return status;

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
  status = MobotMsgTransaction(comms, BTCMD(CMD_GETSERIALID), buf, 0);
  if(status < 0) return status;
  /* Make sure the buf size is correct */
  if(buf[1] != 7) {
    return -1;
  }
  memcpy(comms->serialID, &buf[2], 4);
  return 0;
}

int Mobot_getAccelerometerData(mobot_t* comms, double *accel_x, double *accel_y, double *accel_z)
{
  uint8_t buf[32];
  float f;
  int status;
  int16_t i;
  status = MobotMsgTransaction(comms, BTCMD(CMD_GETACCEL), buf, 0);
  if(status < 0) return status;
  /* Make sure the data size is correct */
  if(buf[1] != 0x09) {
    return -1;
  }
  /* Copy the data */
  memcpy(&i, &buf[2], 2);
  *accel_x = (double)i/16384.0;
  memcpy(&i, &buf[4], 2);
  *accel_y = (double)i/16384.0;
  memcpy(&i, &buf[6], 2);
  *accel_z = (double)i/16384.0;
  return 0;
}

int Mobot_getBatteryVoltage(mobot_t* comms, double *voltage)
{
  uint8_t buf[32];
  float f;
  int status;
  status = MobotMsgTransaction(comms, BTCMD(CMD_GETBATTERYVOLTAGE), buf, 0);
  if(status < 0) return status;
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
  status = MobotMsgTransaction(comms, BTCMD(CMD_GETBUTTONVOLTAGE), buf, 0);
  if(status < 0) return status;
  /* Make sure the data size is correct */
  if(buf[1] != 7) {
    return -1;
  }
  /* Copy the data */
  memcpy(&f, &buf[2], 4);
  *voltage = f;
  return 0;
}

int Mobot_getChildrenInfo(mobot_t* comms, mobotInfo_t **mobotInfo, int *numChildren )
{
  /* Make sure we are the parent */
  if(comms->parent != NULL) {
    return 0;
  }
  /* Count the number of children */
  int n=0, i=0;
  mobotInfo_t* iter;
  MUTEX_LOCK(comms->mobotTree_lock);
  for(iter = comms->children; iter != NULL; iter = iter->next) {
    n++;
  }
  /* Allocate */
  *mobotInfo = (mobotInfo_t*)malloc(sizeof(mobotInfo_t)*n);
  /* Copy IDs */
  for(iter = comms->children, i=0; iter != NULL; iter = iter->next, i++) {
    strcpy((*mobotInfo)[i].serialID, iter->serialID);
    (*mobotInfo)[i].zigbeeAddr = iter->zigbeeAddr;
  }
  *numChildren = n;
  MUTEX_UNLOCK(comms->mobotTree_lock);
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

int Mobot_getDistance(mobot_t* comms, double *distance, double radius)
{
  int rc;
  double angle;
  rc = Mobot_getJointAngleAverage(comms, ROBOT_JOINT1, &angle, 10);
  if(rc) return rc;
  *distance = angle * radius;
  return 0;
}

int Mobot_getEncoderVoltage(mobot_t* comms, int pinNumber, double *voltage)
{
  uint8_t buf[32];
  float f;
  int status;
  buf[0] = pinNumber;
  status = MobotMsgTransaction(comms, BTCMD(CMD_GETENCODERVOLTAGE), buf, 1);
  if(status < 0) return status;
  /* Make sure the data size is correct */
  if(buf[1] != 7) {
    return -1;
  }
  /* Copy the data */
  memcpy(&f, &buf[2], 4);
  *voltage = f;
  return 0;
}

int Mobot_getFormFactor(mobot_t* comms, int* form)
{
  *form = (int)comms->formFactor;
  return 0;
}

int Mobot_getHWRev(mobot_t* comms, int* rev)
{
  uint8_t buf[20];
  int status;
  if(status = MobotMsgTransaction(comms, BTCMD(CMD_GETHWREV), buf, 0)) {
    return status;
  }
  if(buf[0] != RESP_OK) {
    return -1;
  }
  *rev = buf[2];
  return 0;
}

int Mobot_getJointAngle(mobot_t* comms, robotJointId_t id, double *angle)
{
  uint8_t buf[32];
  float f;
  int status;
  buf[0] = (uint8_t)id-1;
  status = MobotMsgTransaction(comms, BTCMD(CMD_GETMOTORANGLEABS), buf, 1);
  if(status < 0) return status;
  /* Make sure the data size is correct */
  if(buf[1] != 7) {
    return -1;
  }
  /* Copy the data */
  memcpy(&f, &buf[2], 4);
  *angle = f;
  return 0;
}

int Mobot_getJointAngleAverage(mobot_t* comms, robotJointId_t id, double *angle, int numReadings)
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
  status = MobotMsgTransaction(comms, BTCMD(CMD_GETMOTORANGLESTIMESTAMPABS), buf, 0);
  if(status < 0) return status;
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
  status = MobotMsgTransaction(comms, BTCMD(CMD_GETMOTORANGLESTIMESTAMPABS), buf, 0);
  if(status < 0) return status;
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
                             robotJointState_t* state1,
                             robotJointState_t* state2,
                             robotJointState_t* state3,
                             robotJointState_t* state4)
{
  uint32_t millis;
  uint8_t buf[64];
  int status;
  int i;
  float angles[4];
  robotJointState_t* states[4];
  states[0] = state1;
  states[1] = state2;
  states[2] = state3;
  states[3] = state4;
  status = MobotMsgTransaction(comms, BTCMD(CMD_GETBIGSTATE), buf, 0);
  if(status < 0) return status;
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
    *states[i] = (robotJointState_t)buf[22+i];
  }
  *angle1 = angles[0];
  *angle2 = angles[1];
  *angle3 = angles[2];
  *angle4 = angles[3];
  if( comms->formFactor == MOBOTFORM_I) {
    if(*angle3 == ROBOT_FORWARD) {
      *angle3 = ROBOT_BACKWARD;
    } else if (*angle3 == ROBOT_BACKWARD) {
      *angle3 = ROBOT_FORWARD;
    }
    *angle2 = 0;
    *state2 = ROBOT_NEUTRAL;
    *angle4 = 0;
    *state4 = ROBOT_NEUTRAL;
  } else if( comms->formFactor == MOBOTFORM_L) {
    *angle3 = 0;
    *state3 = ROBOT_NEUTRAL;
    *angle4 = 0;
    *state4 = ROBOT_NEUTRAL;
  }
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
  robotJointState_t states[4];
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
    if(states[i] == ROBOT_FORWARD ||
        states[i] == ROBOT_BACKWARD)
    {
      *isMoving = 1;
    }
  }
  return 0;
}

int Mobot_getJointDirection(mobot_t* comms, robotJointId_t id, robotJointState_t *dir)
{
  uint8_t buf[32];
  int status;
  buf[0] = (uint8_t)id-1;
  status = MobotMsgTransaction(comms, BTCMD(CMD_GETMOTORDIR), buf, 1);
  if(status < 0) return status;
  /* Make sure the data size is correct */
  if(buf[1] != 0x04) {
    return -1;
  }
  *dir = (robotJointState_t)buf[2];
  if(
      (comms->formFactor == MOBOTFORM_I) &&
      (id == ROBOT_JOINT3)
    )
  {
    if(*dir == ROBOT_FORWARD) {
      *dir = ROBOT_BACKWARD;
    } else if (*dir == ROBOT_BACKWARD) {
      *dir = ROBOT_FORWARD;
    }
  }
  return 0;
}

int Mobot_getJointMaxSpeed(mobot_t* comms, robotJointId_t id, double *maxSpeed)
{
  float f;
  uint8_t buf[64];
  int status;
  int bytes_read;
  buf[0] = (uint8_t) id-1;
  status = MobotMsgTransaction(comms, BTCMD(CMD_GETMOTORMAXSPEED), &buf[0], 1);
  if(status < 0) return status;
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
  status = MobotMsgTransaction(comms, BTCMD(CMD_GETMOTORSAFETYLIMIT), buf, 0);
  if(status < 0) return status;
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
  status = MobotMsgTransaction(comms, BTCMD(CMD_GETMOTORSAFETYTIMEOUT), buf, 0);
  if(status < 0) return status;
  /* Make sure the data size is correct */
  if(buf[1] != 0x07) {
    return -1;
  }
  memcpy(&f, &buf[2], 4);
  *seconds = f;
  return 0;
}

int Mobot_getJointSpeed(mobot_t* comms, robotJointId_t id, double *speed)
{
  uint8_t buf[32];
  float f;
  int status;
  buf[0] = (uint8_t)id-1;
  status = MobotMsgTransaction(comms, BTCMD(CMD_GETMOTORSPEED), buf, 1);
  if(status < 0) return status;
  /* Make sure the data size is correct */
  if(buf[1] != 0x07) {
    return -1;
  }
  memcpy(&f, &buf[2], 4);
  *speed = ABS(f);
  comms->jointSpeeds[id-1] = *speed;
  return 0;
}

int Mobot_getJointSpeedRatio(mobot_t* comms, robotJointId_t id, double *ratio)
{
  double speed;
  Mobot_getJointSpeed(comms, id, &speed);
  *ratio = speed / comms->maxSpeed[(int)id-1];
  return 0;
}

int Mobot_getJointSpeedRatios(mobot_t* comms, double *ratio1, double *ratio2, double *ratio3, double *ratio4)
{
  double *ratios[4];
  int i;
  ratios[0] = ratio1;
  ratios[1] = ratio2;
  ratios[2] = ratio3;
  ratios[3] = ratio4;
  for(i = 0; i < 4; i++) {
    Mobot_getJointSpeedRatio(comms, (robotJointId_t)(i+1), ratios[i]);
  }
  return 0;
}

int Mobot_getJointState(mobot_t* comms, robotJointId_t id, robotJointState_t *state)
{
  uint8_t buf[32];
  int status;
  buf[0] = (uint8_t)id-1;
  status = MobotMsgTransaction(comms, BTCMD(CMD_GETMOTORSTATE), buf, 1);
  if(status < 0) return status;
  /* Make sure the data size is correct */
  if(buf[1] != 0x04) {
    *state = ROBOT_NEUTRAL;
    return -1;
  }
  *state = (robotJointState_t)buf[2];
  if(
      (comms->formFactor == MOBOTFORM_I) &&
      (id == ROBOT_JOINT3)
    )
  {
    if(*state == ROBOT_FORWARD) {
      *state = ROBOT_BACKWARD;
    } else if (*state == ROBOT_BACKWARD) {
      *state = ROBOT_FORWARD;
    }
  }
  return 0;
}

int Mobot_getJointSpeeds(mobot_t* comms, double *speed1, double *speed2, double *speed3, double *speed4)
{
  double *speeds[4];
  int i;
  speeds[0] = speed1;
  speeds[1] = speed2;
  speeds[2] = speed3;
  speeds[3] = speed4;
  for(i = 0; i < 4; i++) {
    Mobot_getJointSpeed(comms, (robotJointId_t)(i+1), speeds[i]);
  }
  return 0;
}

int Mobot_getMasterAddress(mobot_t* comms, uint16_t* addr)
{
  uint8_t buf[32];
  int status;
  status = MobotMsgTransaction(comms, BTCMD(CMD_GET_MASTER_ADDRESS), buf, 0);
  if(status < 0) return status;
  if(buf[0] != RESP_OK) {
    return -1;
  }
  *addr = buf[2]<<8;
  *addr |= buf[3];

  return 0;
}

int Mobot_getNumSlaves(mobot_t* comms, int* num)
{
  uint8_t buf[32];
  int status;
  status = MobotMsgTransaction(comms, BTCMD(CMD_GET_NUM_SLAVES), buf, 0);
  if(status < 0) return status;
  if(buf[0] != RESP_OK) {
    return -1;
  }
  *num = buf[2];
  return 0;
}

int Mobot_getSlaveAddr(mobot_t* comms, uint8_t index, uint16_t* addr)
{
  uint8_t buf[32];
  int status;
  buf[0] = index;
  status = MobotMsgTransaction(comms, BTCMD(CMD_GET_SLAVE_ADDR), buf, 1);
  if(status < 0) return status;
  if(buf[0] != RESP_OK) {
    return -1;
  }
  *addr = buf[2]<<8;
  *addr |= buf[3];
  return 0;
}

int Mobot_getNumPoses(mobot_t* comms, int* num)
{
  uint8_t buf[32];
  int status;
  status = MobotMsgTransaction(comms, BTCMD(CMD_GET_NUM_POSES), buf, 0);
  if(status < 0) return status;
  if(buf[0] != RESP_OK) {
    return -1;
  }
  *num = buf[2];
  return 0;
}

int Mobot_getPoseData(mobot_t* comms, uint8_t index, double *angle1, double *angle2, double *angle3, double *angle4)
{
  uint8_t buf[32];
  int status;
  float f[4];
  buf[0] = index;
  status = MobotMsgTransaction(comms, BTCMD(CMD_GET_POSE_DATA), buf, 1);
  if(status < 0) return status;
  if(buf[0] != RESP_OK) {
    return -1;
  }
  memcpy(&f[0], &buf[2], 4);
  memcpy(&f[1], &buf[6], 4);
  memcpy(&f[2], &buf[10], 4);
  memcpy(&f[3], &buf[14], 4);
  *angle1 = f[0];
  *angle2 = f[1];
  *angle3 = f[2];
  *angle4 = f[3];
  return 0;
}

int Mobot_getColorRGB(mobot_t* comms, int *r, int *g, int *b)
{
  uint8_t buf[32];
  float f;
  int status;

  status = MobotMsgTransaction(comms, BTCMD(CMD_GETRGB), buf, 0);
  if(status < 0) return status;
  /* Make sure the data size is correct */
  if(buf[1] != 6) {
    return -1;
  }
  *r = buf[2];
  *g = buf[3];
  *b = buf[4];
  return 0;
}

int Mobot_getColor(mobot_t* comms, char color[])
{
  uint8_t buf[32];
  float f;
  int status;
  int getRGB[3];
  int retval;
  rgbHashTable * rgbTable = NULL;

  status = MobotMsgTransaction(comms, BTCMD(CMD_GETRGB), buf, 0);
  if(status < 0) return status;
  /* Make sure the data size is correct */
  if(buf[1] != 6){
	return -1;
  }

  getRGB[0] = buf[2];
  getRGB[1] = buf[3]; 
  getRGB[2] = buf[4];

  rgbTable = HT_Create();
  retval = HT_GetKey(rgbTable, getRGB, color);
  HT_Destroy(rgbTable);
  
  return retval; //will be either 0 or -1, depending on if there is a valid hash table
}

int Mobot_getStatus(mobot_t* comms)
{
  uint8_t buf[64];
  int rc;
  rc = MobotMsgTransaction(comms, BTCMD(CMD_STATUS), buf, 0);
  //bInfo(stderr, "(barobo) INFO: %d == MobotMsgTransaction(): %02x %02x %02x\n", rc, buf[0], buf[1], buf[2]);
  if(rc) {return rc;}
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
  int rc;
  rc = MobotMsgTransaction(comms, BTCMD(CMD_GETVERSION), buf, 0);
  if(rc) {return rc;}
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

