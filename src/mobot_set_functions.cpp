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

#define DEPRECATED(from, to) \
  fprintf(stderr, "Warning: The function \"%s()\" is deprecated. Please use \"%s()\"\n" , from, to)

int Mobot_LinkPodAnalogWrite(mobot_t* comms, int pin, int value)
{
  if( ! (
        (pin == 3) ||
        (pin == 5) ||
        (pin == 6) ||
        (pin == 9) ||
        (pin == 10) ||
        (pin == 11)
        )
    )
  {
    fprintf(stderr, "ERROR: in Mobot_LinkPodAnalogWrite: Pin %d does not support\nanalog output.\n", 
        pin);
    return -1;
  }
  if(value < 0 || value > 255) {
    fprintf(stderr, "ERROR: in Mobot_LinkPodAnalogWrite: value must be between 0 and 255 inclusive.\n");
    return -1;
  }
  uint8_t buf[32];
  buf[0] = MSG_REGACCESS;
  buf[1] = TWIMSG_ANALOGWRITEPIN;
  buf[2] = (uint8_t)pin;
  buf[3] = (uint8_t)value;
  return Mobot_twiSend(comms, 0x02, buf, 4);
}

int Mobot_LinkPodAnalogReference(mobot_t* comms, int ref)
{
  uint8_t buf[32];
  buf[0] = MSG_REGACCESS;
  buf[1] = TWIMSG_ANALOGREF;
  buf[2] = (uint8_t)ref;
  return Mobot_twiSend(comms, 0x02, buf, 3);
}

int Mobot_LinkPodDigitalWrite(mobot_t* comms, int pin, int value)
{
  uint8_t buf[32];
  buf[0] = MSG_REGACCESS;
  buf[1] = TWIMSG_DIGITALWRITEPIN;
  buf[2] = (uint8_t)pin;
  buf[3] = (uint8_t)value;
  return Mobot_twiSend(comms, 0x02, buf, 4);
}

int Mobot_LinkPodPinMode(mobot_t* comms, int pin, int mode)
{
  uint8_t buf[32];
  buf[0] = MSG_REGACCESS;
  buf[1] = TWIMSG_SETPINMODE;
  buf[2] = (uint8_t)pin;
  buf[3] = (uint8_t)mode;
  return Mobot_twiSend(comms, 0x02, buf, 4);
}

int Mobot_setBuzzerFrequency(mobot_t* comms, unsigned int frequency, double time)
{
  int rc;
  if(rc = Mobot_setBuzzerFrequencyOn(comms, frequency)) {
    return rc;
  }
#ifndef _WIN32
  usleep(time*1000000);
#else
  Sleep(time*1000);
#endif
  if(rc = Mobot_setBuzzerFrequencyOff(comms)) {
    return rc;
  }
  return 0;
}

int Mobot_setBuzzerFrequencyOn(mobot_t* comms, unsigned int frequency)
{
  uint8_t buf[32];
  uint16_t f;
  f = frequency;
  int status;
  buf[0] = (f>>8) & 0x00ff;
  buf[1] = f & 0x00ff;
  status = MobotMsgTransaction(comms, BTCMD(CMD_BUZZERFREQ), buf, 2);
  if(status < 0) return status;
  /* Make sure the data size is correct */
  if(buf[1] != 3) {
    return -1;
  }
  return 0;
}

int Mobot_setBuzzerFrequencyOff(mobot_t* comms)
{
  return Mobot_setBuzzerFrequencyOn(comms, 0);
}

int Mobot_setExitState(mobot_t* comms, robotJointState_t exitState)
{
  comms->exitState = exitState;
  return 0;
}

int Mobot_setHWRev(mobot_t* comms, uint8_t rev)
{
  uint8_t buf[20];
  int status;
  buf[0] = rev;
  if(status = MobotMsgTransaction(comms, BTCMD(CMD_GETHWREV), buf, 1)) {
    return status;
  }
  if(buf[0] != RESP_OK) {
    return -1;
  }
  return 0;
}

int Mobot_setJointDirection(mobot_t* comms, robotJointId_t id, robotJointState_t dir)
{
  uint8_t buf[32];
  int status;
  buf[0] = (uint8_t)id-1;
  if(
      (comms->formFactor == MOBOTFORM_I) && 
      (id == ROBOT_JOINT3)
    )
  {
    switch (dir) {
      case ROBOT_FORWARD:
        buf[1] = ROBOT_BACKWARD;
        break;
      case ROBOT_BACKWARD:
        buf[1] = ROBOT_FORWARD;
        break;
      case ROBOT_POSITIVE:
        buf[1] = ROBOT_FORWARD;
        break;
      case ROBOT_NEGATIVE:
        buf[1] = ROBOT_BACKWARD;
        break;
      default:
        buf[1] = dir;
    }
  } else {
    buf[1] = (uint8_t) dir;
  }
  status = MobotMsgTransaction(comms, BTCMD(CMD_SETMOTORDIR), buf, 2);
  if(status < 0) return status;
  /* Make sure the data size is correct */
  if(buf[1] != 3) {
    return -1;
  }
  return 0;
}

int Mobot_setJointMovementStateNB(mobot_t* comms, robotJointId_t id, robotJointState_t dir)
{
  //Mobot_setJointSpeed(comms, id, comms->jointSpeeds[(int)id-1]);
  Mobot_setJointDirection(comms, id, dir);
  return 0;
}

int Mobot_setJointMovementStateTime(mobot_t* comms, robotJointId_t id, robotJointState_t dir, double seconds)
{
  int msecs = seconds * 1000;
  Mobot_moveJointContinuousNB(comms, id, dir);
#ifdef _WIN32
  Sleep(msecs);
#else
  usleep(msecs * 1000);
#endif
  return 0;
}

int Mobot_setJointSafetyAngle(mobot_t* comms, double angle)
{
  uint8_t buf[32];
  float f;
  int status;
  f = angle;
  memcpy(&buf[0], &f, 4);
  status = MobotMsgTransaction(comms, BTCMD(CMD_SETMOTORSAFETYLIMIT), buf, 4);
  if(status < 0) return status;
  /* Make sure the data size is correct */
  if(buf[1] != 3) {
    return -1;
  }
  return 0;
}

int Mobot_setJointSafetyAngleTimeout(mobot_t* comms, double seconds)
{
  uint8_t buf[32];
  float f;
  int status;
  f = seconds;
  memcpy(&buf[0], &f, 4);
  status = MobotMsgTransaction(comms, BTCMD(CMD_SETMOTORSAFETYTIMEOUT), buf, 4);
  if(status < 0) return status;
  /* Make sure the data size is correct */
  if(buf[1] != 3) {
    return -1;
  }
  return 0;
}

int Mobot_setJointSpeed(mobot_t* comms, robotJointId_t id, double speed)
{
  uint8_t buf[32];
  float f;
  int status;
  if(speed > comms->maxSpeed[id-1]) {
    fprintf(stderr, 
        "Warning: Cannot set speed for joint %d to %.2lf degrees/second, which is\n"
        "beyond the maximum limit, %.2lf degrees/second.\n",
        id, RAD2DEG(speed), RAD2DEG(comms->maxSpeed[id-1]));
  }
  f = speed;
  buf[0] = (uint8_t)id-1;
  memcpy(&buf[1], &f, 4);
  status = MobotMsgTransaction(comms, BTCMD(CMD_SETMOTORSPEED), buf, 5);
  if(status < 0) return status;
  /* Make sure the data size is correct */
  if(buf[1] != 3) {
    return -1;
  }
  comms->jointSpeeds[id-1] = speed;
  return 0;
}

int Mobot_setJointSpeedRatio(mobot_t* comms, robotJointId_t id, double ratio)
{
  if((ratio < 0) || (ratio > 1)) {
    return -1;
  }
  return Mobot_setJointSpeed(comms, id, ratio * comms->maxSpeed[(int)id-1]);
}

int Mobot_setJointSpeedRatios(mobot_t* comms, double ratio1, double ratio2, double ratio3, double ratio4)
{
  double ratios[4];
  int i;
  ratios[0] = ratio1;
  ratios[1] = ratio2;
  ratios[2] = ratio3;
  ratios[3] = ratio4;
  for(i = 0; i < 4; i++) {
    Mobot_setJointSpeedRatio(comms, (robotJointId_t)(i+1), ratios[i]);
  }
  return 0;
}

int Mobot_setMotorPower(mobot_t* comms, robotJointId_t id, int power)
{
  uint8_t buf[32];
  float f;
  int status;
  int16_t _power;
  memset(buf, 0, sizeof(uint8_t)*32);
  buf[0] = (1<<(id-1));
  _power = power;
  //memcpy(&buf[1+(id-1)*2], &_power, 2);
  buf[1+(id-1)*2] = _power>>8;
  buf[2+(id-1)*2] = (_power&0x00ff);
  status = MobotMsgTransaction(comms, BTCMD(CMD_SETMOTORPOWER), buf, 10);
  if(status < 0) return status;
  /* Make sure the data size is correct */
  if(buf[1] != 3) {
    return -1;
  }
  return 0;
}

int Mobot_setMovementStateNB(mobot_t* comms,
                                  robotJointState_t dir1,
                                  robotJointState_t dir2,
                                  robotJointState_t dir3,
                                  robotJointState_t dir4)
{
  int i;
  int32_t msecs = -1;
  uint8_t buf[64];
  int status;
  robotJointState_t dirs[4];
  dirs[0] = dir1; dirs[1] = dir2; dirs[2] = dir3; dirs[3] = dir4;
  buf[0] = 0x0F;
  if(comms->formFactor == MOBOTFORM_I) {
    switch(dir3) {
      case ROBOT_FORWARD:
        dirs[2] = ROBOT_BACKWARD;
        break;
      case ROBOT_BACKWARD:
        dirs[2] = ROBOT_FORWARD;
        break;
      case ROBOT_POSITIVE:
        dirs[2] = ROBOT_FORWARD;
        break;
      case ROBOT_NEGATIVE:
        dirs[2] = ROBOT_BACKWARD;
        break;
      default:
        dirs[2] = dir3;
    }
  }
  for(i = 0; i < 4; i++) {
    buf[i*6 + 1] = dirs[i];
    buf[i*6 + 2] = ROBOT_HOLD;
    memcpy(&buf[i*6 + 3], &msecs, 4);
  }
  status = MobotMsgTransaction(comms, BTCMD(CMD_TIMEDACTION), buf, 6*4 + 1);
  if(status < 0) return status;
  /* Make sure the data size is correct */
  if(buf[1] != 3) {
    return -1;
  }
  return 0;
}

int Mobot_setMovementStateTime(mobot_t* comms,
                                  robotJointState_t dir1,
                                  robotJointState_t dir2,
                                  robotJointState_t dir3,
                                  robotJointState_t dir4,
                                  double seconds)
{
  int32_t msecs = seconds * 1000;
  int rc = Mobot_setMovementStateTimeNB(comms,
      dir1,
      dir2,
      dir3,
      dir4,
      seconds);
  if(rc) return rc;
#ifdef _WIN32
  Sleep(msecs);
  Sleep(250);
#else
  usleep(msecs * 1000);
  usleep(250000);
#endif
  return 0;
}

int Mobot_setMovementStateTimeNB(mobot_t* comms,
                                  robotJointState_t dir1,
                                  robotJointState_t dir2,
                                  robotJointState_t dir3,
                                  robotJointState_t dir4,
                                  double seconds)
{
  int i;
  int32_t msecs = seconds * 1000;
  uint8_t buf[64];
  int status;
  robotJointState_t dirs[4];
  dirs[0] = dir1; dirs[1] = dir2; dirs[2] = dir3; dirs[3] = dir4;
  if(comms->formFactor == MOBOTFORM_I) {
    switch(dir3) {
      case ROBOT_FORWARD:
        dirs[2] = ROBOT_BACKWARD;
        break;
      case ROBOT_BACKWARD:
        dirs[2] = ROBOT_FORWARD;
        break;
      case ROBOT_POSITIVE:
        dirs[2] = ROBOT_FORWARD;
        break;
      case ROBOT_NEGATIVE:
        dirs[2] = ROBOT_BACKWARD;
        break;
      default:
        dirs[2] = dir3;
    }
  }
  buf[0] = 0x0F;
  for(i = 0; i < 4; i++) {
    buf[i*6 + 1] = dirs[i];
    buf[i*6 + 2] = ROBOT_HOLD;
    memcpy(&buf[i*6 + 3], &msecs, 4);
  }
  status = MobotMsgTransaction(comms, BTCMD(CMD_TIMEDACTION), buf, 6*4 + 1);
  if(status < 0) return status;
  /* Make sure the data size is correct */
  if(buf[1] != 3) {
    return -1;
  }
  return 0;
}

int Mobot_setColorRGB(mobot_t* comms, int r, int g, int b)
{
  uint8_t buf[32];
  float f;
  int status;
  buf[0] = 0xff;
  buf[1] = 0xff;
  buf[2] = 0xff;
  buf[3] = (uint8_t)r;
  buf[4] = (uint8_t)g;
  buf[5] = (uint8_t)b;

  status = MobotMsgTransaction(comms, BTCMD(CMD_RGBLED), buf, 6);
  if(status < 0) return status;
  /* Make sure the data size is correct */
  if(buf[1] != 3) {
    return -1;
  }
  return 0;
}

int Mobot_setColor(mobot_t* comms, char * color)
{
  uint8_t buf[32];
  int status;
  int htRetval;
  int getRGB[3];
  rgbHashTable * rgbTable = HT_Create();

  htRetval = HT_Get(rgbTable, color, getRGB);

  HT_Destroy(rgbTable);

  if(htRetval > 0) //HT_Get() will return 1 if the color is in the hash table, otherwise -1 is returned.
  {
	buf[0] = 0xff;
	buf[1] = 0xff;
	buf[2] = 0xff;
	buf[3] = (uint8_t)getRGB[0];
	buf[4] = (uint8_t)getRGB[1];
	buf[5] = (uint8_t)getRGB[2];

	status = MobotMsgTransaction(comms, BTCMD(CMD_RGBLED), buf, 6);
	if(status < 0) return status;
	/* Make sure the data size is correct */
	if(buf[1] != 3){
	  return -1;
	}
	return 0;
  }else{
	printf("Error: color '%s' is undefined.\n", color);
	return htRetval;
  }
}

int Mobot_setTwoWheelRobotSpeed(mobot_t* comms, double speed, double radius)
{
  double omega;
  omega = speed/radius;
  if(comms->formFactor == MOBOTFORM_ORIGINAL) {
    Mobot_setJointSpeed(comms, ROBOT_JOINT1, omega);
    Mobot_setJointSpeed(comms, ROBOT_JOINT4, omega);
  } else if (comms->formFactor == MOBOTFORM_I) {
    Mobot_setJointSpeed(comms, ROBOT_JOINT1, omega);
    Mobot_setJointSpeed(comms, ROBOT_JOINT3, omega);
  } else {
    return -1;
  }
  return 0;
}

int Mobot_setJointSpeeds(mobot_t* comms, double speed1, double speed2, double speed3, double speed4)
{
  double speeds[4];
  int i;
  speeds[0] = speed1;
  speeds[1] = speed2;
  speeds[2] = speed3;
  speeds[3] = speed4;
  for(i = 0; i < 4; i++) {
    Mobot_setJointSpeed(comms, (robotJointId_t)(i+1), speeds[i]);
  }
  return 0;
}

