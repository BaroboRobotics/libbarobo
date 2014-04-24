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

#define DEPRECATED(from, to) \
  fprintf(stderr, "Warning: The function \"%s()\" is deprecated. Please use \"%s()\"\n" , from, to)

int Mobot_isMoving(mobot_t* comms)
{
  int moving = 0;
  robotJointState_t state;
  int i;
  for(i = 1; i <= 4; i++) {
    Mobot_getJointState(comms, (robotJointId_t)i, &state);
    if( (state == ROBOT_FORWARD) ||
        (state == ROBOT_BACKWARD) ||
        (state == ROBOT_ACCEL) 
     ) 
    {
      moving = 1;
      break;
    }
  }
  return moving;
}

int Mobot_move(mobot_t* comms,
                               double angle1,
                               double angle2,
                               double angle3,
                               double angle4)
{
  Mobot_moveNB(comms, 
      angle1,
      angle2,
      angle3,
      angle4);

  /* Wait for the motion to complete */
  return Mobot_moveWait(comms);
}

int Mobot_moveNB(mobot_t* comms,
                               double angle1,
                               double angle2,
                               double angle3,
                               double angle4)
{
  double angles[4];
  double curAngles[4];
  int i;
  double time;
  uint8_t buf[32];
  float f;
  angles[0] = angle1;
  angles[1] = angle2;
  angles[2] = angle3;
  angles[3] = angle4;
  /* Get the current joint angles */
  Mobot_getJointAnglesTime(comms, &time, 
      &curAngles[0],
      &curAngles[1],
      &curAngles[2],
      &curAngles[3] );
  /* Calculate new angles */
  for(i = 0; i < 4; i++) {
    angles[i] = curAngles[i] + angles[i];
  }
  /* Set up message buffer */
  for(i = 0; i < 4; i++) {
    f = angles[i];
    memcpy(&buf[i*4], &f, 4);
  }
  if(MobotMsgTransaction(comms, BTCMD(CMD_SETMOTORANGLESABS), buf, 4*4)) {
    return -1;
  }
  /* Make sure the data size is correct */
  if(buf[1] != 0x03) {
    return -1;
  }
  return 0;
}

int Mobot_moveBackward(mobot_t* comms, double angle)
{
  int rc;
  rc = Mobot_moveBackwardNB(comms, angle);
  if(rc) return rc;
  return Mobot_moveWait(comms);
}

int Mobot_moveBackwardNB(mobot_t* comms, double angle)
{
  switch(comms->formFactor) {
    case MOBOTFORM_ORIGINAL:
      return Mobot_moveNB(comms, -angle, 0, 0, -angle);
    case MOBOTFORM_I:
      return Mobot_moveNB(comms, -angle, 0, angle, 0);
    default:
      return -1;
  }
}

int Mobot_moveDistance(mobot_t* comms, double distance, double radius)
{
  double theta;
  theta = distance/radius;
  return Mobot_moveForward(comms, theta);
}

int Mobot_moveDistanceNB(mobot_t* comms, double distance, double radius)
{
  double theta;
  theta = distance/radius;
  return Mobot_moveForwardNB(comms, theta);
}

int Mobot_moveForward(mobot_t* comms, double angle)
{
  int rc;
  rc = Mobot_moveForwardNB(comms, angle);
  if(rc) return rc;
  return Mobot_moveWait(comms);
}

int Mobot_moveForwardNB(mobot_t* comms, double angle)
{
  switch(comms->formFactor) {
    case MOBOTFORM_ORIGINAL:
      return Mobot_moveNB(comms, angle, 0, 0, angle);
    case MOBOTFORM_I:
      return Mobot_moveNB(comms, angle, 0, -angle, 0);
    default:
      return -1;
  }
}

int Mobot_moveContinuousNB(mobot_t* comms,
                                  robotJointState_t dir1,
                                  robotJointState_t dir2,
                                  robotJointState_t dir3,
                                  robotJointState_t dir4)
{
  return Mobot_setMovementStateNB(comms, dir1, dir2, dir3, dir4);
}

int Mobot_moveContinuousTime(mobot_t* comms,
                                  robotJointState_t dir1,
                                  robotJointState_t dir2,
                                  robotJointState_t dir3,
                                  robotJointState_t dir4,
                                  double seconds)
{
  return Mobot_setMovementStateTime(comms, dir1, dir2, dir3, dir4, seconds);
}

int Mobot_moveJointContinuousNB(mobot_t* comms, robotJointId_t id, robotJointState_t dir)
{
  return Mobot_setJointMovementStateNB(comms, id, dir);
}

int Mobot_moveJointContinuousTime(mobot_t* comms, robotJointId_t id, robotJointState_t dir, double seconds)
{
  return Mobot_setJointMovementStateTime(comms, id, dir, seconds);
}

int Mobot_moveJoint(mobot_t* comms, robotJointId_t id, double angle)
{
  double curAngle;
  if(Mobot_getJointAngle(comms, id, &curAngle)) {
    return -1;
  }
  return Mobot_moveJointTo(comms, id, curAngle + angle);
}

int Mobot_moveJointNB(mobot_t* comms, robotJointId_t id, double angle)
{
  double curAngle;
  if(Mobot_getJointAngle(comms, id, &curAngle)) {
    return -1;
  }
  return Mobot_moveJointToNB(comms, id, curAngle + angle);
}

int Mobot_moveJointTo(mobot_t* comms, robotJointId_t id, double angle)
{
  Mobot_moveJointToNB(comms, id, angle);
  /* Wait for the motion to finish */
  return Mobot_moveJointWait(comms, id);
}

int Mobot_moveJointToDirect(mobot_t* comms, robotJointId_t id, double angle)
{
  Mobot_moveJointToDirectNB(comms, id, angle);
  /* Wait for the motion to finish */
  return Mobot_moveJointWait(comms, id);
}

int Mobot_driveJointToDirect(mobot_t* comms, robotJointId_t id, double angle)
{
  Mobot_driveJointToDirectNB(comms, id, angle);
  return Mobot_moveWait(comms);
}

int Mobot_driveJointTo(mobot_t* comms, robotJointId_t id, double angle)
{
  Mobot_driveJointToDirectNB(comms, id, angle);
  return Mobot_moveWait(comms);
}

int Mobot_driveJointToDirectNB(mobot_t* comms, robotJointId_t id, double angle)
{
  uint8_t buf[32];
  float f;
  int status;
  buf[0] = (uint8_t)id-1;
  f = angle;
  memcpy(&buf[1], &f, 4);
  status = MobotMsgTransaction(comms, BTCMD(CMD_SETMOTORANGLEPID), buf, 5);
  if(status < 0) return status;
  /* Make sure the data size is correct */
  if(buf[1] != 3) {
    return -1;
  }
  return 0;
}

int Mobot_driveJointToNB(mobot_t* comms, robotJointId_t id, double angle)
{
  uint8_t buf[32];
  float f;
  int status;
  buf[0] = (uint8_t)id-1;
  f = angle;
  memcpy(&buf[1], &f, 4);
  status = MobotMsgTransaction(comms, BTCMD(CMD_SETMOTORANGLEPID), buf, 5);
  if(status < 0) return status;
  /* Make sure the data size is correct */
  if(buf[1] != 3) {
    return -1;
  }
  return 0;
}

int Mobot_moveJointToNB(mobot_t* comms, robotJointId_t id, double angle)
{
  uint8_t buf[32];
  float f;
  int status;
  if((id == ROBOT_JOINT2) || (id == ROBOT_JOINT3)) {
    if(angle > 90) {
      fprintf(stderr, "Warning: Angle for joint %d set beyond limits.\n", (int)(id + 1));
      angle = 90;
    }
    if(angle < -90) {
      fprintf(stderr, "Warning: Angle for joint %d set beyond limits.\n", (int)(id + 1));
      angle = -90;
    }
  }
  buf[0] = (uint8_t)id-1;
  f = angle;
  memcpy(&buf[1], &f, 4);
  status = MobotMsgTransaction(comms, BTCMD(CMD_SETMOTORANGLEABS), buf, 5);
  if(status < 0) return status;
  /* Make sure the data size is correct */
  if(buf[1] != 3) {
    return -1;
  }
  return 0;
}

int Mobot_moveJointToDirectNB(mobot_t* comms, robotJointId_t id, double angle)
{
  uint8_t buf[32];
  float f;
  int status;
  if((id == ROBOT_JOINT2) || (id == ROBOT_JOINT3)) {
    if(angle > 90) {
      fprintf(stderr, "Warning: Angle for joint %d set beyond limits.\n", (int)(id + 1));
      angle = 90;
    }
    if(angle < -90) {
      fprintf(stderr, "Warning: Angle for joint %d set beyond limits.\n", (int)(id + 1));
      angle = -90;
    }
  }
  buf[0] = (uint8_t)id-1;
  f = angle;
  memcpy(&buf[1], &f, 4);
  status = MobotMsgTransaction(comms, BTCMD(CMD_SETMOTORANGLEDIRECT), buf, 5);
  if(status < 0) return status;
  /* Make sure the data size is correct */
  if(buf[1] != 3) {
    return -1;
  }
  return 0;
}

int Mobot_moveJointWait(mobot_t* comms, robotJointId_t id)
{
  robotJointState_t state;
  /* Make sure there is no non-blocking function running */
//  THREAD_JOIN(comms->thread);

  while(1)
  {
    if(Mobot_getJointState(comms, id, &state)) {
      return -1;
    }
    if(
      (state == ROBOT_NEUTRAL) ||
      (state == ROBOT_HOLD) )
    {
      return 0;
    } else {
#ifndef _WIN32
      usleep(200000);
#else
      Sleep(500);
#endif
    }
  }
  return 0;
}

int Mobot_moveToZero(mobot_t* comms)
{
  return Mobot_moveTo(comms, 0, 0, 0, 0);
}

int Mobot_moveToZeroNB(mobot_t* comms)
{
  return Mobot_moveToNB(comms, 0, 0, 0, 0);
}

int Mobot_moveTo(mobot_t* comms,
                               double angle1,
                               double angle2,
                               double angle3,
                               double angle4)
{
  Mobot_moveToNB(comms, 
      angle1, 
      angle2, 
      angle3, 
      angle4 );
  return Mobot_moveWait(comms);
}

int Mobot_moveToDirect(mobot_t* comms,
                               double angle1,
                               double angle2,
                               double angle3,
                               double angle4)
{
  Mobot_moveToDirectNB(comms, 
      angle1, 
      angle2, 
      angle3, 
      angle4 );
  return Mobot_moveWait(comms);
}

int Mobot_driveToDirect(mobot_t* comms,
                               double angle1,
                               double angle2,
                               double angle3,
                               double angle4)
{
  Mobot_driveToDirectNB(comms, 
      angle1, 
      angle2, 
      angle3, 
      angle4 );
  return Mobot_moveWait(comms);
}

int Mobot_driveTo(mobot_t* comms,
                               double angle1,
                               double angle2,
                               double angle3,
                               double angle4)
{
  Mobot_driveToDirectNB(comms, 
      angle1, 
      angle2, 
      angle3, 
      angle4 );
  return Mobot_moveWait(comms);
}

int Mobot_moveToNB(mobot_t* comms,
                               double angle1,
                               double angle2,
                               double angle3,
                               double angle4)
{
  uint8_t buf[32];
  float f;
  int status;
  f = angle1;
  memcpy(&buf[0], &f, 4);
  f = angle2;
  memcpy(&buf[4], &f, 4);
  f = angle3;
  memcpy(&buf[8], &f, 4);
  f = angle4;
  memcpy(&buf[12], &f, 4);
  status = MobotMsgTransaction(comms, BTCMD(CMD_SETMOTORANGLESABS), buf, 16);
  if(status < 0) return status;
  /* Make sure the data size is correct */
  if(buf[1] != 3) {
    return -1;
  }
  return 0;
}

int Mobot_moveToDirectNB(mobot_t* comms,
                               double angle1,
                               double angle2,
                               double angle3,
                               double angle4)
{
  uint8_t buf[32];
  float f;
  int status;
  f = angle1;
  memcpy(&buf[0], &f, 4);
  f = angle2;
  memcpy(&buf[4], &f, 4);
  f = angle3;
  memcpy(&buf[8], &f, 4);
  f = angle4;
  memcpy(&buf[12], &f, 4);
  status = MobotMsgTransaction(comms, BTCMD(CMD_SETMOTORANGLESDIRECT), buf, 16);
  if(status < 0) return status;
  /* Make sure the data size is correct */
  if(buf[1] != 3) {
    return -1;
  }
  return 0;
}


int Mobot_driveToDirectNB(mobot_t* comms,
                               double angle1,
                               double angle2,
                               double angle3,
                               double angle4)
{
  uint8_t buf[32];
  float f;
  int status;
  f = angle1;
  memcpy(&buf[0], &f, 4);
  f = angle2;
  memcpy(&buf[4], &f, 4);
  f = angle3;
  memcpy(&buf[8], &f, 4);
  f = angle4;
  memcpy(&buf[12], &f, 4);
  status = MobotMsgTransaction(comms, BTCMD(CMD_SETMOTORANGLESPID), buf, 16);
  if(status < 0) return status;
  /* Make sure the data size is correct */
  if(buf[1] != 3) {
    return -1;
  }
  return 0;
}

int Mobot_driveToNB(mobot_t* comms,
                               double angle1,
                               double angle2,
                               double angle3,
                               double angle4)
{
  uint8_t buf[32];
  float f;
  int status;
  f = angle1;
  memcpy(&buf[0], &f, 4);
  f = angle2;
  memcpy(&buf[4], &f, 4);
  f = angle3;
  memcpy(&buf[8], &f, 4);
  f = angle4;
  memcpy(&buf[12], &f, 4);
  status = MobotMsgTransaction(comms, BTCMD(CMD_SETMOTORANGLESPID), buf, 16);
  if(status < 0) return status;
  /* Make sure the data size is correct */
  if(buf[1] != 3) {
    return -1;
  }
  return 0;
}

int Mobot_moveWait(mobot_t* comms)
{
  int i;
  if(comms->formFactor == MOBOTFORM_I) {
    Mobot_moveJointWait(comms, ROBOT_JOINT1);
    Mobot_moveJointWait(comms, ROBOT_JOINT3);
  } else if (comms->formFactor == MOBOTFORM_L) {
    Mobot_moveJointWait(comms, ROBOT_JOINT1);
    Mobot_moveJointWait(comms, ROBOT_JOINT2);
  } else {
    for(i = 0; i < 4; i++) {
      if(Mobot_moveJointWait(comms, (robotJointId_t)(i+1))) {
        return -1;
      }
    }
  }
  return 0;
}

int Mobot_movexy(mobot_t* comms, double x, double y, double radius, double trackwidth)
{
  /* First, turn the robot to face the goal position */
  double angle;
  angle = atan2(y, x);
  if(angle > 0) {
    Mobot_turnLeft(comms, angle, radius, trackwidth);
  } else {
    Mobot_turnRight(comms, -angle, radius, trackwidth);
  }
  /* Go forward the correct amount of distance */
  double distance;
  distance = sqrt( x*x + y*y );
  Mobot_moveDistance(comms, distance, radius);
  return 0;
}

void* Mobot_movexyThread(void* arg)
{
  mobot_t* comms = (mobot_t*)arg;
  double* args = &(((double*)arg)[1]);
  Mobot_movexy(comms, args[0], args[1], args[2], args[3]);
  comms->motionInProgress--;
  return NULL;
}

int Mobot_movexyNB(mobot_t* comms, double x, double y, double radius, double trackwidth)
{
  static void* args[5];
  static double _x, _y, _r, _t;
  _x = x;
  _y = y;
  _r = radius;
  _t = trackwidth;
  args[0] = (void*)comms;
  args[1] = (void*)&_x;
  args[2] = (void*)&_y;
  args[3] = (void*)&_r;
  args[4] = (void*)&_t;
  comms->motionInProgress++;
  THREAD_CREATE(comms->thread, Mobot_movexyThread, args);
  return 0;
}

int Mobot_beginFourierControl(mobot_t* comms, uint8_t motorMask)
{
  uint8_t buf[32];
  int status;
  buf[0] = motorMask;
  status = MobotMsgTransaction(comms, BTCMD(CMD_STARTFOURIER), buf, 1);
  if(status < 0) return status;
  /* Make sure the data size is correct */
  if(buf[1] != 3) {
    return -1;
  }
  return 0;
}

int Mobot_stop(mobot_t* comms)
{
  uint8_t buf[32];
  float f;
  int status;
  status = MobotMsgTransaction(comms, BTCMD(CMD_STOP), buf, 0);
  if(status < 0) return status;
  /* Make sure the data size is correct */
  if(buf[1] != 3) {
    return -1;
  }
  return 0;
}

int Mobot_stopOneJoint(mobot_t* comms, robotJointId_t id)
{
  return Mobot_moveJointContinuousNB(comms, id, ROBOT_NEUTRAL);
}

int Mobot_stopTwoJoints(mobot_t* comms, robotJointId_t id1, robotJointId_t id2)
{
  Mobot_moveJointContinuousNB(comms, id1, ROBOT_NEUTRAL);
  return Mobot_moveJointContinuousNB(comms, id2, ROBOT_NEUTRAL);
}

int Mobot_stopThreeJoints(mobot_t* comms, robotJointId_t id1, robotJointId_t id2, robotJointId_t id3)
{
  Mobot_moveJointContinuousNB(comms, id1, ROBOT_NEUTRAL);
  Mobot_moveJointContinuousNB(comms, id2, ROBOT_NEUTRAL);
  return Mobot_moveJointContinuousNB(comms, id3, ROBOT_NEUTRAL);
}

int Mobot_stopAllJoints(mobot_t* comms)
{
  return Mobot_stop(comms);
}

int Mobot_turnLeft(mobot_t* comms, double angle, double radius, double tracklength)
{
  int rc;
  if(rc = Mobot_turnLeftNB(comms, angle, radius, tracklength)) {
    return rc;
  }
  return Mobot_moveWait(comms);
}

int Mobot_turnLeftNB(mobot_t* comms, double angle, double radius, double tracklength)
{
  double theta;
  theta = (angle*tracklength)/(2*radius);
  switch(comms->formFactor) {
    case MOBOTFORM_ORIGINAL:
      return Mobot_moveNB(comms, -theta, 0, 0, theta);
    case MOBOTFORM_I:
      return Mobot_moveNB(comms, -theta, 0, -theta, 0);
    default:
      return -1;
  }
}

int Mobot_turnRight(mobot_t* comms, double angle, double radius, double tracklength)
{
  int rc;
  if(rc = Mobot_turnRightNB(comms, angle, radius, tracklength)) {
    return rc;
  }
  return Mobot_moveWait(comms);
}

int Mobot_turnRightNB(mobot_t* comms, double angle, double radius, double tracklength)
{
  double theta;
  theta = (angle*tracklength)/(2*radius);
  switch(comms->formFactor) {
    case MOBOTFORM_ORIGINAL:
      return Mobot_moveNB(comms, theta, 0, 0, -theta);
    case MOBOTFORM_I:
      return Mobot_moveNB(comms, theta, 0, theta, 0);
    default:
      return -1;
  }
}
