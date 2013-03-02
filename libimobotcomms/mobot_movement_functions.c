
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
  mobotJointState_t state;
  int i;
  for(i = 1; i <= 4; i++) {
    Mobot_getJointState(comms, (mobotJointId_t)i, &state);
    if( (state == MOBOT_FORWARD) ||
        (state == MOBOT_BACKWARD) ) 
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
  SendToIMobot(comms, BTCMD(CMD_SETMOTORANGLESABS), buf, 4*4);
  if(RecvFromIMobot(comms, buf, sizeof(buf))) {
    return -1;
  }
  /* Make sure the data size is correct */
  if(buf[1] != 0x03) {
    return -1;
  }
  return 0;
}

int Mobot_moveContinuousNB(mobot_t* comms,
                                  mobotJointState_t dir1,
                                  mobotJointState_t dir2,
                                  mobotJointState_t dir3,
                                  mobotJointState_t dir4)
{
  return Mobot_setMovementStateNB(comms, dir1, dir2, dir3, dir4);
}

int Mobot_moveContinuousTime(mobot_t* comms,
                                  mobotJointState_t dir1,
                                  mobotJointState_t dir2,
                                  mobotJointState_t dir3,
                                  mobotJointState_t dir4,
                                  double seconds)
{
  return Mobot_setMovementStateTime(comms, dir1, dir2, dir3, dir4, seconds);
}

int Mobot_moveJointContinuousNB(mobot_t* comms, mobotJointId_t id, mobotJointState_t dir)
{
  return Mobot_setJointMovementStateNB(comms, id, dir);
}

int Mobot_moveJointContinuousTime(mobot_t* comms, mobotJointId_t id, mobotJointState_t dir, double seconds)
{
  return Mobot_setJointMovementStateTime(comms, id, dir, seconds);
}

int Mobot_moveJoint(mobot_t* comms, mobotJointId_t id, double angle)
{
  double curAngle;
  if(Mobot_getJointAngle(comms, id, &curAngle)) {
    return -1;
  }
  return Mobot_moveJointTo(comms, id, curAngle + angle);
}

int Mobot_moveJointNB(mobot_t* comms, mobotJointId_t id, double angle)
{
  double curAngle;
  if(Mobot_getJointAngle(comms, id, &curAngle)) {
    return -1;
  }
  return Mobot_moveJointToNB(comms, id, curAngle + angle);
}

int Mobot_moveJointTo(mobot_t* comms, mobotJointId_t id, double angle)
{
  Mobot_moveJointToNB(comms, id, angle);
  /* Wait for the motion to finish */
  return Mobot_moveJointWait(comms, id);
}

int Mobot_moveJointToDirect(mobot_t* comms, mobotJointId_t id, double angle)
{
  Mobot_moveJointToDirectNB(comms, id, angle);
  /* Wait for the motion to finish */
  return Mobot_moveJointWait(comms, id);
}

int Mobot_driveJointToDirect(mobot_t* comms, mobotJointId_t id, double angle)
{
  Mobot_driveJointToDirectNB(comms, id, angle);
  return Mobot_moveWait(comms);
}

int Mobot_driveJointTo(mobot_t* comms, mobotJointId_t id, double angle)
{
  Mobot_driveJointToDirectNB(comms, id, angle);
  return Mobot_moveWait(comms);
}

int Mobot_driveJointToDirectNB(mobot_t* comms, mobotJointId_t id, double angle)
{
  uint8_t buf[32];
  float f;
  int status;
  buf[0] = (uint8_t)id-1;
  f = angle;
  memcpy(&buf[1], &f, 4);
  status = SendToIMobot(comms, BTCMD(CMD_SETMOTORANGLEPID), buf, 5);
  if(status < 0) return status;
  if(RecvFromIMobot(comms, buf, sizeof(buf))) {
    return -1;
  }
  /* Make sure the data size is correct */
  if(buf[1] != 3) {
    return -1;
  }
  return 0;
}

int Mobot_driveJointToNB(mobot_t* comms, mobotJointId_t id, double angle)
{
  uint8_t buf[32];
  float f;
  int status;
  buf[0] = (uint8_t)id-1;
  f = angle;
  memcpy(&buf[1], &f, 4);
  status = SendToIMobot(comms, BTCMD(CMD_SETMOTORANGLEPID), buf, 5);
  if(status < 0) return status;
  if(RecvFromIMobot(comms, buf, sizeof(buf))) {
    return -1;
  }
  /* Make sure the data size is correct */
  if(buf[1] != 3) {
    return -1;
  }
  return 0;
}

int Mobot_moveJointToNB(mobot_t* comms, mobotJointId_t id, double angle)
{
  uint8_t buf[32];
  float f;
  int status;
  if((id == MOBOT_JOINT2) || (id == MOBOT_JOINT3)) {
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
  status = SendToIMobot(comms, BTCMD(CMD_SETMOTORANGLEABS), buf, 5);
  if(status < 0) return status;
  if(RecvFromIMobot(comms, buf, sizeof(buf))) {
    return -1;
  }
  /* Make sure the data size is correct */
  if(buf[1] != 3) {
    return -1;
  }
  return 0;
}

int Mobot_moveJointToDirectNB(mobot_t* comms, mobotJointId_t id, double angle)
{
  uint8_t buf[32];
  float f;
  int status;
  if((id == MOBOT_JOINT2) || (id == MOBOT_JOINT3)) {
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
  status = SendToIMobot(comms, BTCMD(CMD_SETMOTORANGLEDIRECT), buf, 5);
  if(status < 0) return status;
  if(RecvFromIMobot(comms, buf, sizeof(buf))) {
    return -1;
  }
  /* Make sure the data size is correct */
  if(buf[1] != 3) {
    return -1;
  }
  return 0;
}

int Mobot_moveJointWait(mobot_t* comms, mobotJointId_t id)
{
  mobotJointState_t state;
  /* Make sure there is no non-blocking function running */
//  THREAD_JOIN(comms->thread);

  while(1)
  {
    if(Mobot_getJointState(comms, id, &state)) {
      return -1;
    }
    if(
      (state == MOBOT_NEUTRAL) ||
      (state == MOBOT_HOLD) )
    {
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
  status = SendToIMobot(comms, BTCMD(CMD_SETMOTORANGLESABS), buf, 16);
  if(status < 0) return status;
  if(RecvFromIMobot(comms, buf, sizeof(buf))) {
    return -1;
  }
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
  status = SendToIMobot(comms, BTCMD(CMD_SETMOTORANGLESDIRECT), buf, 16);
  if(status < 0) return status;
  if(RecvFromIMobot(comms, buf, sizeof(buf))) {
    return -1;
  }
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
  status = SendToIMobot(comms, BTCMD(CMD_SETMOTORANGLESPID), buf, 16);
  if(status < 0) return status;
  if(RecvFromIMobot(comms, buf, sizeof(buf))) {
    return -1;
  }
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
  status = SendToIMobot(comms, BTCMD(CMD_SETMOTORANGLESPID), buf, 16);
  if(status < 0) return status;
  if(RecvFromIMobot(comms, buf, sizeof(buf))) {
    return -1;
  }
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
    Mobot_moveJointWait(comms, MOBOT_JOINT1);
    Mobot_moveJointWait(comms, MOBOT_JOINT3);
  } else if (comms->formFactor == MOBOTFORM_L) {
    Mobot_moveJointWait(comms, MOBOT_JOINT1);
    Mobot_moveJointWait(comms, MOBOT_JOINT2);
  } else {
    for(i = 0; i < 4; i++) {
      if(Mobot_moveJointWait(comms, (mobotJointId_t)(i+1))) {
        return -1;
      }
    }
  }
  return 0;
}

int Mobot_beginFourierControl(mobot_t* comms, uint8_t motorMask)
{
  uint8_t buf[32];
  int status;
  buf[0] = motorMask;
  status = SendToIMobot(comms, BTCMD(CMD_STARTFOURIER), buf, 1);
  if(status < 0) return status;
  if(RecvFromIMobot(comms, buf, sizeof(buf))) {
    return -1;
  }
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
  status = SendToIMobot(comms, BTCMD(CMD_STOP), NULL, 0);
  if(status < 0) return status;
  if(RecvFromIMobot(comms, buf, sizeof(buf))) {
    return -1;
  }
  /* Make sure the data size is correct */
  if(buf[1] != 3) {
    return -1;
  }
  return 0;
}

int Mobot_stopOneJoint(mobot_t* comms, mobotJointId_t id)
{
  return Mobot_moveJointContinuousNB(comms, id, MOBOT_NEUTRAL);
}

int Mobot_stopTwoJoints(mobot_t* comms, mobotJointId_t id1, mobotJointId_t id2)
{
  Mobot_moveJointContinuousNB(comms, id1, MOBOT_NEUTRAL);
  return Mobot_moveJointContinuousNB(comms, id2, MOBOT_NEUTRAL);
}

int Mobot_stopThreeJoints(mobot_t* comms, mobotJointId_t id1, mobotJointId_t id2, mobotJointId_t id3)
{
  Mobot_moveJointContinuousNB(comms, id1, MOBOT_NEUTRAL);
  Mobot_moveJointContinuousNB(comms, id2, MOBOT_NEUTRAL);
  return Mobot_moveJointContinuousNB(comms, id3, MOBOT_NEUTRAL);
}

int Mobot_stopAllJoints(mobot_t* comms)
{
  return Mobot_stop(comms);
}

