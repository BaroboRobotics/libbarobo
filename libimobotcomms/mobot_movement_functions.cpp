
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
  /* Make sure there is no non-blocking function running */
//  THREAD_JOIN(comms->thread);
  for(i = 0; i < 4; i++) {
    if(Mobot_moveJointWait(comms, (mobotJointId_t)(i+1))) {
      return -1;
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

/* C + + */
#ifdef __cplusplus

int CMobot::driveJointToDirect(mobotJointId_t id, double angle)
{
  return Mobot_driveJointToDirect(_comms, id, DEG2RAD(angle));
}

int CMobot::driveJointTo(mobotJointId_t id, double angle)
{
  return Mobot_driveJointToDirect(_comms, id, DEG2RAD(angle));
}

int CMobot::driveJointToDirectNB(mobotJointId_t id, double angle)
{
  return Mobot_driveJointToDirectNB(_comms, id, DEG2RAD(angle));
}

int CMobot::driveJointToNB(mobotJointId_t id, double angle)
{
  return Mobot_driveJointToDirectNB(_comms, id, DEG2RAD(angle));
}

int CMobot::driveToDirect( double angle1,
                          double angle2,
                          double angle3,
                          double angle4)
{
  return Mobot_driveToDirect(
      _comms, 
      DEG2RAD(angle1), 
      DEG2RAD(angle2), 
      DEG2RAD(angle3), 
      DEG2RAD(angle4));
}

int CMobot::driveTo( double angle1,
                          double angle2,
                          double angle3,
                          double angle4)
{
  return Mobot_driveToDirect(
      _comms, 
      DEG2RAD(angle1), 
      DEG2RAD(angle2), 
      DEG2RAD(angle3), 
      DEG2RAD(angle4));
}

int CMobot::driveToDirectNB( double angle1,
                          double angle2,
                          double angle3,
                          double angle4)
{
  return Mobot_driveToDirectNB(
      _comms, 
      DEG2RAD(angle1), 
      DEG2RAD(angle2), 
      DEG2RAD(angle3), 
      DEG2RAD(angle4));
}

int CMobot::driveToNB( double angle1,
                          double angle2,
                          double angle3,
                          double angle4)
{
  return Mobot_driveToDirectNB(
      _comms, 
      DEG2RAD(angle1), 
      DEG2RAD(angle2), 
      DEG2RAD(angle3), 
      DEG2RAD(angle4));
}

int CMobot::isMoving()
{
  return Mobot_isMoving(_comms);
}

int CMobot::move( double angle1,
                        double angle2,
                        double angle3,
                        double angle4)
{
  return Mobot_move(
      _comms, 
      DEG2RAD(angle1), 
      DEG2RAD(angle2), 
      DEG2RAD(angle3), 
      DEG2RAD(angle4));
}

int CMobot::moveNB( double angle1,
                        double angle2,
                        double angle3,
                        double angle4)
{
  return Mobot_moveNB(
      _comms, 
      DEG2RAD(angle1), 
      DEG2RAD(angle2), 
      DEG2RAD(angle3), 
      DEG2RAD(angle4));
}

int CMobot::moveContinuousNB( mobotJointState_t dir1, mobotJointState_t dir2, mobotJointState_t dir3, mobotJointState_t dir4)
{
  DEPRECATED("moveContinuousNB", "setMovementStateNB");
  return Mobot_moveContinuousNB(_comms, dir1, dir2, dir3, dir4);
}

int CMobot::moveContinuousTime( mobotJointState_t dir1, mobotJointState_t dir2, mobotJointState_t dir3, mobotJointState_t dir4, double seconds)
{
  DEPRECATED("moveContinuousTime", "setMovementStateTime");
  return Mobot_moveContinuousTime(_comms, dir1, dir2, dir3, dir4, seconds);
}

int CMobot::moveJointContinuousNB(mobotJointId_t id, mobotJointState_t dir)
{
  DEPRECATED("moveJointContinuousNB", "setJointMovementStateNB");
  return Mobot_moveJointContinuousNB(_comms, id, dir);
}

int CMobot::moveJointContinuousTime(mobotJointId_t id, mobotJointState_t dir, double seconds)
{
  DEPRECATED("moveJointContinuousTime", "setJointMovementStateTime");
  return Mobot_moveJointContinuousTime(_comms, id, dir, seconds);
}

int CMobot::moveJoint(mobotJointId_t id, double angle)
{
  return Mobot_moveJoint(_comms, id, DEG2RAD(angle));
}

int CMobot::moveJointNB(mobotJointId_t id, double angle)
{
  return Mobot_moveJointNB(_comms, id, DEG2RAD(angle));
}

int CMobot::moveJointTo(mobotJointId_t id, double angle)
{
  return Mobot_moveJointTo(_comms, id, DEG2RAD(angle));
}

int CMobot::moveJointToDirect(mobotJointId_t id, double angle)
{
  return Mobot_moveJointToDirect(_comms, id, DEG2RAD(angle));
}

int CMobot::moveJointToNB(mobotJointId_t id, double angle)
{
  return Mobot_moveJointToNB(_comms, id, DEG2RAD(angle));
}

int CMobot::moveJointToDirectNB(mobotJointId_t id, double angle)
{
  return Mobot_moveJointToDirectNB(_comms, id, DEG2RAD(angle));
}

int CMobot::moveJointWait(mobotJointId_t id)
{
  return Mobot_moveJointWait(_comms, id);
}

int CMobot::moveTo( double angle1,
                          double angle2,
                          double angle3,
                          double angle4)
{
  return Mobot_moveTo(
      _comms, 
      DEG2RAD(angle1), 
      DEG2RAD(angle2), 
      DEG2RAD(angle3), 
      DEG2RAD(angle4));
}

int CMobot::moveToDirect( double angle1,
                          double angle2,
                          double angle3,
                          double angle4)
{
  return Mobot_moveToDirect(
      _comms, 
      DEG2RAD(angle1), 
      DEG2RAD(angle2), 
      DEG2RAD(angle3), 
      DEG2RAD(angle4));
}

int CMobot::moveToNB( double angle1,
                          double angle2,
                          double angle3,
                          double angle4)
{
  return Mobot_moveToNB(
      _comms, 
      DEG2RAD(angle1), 
      DEG2RAD(angle2), 
      DEG2RAD(angle3), 
      DEG2RAD(angle4));
}

int CMobot::moveToDirectNB( double angle1,
                          double angle2,
                          double angle3,
                          double angle4)
{
  return Mobot_moveToDirectNB(
      _comms, 
      DEG2RAD(angle1), 
      DEG2RAD(angle2), 
      DEG2RAD(angle3), 
      DEG2RAD(angle4));
}

int CMobot::moveWait()
{
  return Mobot_moveWait(_comms);
}

int CMobot::moveToZero()
{
  return Mobot_moveToZero(_comms);
}

int CMobot::moveToZeroNB()
{
  return Mobot_moveToZeroNB(_comms);
}

int CMobot::stop()
{
  return Mobot_stop(_comms);
}

int CMobot::stopOneJoint(mobotJointId_t id)
{
  return Mobot_stopOneJoint(_comms, id);
}

int CMobot::stopTwoJoints(mobotJointId_t id1, mobotJointId_t id2)
{
  return Mobot_stopTwoJoints(_comms, id1, id2);
}

int CMobot::stopThreeJoints(mobotJointId_t id1, mobotJointId_t id2, mobotJointId_t id3)
{
  return Mobot_stopThreeJoints(_comms, id1, id2, id3);
}

int CMobot::stopAllJoints()
{
  return Mobot_stopAllJoints(_comms);
}

int CMobotGroup::driveJointToDirect(mobotJointId_t id, double angle)
{
  driveJointToDirectNB(id, angle);
  return moveWait();
}

int CMobotGroup::driveJointTo(mobotJointId_t id, double angle)
{
  driveJointToDirectNB(id, angle);
  return moveWait();
}

int CMobotGroup::driveJointToDirectNB(mobotJointId_t id, double angle)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->driveJointToDirectNB(id, angle);
  }
  return 0;
}

int CMobotGroup::driveJointToNB(mobotJointId_t id, double angle)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->driveJointToDirectNB(id, angle);
  }
  return 0;
}

int CMobotGroup::driveToDirect(double angle1, double angle2, double angle3, double angle4)
{
  driveToDirectNB(angle1, angle2, angle3, angle4);
  return moveWait();
}

int CMobotGroup::driveTo(double angle1, double angle2, double angle3, double angle4)
{
  driveToDirectNB(angle1, angle2, angle3, angle4);
  return moveWait();
}

int CMobotGroup::driveToDirectNB(double angle1, double angle2, double angle3, double angle4)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->driveToDirectNB(angle1, angle2, angle3, angle4);
  }
  return 0;
}

int CMobotGroup::driveToNB(double angle1, double angle2, double angle3, double angle4)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->driveToDirectNB(angle1, angle2, angle3, angle4);
  }
  return 0;
}

int CMobotGroup::isMoving()
{
  for(int i = 0; i < _numRobots; i++) {
    if(_robots[i]->isMoving()) {
      return 1;
    }
  }
  return 0;
}

int CMobotGroup::move(double angle1, double angle2, double angle3, double angle4)
{
  moveNB(angle1, angle2, angle3, angle4);
  return moveWait();
}

int CMobotGroup::moveNB(double angle1, double angle2, double angle3, double angle4)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->moveNB(angle1, angle2, angle3, angle4);
  }
  return 0;
} 

int CMobotGroup::moveContinuousNB(mobotJointState_t dir1, 
                       mobotJointState_t dir2, 
                       mobotJointState_t dir3, 
                       mobotJointState_t dir4)
{
  DEPRECATED("moveContinuousNB", "setMovementStateNB");
  return setMovementStateNB(dir1, dir2, dir3, dir4);
}

int CMobotGroup::moveContinuousTime(mobotJointState_t dir1, 
                           mobotJointState_t dir2, 
                           mobotJointState_t dir3, 
                           mobotJointState_t dir4, 
                           double seconds)
{
  DEPRECATED("moveContinuousTime", "setMovementStateTime");
  return setMovementStateTime(dir1, dir2, dir3, dir4, seconds);
}

int CMobotGroup::moveJointContinuousNB(mobotJointId_t id, mobotJointState_t dir)
{
  DEPRECATED("moveJointContinuousNB", "setJointMovementStateNB");
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->moveJointContinuousNB(id, dir);
  }
  return 0;
}

int CMobotGroup::moveJointContinuousTime(mobotJointId_t id, mobotJointState_t dir, double seconds)
{
  DEPRECATED("moveJointContinuousTime", "setJointMovementStateTime");
  return setJointMovementStateTime(id, dir, seconds);
}

int CMobotGroup::moveJointTo(mobotJointId_t id, double angle)
{
  moveJointToNB(id, angle);
  return moveWait();
}

int CMobotGroup::moveJointToDirect(mobotJointId_t id, double angle)
{
  moveJointToDirectNB(id, angle);
  return moveWait();
}

int CMobotGroup::moveJointToNB(mobotJointId_t id, double angle)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->moveJointToNB(id, angle);
  }
  return 0;
}

int CMobotGroup::moveJointToDirectNB(mobotJointId_t id, double angle)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->moveJointToDirectNB(id, angle);
  }
  return 0;
}

int CMobotGroup::moveJointWait(mobotJointId_t id)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->moveJointWait(id);
  }
  return 0;
}

int CMobotGroup::moveTo(double angle1, double angle2, double angle3, double angle4)
{
  moveToNB(angle1, angle2, angle3, angle4);
  return moveWait();
}

int CMobotGroup::moveToDirect(double angle1, double angle2, double angle3, double angle4)
{
  moveToDirectNB(angle1, angle2, angle3, angle4);
  return moveWait();
}

int CMobotGroup::moveToNB(double angle1, double angle2, double angle3, double angle4)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->moveToNB(angle1, angle2, angle3, angle4);
  }
  return 0;
}

int CMobotGroup::moveToDirectNB(double angle1, double angle2, double angle3, double angle4)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->moveToDirectNB(angle1, angle2, angle3, angle4);
  }
  return 0;
}

int CMobotGroup::moveWait()
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->moveWait();
  }
  return 0;
}

int CMobotGroup::moveToZeroNB()
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->moveToZeroNB();
  }
  return 0;
}

int CMobotGroup::moveToZero()
{
  moveToZeroNB();
  return moveWait();
}

int CMobotGroup::stopAllJoints()
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->stopAllJoints();
  }
  return 0;
}

int CMobotGroup::stopOneJoint(mobotJointId_t id)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->stopOneJoint(id);
  }
  return 0;
}

int CMobotGroup::stopTwoJoints(mobotJointId_t id1, mobotJointId_t id2)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->stopTwoJoints(id1, id2);
  }
  return 0;
}

int CMobotGroup::stopThreeJoints(mobotJointId_t id1, mobotJointId_t id2, mobotJointId_t id3)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->stopThreeJoints(id1, id2, id3);
  }
  return 0;
}

#endif // __cplusplus
