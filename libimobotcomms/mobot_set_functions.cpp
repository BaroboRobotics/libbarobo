
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

int Mobot_setExitState(mobot_t* comms, mobotJointState_t exitState)
{
  comms->exitState = exitState;
  return 0;
}

int Mobot_setHWRev(mobot_t* comms, uint8_t rev)
{
  uint8_t buf[20];
  int status;
  buf[0] = rev;
  if(status = SendToIMobot(comms, BTCMD(CMD_GETHWREV), buf, 1)) {
    return status;
  }
  if(RecvFromIMobot(comms, buf, sizeof(buf))) {
    return -1;
  }
  if(buf[0] != RESP_OK) {
    return -1;
  }
  return 0;
}

int Mobot_setJointDirection(mobot_t* comms, mobotJointId_t id, mobotJointState_t dir)
{
  uint8_t buf[32];
  int status;
  buf[0] = (uint8_t)id-1;
  buf[1] = (uint8_t) dir;
  status = SendToIMobot(comms, BTCMD(CMD_SETMOTORDIR), buf, 2);
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

int Mobot_setJointMovementStateNB(mobot_t* comms, mobotJointId_t id, mobotJointState_t dir)
{
  Mobot_setJointSpeed(comms, id, comms->jointSpeeds[(int)id-1]);
  Mobot_setJointDirection(comms, id, dir);
  return 0;
}

int Mobot_setJointMovementStateTime(mobot_t* comms, mobotJointId_t id, mobotJointState_t dir, double seconds)
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
  status = SendToIMobot(comms, BTCMD(CMD_SETMOTORSAFETYLIMIT), buf, 4);
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

int Mobot_setJointSafetyAngleTimeout(mobot_t* comms, double seconds)
{
  uint8_t buf[32];
  float f;
  int status;
  f = seconds;
  memcpy(&buf[0], &f, 4);
  status = SendToIMobot(comms, BTCMD(CMD_SETMOTORSAFETYTIMEOUT), buf, 4);
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

int Mobot_setJointSpeed(mobot_t* comms, mobotJointId_t id, double speed)
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
  status = SendToIMobot(comms, BTCMD(CMD_SETMOTORSPEED), buf, 5);
  if(status < 0) return status;
  if(RecvFromIMobot(comms, buf, sizeof(buf))) {
    return -1;
  }
  /* Make sure the data size is correct */
  if(buf[1] != 3) {
    return -1;
  }
  comms->jointSpeeds[id-1] = speed;
  return 0;
}

int Mobot_setJointSpeedRatio(mobot_t* comms, mobotJointId_t id, double ratio)
{
  if((ratio < 0) || (ratio > 1)) {
    return -1;
  }
  return Mobot_setJointSpeed(comms, id, ratio * comms->maxSpeed[(int)id-1]);
}

int Mobot_setJointSpeedRatios(mobot_t* comms, double ratio1, double ratio2, double ratio3, double ratio4)
{
  double ratios[4];
  ratios[0] = ratio1;
  ratios[1] = ratio2;
  ratios[2] = ratio3;
  ratios[3] = ratio4;
  for(int i = 0; i < 4; i++) {
    Mobot_setJointSpeedRatio(comms, (mobotJointId_t)(i+1), ratios[i]);
  }
  return 0;
}

int Mobot_setMotorPower(mobot_t* comms, mobotJointId_t id, int power)
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
  status = SendToIMobot(comms, BTCMD(CMD_SETMOTORPOWER), buf, 10);
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

int Mobot_setMovementStateNB(mobot_t* comms,
                                  mobotJointState_t dir1,
                                  mobotJointState_t dir2,
                                  mobotJointState_t dir3,
                                  mobotJointState_t dir4)
{
  int i;
  int32_t msecs = -1;
  uint8_t buf[64];
  int status;
  mobotJointState_t dirs[4];
  dirs[0] = dir1; dirs[1] = dir2; dirs[2] = dir3; dirs[3] = dir4;
  buf[0] = 0x0F;
  for(i = 0; i < 4; i++) {
    buf[i*6 + 1] = dirs[i];
    buf[i*6 + 2] = MOBOT_HOLD;
    memcpy(&buf[i*6 + 3], &msecs, 4);
  }
  status = SendToIMobot(comms, BTCMD(CMD_TIMEDACTION), buf, 6*4 + 1);
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

int Mobot_setMovementStateTime(mobot_t* comms,
                                  mobotJointState_t dir1,
                                  mobotJointState_t dir2,
                                  mobotJointState_t dir3,
                                  mobotJointState_t dir4,
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
#else
  usleep(msecs * 1000);
#endif
  return 0;
}

int Mobot_setMovementStateTimeNB(mobot_t* comms,
                                  mobotJointState_t dir1,
                                  mobotJointState_t dir2,
                                  mobotJointState_t dir3,
                                  mobotJointState_t dir4,
                                  double seconds)
{
  int i;
  int32_t msecs = seconds * 1000;
  uint8_t buf[64];
  int status;
  mobotJointState_t dirs[4];
  dirs[0] = dir1; dirs[1] = dir2; dirs[2] = dir3; dirs[3] = dir4;
  buf[0] = 0x0F;
  for(i = 0; i < 4; i++) {
    buf[i*6 + 1] = dirs[i];
    buf[i*6 + 2] = MOBOT_HOLD;
    memcpy(&buf[i*6 + 3], &msecs, 4);
  }
  status = SendToIMobot(comms, BTCMD(CMD_TIMEDACTION), buf, 6*4 + 1);
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

int Mobot_setRGB(mobot_t* comms, double r, double g, double b)
{
  uint8_t buf[32];
  float f;
  int status;
  buf[0] = 0xff;
  buf[1] = 0xff;
  buf[2] = 0xff;
  buf[3] = (r/100.0)*255;
  buf[4] = (g/100.0)*255;
  buf[5] = (b/100.0)*255;

  status = SendToIMobot(comms, BTCMD(CMD_RGBLED), buf, 6);
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

int Mobot_setTwoWheelRobotSpeed(mobot_t* comms, double speed, double radius)
{
  double omega;
  omega = speed/radius;
  Mobot_setJointSpeed(comms, MOBOT_JOINT1, omega);
  Mobot_setJointSpeed(comms, MOBOT_JOINT4, omega);
  return 0;
}

int Mobot_setJointSpeeds(mobot_t* comms, double speed1, double speed2, double speed3, double speed4)
{
  double speeds[4];
  speeds[0] = speed1;
  speeds[1] = speed2;
  speeds[2] = speed3;
  speeds[3] = speed4;
  for(int i = 0; i < 4; i++) {
    Mobot_setJointSpeed(comms, (mobotJointId_t)(i+1), speeds[i]);
  }
  return 0;
}

int CMobot::setExitState(mobotJointState_t exitState)
{
  return Mobot_setExitState(_comms, exitState);
}

int CMobot::setJointSafetyAngle(double angle)
{
  angle = DEG2RAD(angle);
  return Mobot_setJointSafetyAngle(_comms, angle);
}

int CMobot::setJointSafetyAngleTimeout(double seconds)
{
  return Mobot_setJointSafetyAngleTimeout(_comms, seconds);
}

int CMobot::setJointDirection(mobotJointId_t id, mobotJointState_t dir)
{
  return Mobot_setJointDirection(_comms, id, dir);
}

int CMobot::setJointMovementStateNB(mobotJointId_t id, mobotJointState_t dir)
{
  return Mobot_setJointMovementStateNB(_comms, id, dir);
}

int CMobot::setJointMovementStateTime(mobotJointId_t id, mobotJointState_t dir, double seconds)
{
  return Mobot_setJointMovementStateTime(_comms, id, dir, seconds);
}

int CMobot::setJointSpeed(mobotJointId_t id, double speed)
{
  return Mobot_setJointSpeed(_comms, id, DEG2RAD(speed));
}

int CMobot::setJointSpeeds(double speed1, double speed2, double speed3, double speed4)
{
  return Mobot_setJointSpeeds(
      _comms, 
      DEG2RAD(speed1), 
      DEG2RAD(speed2), 
      DEG2RAD(speed3), 
      DEG2RAD(speed4));
}

int CMobot::setJointSpeedRatio(mobotJointId_t id, double ratio)
{
  return Mobot_setJointSpeedRatio(_comms, id, ratio);
}

int CMobot::setJointSpeedRatios(double ratio1, double ratio2, double ratio3, double ratio4)
{
  return Mobot_setJointSpeedRatios(_comms, ratio1, ratio2, ratio3, ratio4);
}

int CMobot::setMotorPower(mobotJointId_t id, int power)
{
  return Mobot_setMotorPower(_comms, id, power);
}

int CMobot::setMovementStateNB( mobotJointState_t dir1,
                                mobotJointState_t dir2,
                                mobotJointState_t dir3,
                                mobotJointState_t dir4)
{
  return Mobot_setMovementStateNB(_comms, dir1, dir2, dir3, dir4);
}

int CMobot::setMovementStateTime( mobotJointState_t dir1,
                                  mobotJointState_t dir2,
                                  mobotJointState_t dir3,
                                  mobotJointState_t dir4,
                                  double seconds)
{
  return Mobot_setMovementStateTime(_comms, dir1, dir2, dir3, dir4, seconds);
}

int CMobot::setMovementStateTimeNB( mobotJointState_t dir1,
                                  mobotJointState_t dir2,
                                  mobotJointState_t dir3,
                                  mobotJointState_t dir4,
                                  double seconds)
{
  return Mobot_setMovementStateTimeNB(_comms, dir1, dir2, dir3, dir4, seconds);
}

int CMobot::setTwoWheelRobotSpeed(double speed, double radius)
{
  return Mobot_setTwoWheelRobotSpeed(_comms, speed, radius);
}

int CMobotGroup::setExitState(mobotJointState_t exitState)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->setExitState(exitState);
  }
  return 0;
}

int CMobotGroup::setJointMovementStateNB(mobotJointId_t id, mobotJointState_t dir)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->setJointMovementStateNB(id, dir);
  }
  return 0;
}

int CMobotGroup::setJointMovementStateTime(mobotJointId_t id, mobotJointState_t dir, double seconds)
{
  int msecs = seconds * 1000.0;
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->setJointMovementStateNB(id, dir);
  }
#ifdef _WIN32
  Sleep(msecs);
#else
  usleep(msecs * 1000);
#endif
  return 0;
}

int CMobotGroup::setJointMovementStateTimeNB(mobotJointId_t id, mobotJointState_t dir, double seconds)
{
  int msecs = seconds * 1000.0;
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->setJointMovementStateNB(id, dir);
  }
  return 0;
}

int CMobotGroup::setJointSafetyAngle(double angle)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->setJointSafetyAngle(angle);
  }
  return 0;
}

int CMobotGroup::setJointSafetyAngleTimeout(double seconds)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->setJointSafetyAngleTimeout(seconds);
  }
  return 0;
}

int CMobotGroup::setJointSpeed(mobotJointId_t id, double speed)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->setJointSpeed(id, speed);
  }
  return 0;
}

int CMobotGroup::setJointSpeeds(double speed1, double speed2, double speed3, double speed4)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->setJointSpeeds(speed1, speed2, speed3, speed4);
  }
  return 0;
}

int CMobotGroup::setJointSpeedRatio(mobotJointId_t id, double ratio)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->setJointSpeedRatio(id, ratio);
  }
  return 0;
}

int CMobotGroup::setJointSpeedRatios(double ratio1, double ratio2, double ratio3, double ratio4)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->setJointSpeedRatios(ratio1, ratio2, ratio3, ratio4);
  }
  return 0;
}

int CMobotGroup::setMovementStateNB(mobotJointState_t dir1, 
                       mobotJointState_t dir2, 
                       mobotJointState_t dir3, 
                       mobotJointState_t dir4)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->setMovementStateNB(dir1, dir2, dir3, dir4);
  }
  return 0;
}

int CMobotGroup::setMovementStateTime(mobotJointState_t dir1, 
                           mobotJointState_t dir2, 
                           mobotJointState_t dir3, 
                           mobotJointState_t dir4, 
                           double seconds)
{
  int msecs = seconds * 1000.0;
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->setMovementStateNB(dir1, dir2, dir3, dir4);
  }
#ifdef _WIN32
  Sleep(msecs);
#else
  usleep(msecs*1000);
#endif
  for(int i = 0; i < _numRobots; i++) {
    //_robots[i]->stop();
    _robots[i]->setMovementStateNB(MOBOT_HOLD, MOBOT_HOLD, MOBOT_HOLD, MOBOT_HOLD);
  }
  return 0;
}

int CMobotGroup::setMovementStateTimeNB(mobotJointState_t dir1, 
                           mobotJointState_t dir2, 
                           mobotJointState_t dir3, 
                           mobotJointState_t dir4, 
                           double seconds)
{
  int msecs = seconds * 1000.0;
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->setMovementStateNB(dir1, dir2, dir3, dir4);
  }
  return 0;
}

int CMobotGroup::setTwoWheelRobotSpeed(double speed, double radius)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->setTwoWheelRobotSpeed(speed, radius);
  }
  return 0;
}

