#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "mobot.h"
#define DEPRECATED(from, to) \
  fprintf(stderr, "Warning: The function \"%s()\" is deprecated. Please use \"%s()\"\n" , from, to)

const char* CMobot::getConfigFilePath()
{
  return Mobot_getConfigFilePath();
}

int CMobot::getFormFactor(int &formFactor)
{
  return Mobot_getFormFactor(_comms, &formFactor);
}

int CMobot::getJointAngle(robotJointId_t id, double &angle)
{
  int err;
  err = Mobot_getJointAngle(_comms, id, &angle);
  angle = RAD2DEG(angle);
  return err;
}

int CMobot::getJointAngleAverage(robotJointId_t id, double &angle, int numReadings)
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

int CMobot::getJointDirection(robotJointId_t id, mobotJointState_t &dir)
{
  return Mobot_getJointDirection(_comms, id, &dir);
}

int CMobot::getJointMaxSpeed(robotJointId_t id, double &maxSpeed)
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

int CMobot::getJointSpeed(robotJointId_t id, double &speed)
{
  int err;
  err = Mobot_getJointSpeed(_comms, id, &speed);
  speed = RAD2DEG(speed);
  return err;
}

int CMobot::getJointSpeedRatio(robotJointId_t id, double &ratio)
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

int CMobot::getJointState(robotJointId_t id, mobotJointState_t &state)
{
  return Mobot_getJointState(_comms, id, &state);
}

mobot_t* CMobot::getMobotObject()
{
  return _comms;
}

