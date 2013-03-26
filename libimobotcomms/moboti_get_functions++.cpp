#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "mobot.h"
#define DEPRECATED(from, to) \
  fprintf(stderr, "Warning: The function \"%s()\" is deprecated. Please use \"%s()\"\n" , from, to)

int CMobotI::getID()
{
  return Mobot_getID(_comms);
}

int CMobotI::getAccelerometerData(double &accel_x, double &accel_y, double &accel_z)
{
  double _x, _y, _z; int rc;
  rc = Mobot_getAccelerometerData(_comms, &_x, &_y, &_z);
  if(rc) {
    return rc;
  }
  accel_x = _x;
  accel_y = _y;
  accel_z = _z;
  return 0;
}

int CMobotI::getBatteryVoltage(double &voltage)
{
  return Mobot_getBatteryVoltage(_comms, &voltage);
}

int CMobotI::getJointAngles(
    double &angle1,
    double &angle2,
    double &angle3)
{
  double time;
  double angle4;
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

int CMobotI::getJointAnglesAverage(
    double &angle1,
    double &angle2,
    double &angle3,
    int numReadings)
{
  int err;
  double angle4;
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

int CMobotI::getJointSpeeds(double &speed1, double &speed2, double &speed3)
{
  int i;
  double speed4;
  int err = Mobot_getJointSpeeds(_comms, &speed1, &speed2, &speed3, &speed4);
  speed1 = RAD2DEG(speed1);
  speed2 = RAD2DEG(speed2);
  speed3 = RAD2DEG(speed3);
  speed4 = RAD2DEG(speed4);
  return err;
}

int CMobotI::getJointSpeedRatios(double &ratio1, double &ratio2, double &ratio3)
{
  double ratio4;
  return Mobot_getJointSpeedRatios(_comms, &ratio1, &ratio2, &ratio3, &ratio4);
}

int CMobotI::getColorRGB(int &r, int &g, int &b)
{
  return Mobot_getColorRGB(_comms, &r, &g, &b);
}
