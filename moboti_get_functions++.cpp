/*
   Copyright 2013 Barobo, Inc.

   This file is part of libbarobo.

   Foobar is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Foobar is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "mobot.h"
#include "linkbot.h"
#define DEPRECATED(from, to) \
  fprintf(stderr, "Warning: The function \"%s()\" is deprecated. Please use \"%s()\"\n" , from, to)

int CLinkbotI::getID()
{
  return Mobot_getID(_comms);
}

int CLinkbotI::getAccelerometerData(double &accel_x, double &accel_y, double &accel_z)
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

int CLinkbotI::getBatteryVoltage(double &voltage)
{
  return Mobot_getBatteryVoltage(_comms, &voltage);
}

int CLinkbotI::getJointAngles(
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

int CLinkbotI::getJointAnglesAverage(
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

int CLinkbotI::getJointSpeeds(double &speed1, double &speed2, double &speed3)
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

int CLinkbotI::getJointSpeedRatios(double &ratio1, double &ratio2, double &ratio3)
{
  double ratio4;
  return Mobot_getJointSpeedRatios(_comms, &ratio1, &ratio2, &ratio3, &ratio4);
}

int CLinkbotI::getColorRGB(int &r, int &g, int &b)
{
  return Mobot_getColorRGB(_comms, &r, &g, &b);
}
