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


#include "mobot.h"
#include "mobot_internal.h"

int CMobot::recordAngle(robotJointId_t id, double* time, double* angle, int num, double seconds, int shiftTime)
{
  return Mobot_recordAngle(_comms, id, time, angle, num, seconds, shiftTime);
}

int CMobot::recordAngleBegin(robotJointId_t id, double* &time, double* &angle, double seconds, int shiftData)
{
return Mobot_recordAngleBegin(_comms, id, &time, &angle, seconds, shiftData);
}

int CMobot::recordAngleEnd(robotJointId_t id, int &num)
{
  return Mobot_recordAngleEnd(_comms, id, &num);
}

int CMobot::recordAngles(double *time, 
    double *angle1, 
    double *angle2, 
    double *angle3, 
    double *angle4, 
    int num, 
    double seconds,
    int shiftData)
{
  return Mobot_recordAngles(_comms, time, angle1, angle2, angle3, angle4, num, seconds, shiftData);
}

int CMobot::recordAnglesBegin(double* &time, 
    double* &angle1, 
    double* &angle2, 
    double* &angle3, 
    double* &angle4, 
    double seconds,
    int shiftData)
{
  return Mobot_recordAnglesBegin(_comms, &time, &angle1, &angle2, &angle3, &angle4, seconds, shiftData);
}

int CMobot::recordAnglesEnd(int &num)
{
  return Mobot_recordAnglesEnd(_comms, &num);
}

int CMobot::recordDistanceBegin( robotJointId_t id,
                                     double* &time,
                                     double* &distance,
                                     double radius,
                                     double timeInterval,
                                     int shiftData)
{
  return Mobot_recordDistanceBegin(_comms, 
      id,
      &time,
      &distance,
      radius,
      timeInterval,
      shiftData);
}

int CMobot::recordDistanceEnd( robotJointId_t id, int &num)
{
  return Mobot_recordDistanceEnd(_comms, id, &num);
}

int CMobot::recordDistanceOffset(double distance)
{
  return Mobot_recordDistanceOffset(_comms, distance);
}

int CMobot::recordDistancesBegin(
    double* &time,
    double* &distance1,
    double* &distance2,
    double* &distance3,
    double* &distance4,
    double radius, 
    double timeInterval,
    int shiftData)
{
  return Mobot_recordDistancesBegin(_comms,
    &time,
    &distance1,
    &distance2,
    &distance3,
    &distance4,
    radius, 
    timeInterval,
    shiftData);
}

int CMobot::recordDistancesEnd(int &num)
{
  return Mobot_recordDistancesEnd(_comms, &num);
}

int CMobot::recordWait()
{
  return Mobot_recordWait(_comms);
}

