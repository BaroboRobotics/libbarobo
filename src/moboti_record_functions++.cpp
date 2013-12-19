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

#include <stdlib.h>
#include "mobot.h"
#include "mobot_internal.h"
#include "linkbot.h"

int CLinkbotI::recordAngles(double *time, 
    double *angle1, 
    double *angle2, 
    double *angle3, 
    int num, 
    double seconds,
    int shiftData)
{
  int rc;
  double *angle4;
  angle4 = (double*)malloc(sizeof(double)*num);
  rc = Mobot_recordAngles(_comms, time, angle1, angle2, angle3, angle4, num, seconds, shiftData);
  free(angle4);
  return rc;
}

int CLinkbotI::recordAnglesBegin(double* &time, 
    double* &angle1, 
    double* &angle2, 
    double* &angle3, 
    double seconds,
    int shiftData)
{
  double *angle4;
  return Mobot_recordAnglesBegin(_comms, &time, &angle1, &angle2, &angle3, &angle4, seconds, shiftData);
}

int CLinkbotI::recordDistancesBegin(
    double* &time,
    double* &distance1,
    double* &distance2,
    double* &distance3,
    double radius, 
    double timeInterval,
    int shiftData)
{
  double* distance4;
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

