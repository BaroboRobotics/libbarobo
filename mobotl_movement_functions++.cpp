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
#include "linkbot.h"

#define DEPRECATED(from, to) \
  fprintf(stderr, "Warning: The function \"%s()\" is deprecated. Please use \"%s()\"\n" , from, to)

int CLinkbotL::driveToDirect( double angle1,
                          double angle2,
                          double angle3)
{
  return Mobot_driveToDirect(
      _comms, 
      DEG2RAD(angle1), 
      DEG2RAD(angle2), 
      DEG2RAD(angle3), 
      DEG2RAD(0));
}

int CLinkbotL::driveTo( double angle1,
                          double angle2,
                          double angle3)
{
  return Mobot_driveToDirect(
      _comms, 
      DEG2RAD(angle1), 
      DEG2RAD(angle2), 
      DEG2RAD(angle3), 
      DEG2RAD(0));
}

int CLinkbotL::driveToDirectNB( double angle1,
                          double angle2,
                          double angle3)
{
  return Mobot_driveToDirectNB(
      _comms, 
      DEG2RAD(angle1), 
      DEG2RAD(angle2), 
      DEG2RAD(angle3), 
      DEG2RAD(0));
}

int CLinkbotL::driveToNB( double angle1,
                          double angle2,
                          double angle3)
{
  return Mobot_driveToDirectNB(
      _comms, 
      DEG2RAD(angle1), 
      DEG2RAD(angle2), 
      DEG2RAD(angle3), 
      DEG2RAD(0));
}

int CLinkbotL::move( double angle1,
                        double angle2,
                        double angle3)
{
  return Mobot_move(
      _comms, 
      DEG2RAD(angle1), 
      DEG2RAD(angle2), 
      DEG2RAD(angle3), 
      DEG2RAD(0));
}

int CLinkbotL::moveNB( double angle1,
                        double angle2,
                        double angle3)
{
  return Mobot_moveNB(
      _comms, 
      DEG2RAD(angle1), 
      DEG2RAD(angle2), 
      DEG2RAD(angle3), 
      DEG2RAD(0));
}

int CLinkbotL::moveContinuousNB( robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3)
{
  DEPRECATED("moveContinuousNB", "setMovementStateNB");
  return Mobot_moveContinuousNB(_comms, dir1, dir2, dir3, ROBOT_NEUTRAL);
}

int CLinkbotL::moveContinuousTime( robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3, double seconds)
{
  DEPRECATED("moveContinuousTime", "setMovementStateTime");
  return Mobot_moveContinuousTime(_comms, dir1, dir2, dir3, ROBOT_NEUTRAL, seconds);
}

int CLinkbotL::moveTo( double angle1,
                          double angle2,
                          double angle3)
{
  return Mobot_moveTo(
      _comms, 
      DEG2RAD(angle1), 
      DEG2RAD(angle2), 
      DEG2RAD(angle3), 
      DEG2RAD(0));
}

int CLinkbotL::moveToDirect( double angle1,
                          double angle2,
                          double angle3)
{
  return Mobot_moveToDirect(
      _comms, 
      DEG2RAD(angle1), 
      DEG2RAD(angle2), 
      DEG2RAD(angle3), 
      DEG2RAD(0));
}

int CLinkbotL::moveToNB( double angle1,
                          double angle2,
                          double angle3)
{
  return Mobot_moveToNB(
      _comms, 
      DEG2RAD(angle1), 
      DEG2RAD(angle2), 
      DEG2RAD(angle3), 
      DEG2RAD(0));
}

int CLinkbotL::moveToDirectNB( double angle1,
                          double angle2,
                          double angle3)
{
  return Mobot_moveToDirectNB(
      _comms, 
      DEG2RAD(angle1), 
      DEG2RAD(angle2), 
      DEG2RAD(angle3), 
      DEG2RAD(0));
}

