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

int CLinkbotI::setBuzzerFrequency(int frequency, double time)
{
  return Mobot_setBuzzerFrequency(_comms, frequency, time);
}

int CLinkbotI::setBuzzerFrequencyOn(int frequency)
{
  return Mobot_setBuzzerFrequencyOn(_comms, frequency);
}

int CLinkbotI::setBuzzerFrequencyOff()
{
  return Mobot_setBuzzerFrequencyOff(_comms);
}

int CLinkbotI::setColorRGB(int r, int g, int b)
{
  return Mobot_setColorRGB(_comms, r, g, b);
}

int CLinkbotI::setJointSpeeds(double speed1, double speed2, double speed3)
{
  return Mobot_setJointSpeeds(
      _comms, 
      DEG2RAD(speed1), 
      DEG2RAD(speed2), 
      DEG2RAD(speed3), 
      DEG2RAD(0));
}

int CLinkbotI::setJointSpeedRatios(double ratio1, double ratio2, double ratio3)
{
  return Mobot_setJointSpeedRatios(_comms, ratio1, ratio2, ratio3, 0);
}

int CLinkbotI::setMovementStateNB( robotJointState_t dir1,
                                robotJointState_t dir2,
                                robotJointState_t dir3)
{
  return Mobot_setMovementStateNB(_comms, dir1, dir2, dir3, ROBOT_NEUTRAL);
}

int CLinkbotI::setMovementStateTime( robotJointState_t dir1,
                                  robotJointState_t dir2,
                                  robotJointState_t dir3,
                                  double seconds)
{
  return Mobot_setMovementStateTime(_comms, dir1, dir2, dir3, ROBOT_NEUTRAL, seconds);
}

int CLinkbotI::setMovementStateTimeNB( robotJointState_t dir1,
                                  robotJointState_t dir2,
                                  robotJointState_t dir3,
                                  double seconds)
{
  return Mobot_setMovementStateTimeNB(_comms, dir1, dir2, dir3, ROBOT_NEUTRAL, seconds);
}

