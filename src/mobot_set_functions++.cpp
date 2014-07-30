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

#define DEPRECATED(from, to) \
  fprintf(stderr, "Warning: The function \"%s()\" is deprecated. Please use \"%s()\"\n" , from, to)

int CMobot::setBuzzerFrequencyOn (double freq)
{
  return Mobot_setBuzzerFrequencyOn(_comms, freq);
}

int CMobot::setExitState(robotJointState_t exitState)
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

int CMobot::setJointDirection(robotJointId_t id, robotJointState_t dir)
{
  return Mobot_setJointDirection(_comms, id, dir);
}

int CMobot::setJointMovementStateNB(robotJointId_t id, robotJointState_t dir)
{
  return Mobot_setJointMovementStateNB(_comms, id, dir);
}

int CMobot::setJointMovementStateTime(robotJointId_t id, robotJointState_t dir, double seconds)
{
  return Mobot_setJointMovementStateTime(_comms, id, dir, seconds);
}

/*int CMobot::setJointSpeed(robotJointId_t id, double speed)
{
  return Mobot_setJointSpeed(_comms, id, DEG2RAD(speed));
}*/

int CMobot::setJointSpeed(robotJointId_t id, double speed)
{
    robotJointState_t dir;
	Mobot_getJointDirection(_comms, id, &dir);
	Mobot_setJointSpeed(_comms, id, DEG2RAD(speed));
	return 0;

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

int CMobot::setJointSpeedRatio(robotJointId_t id, double ratio)
{
  return Mobot_setJointSpeedRatio(_comms, id, ratio);
}

int CMobot::setJointSpeedRatios(double ratio1, double ratio2, double ratio3, double ratio4)
{
  return Mobot_setJointSpeedRatios(_comms, ratio1, ratio2, ratio3, ratio4);
}

int CMobot::setMotorPower(robotJointId_t id, int power)
{
  return Mobot_setMotorPower(_comms, id, power);
}

int CMobot::setJointPower(robotJointId_t id, double power)
{
    DEPRECATED("setJointPower", "moveJointByPowerNB");
    return Mobot_setJointPower(_comms, id, power);
}

int CMobot::moveJointByPowerNB(robotJointId_t id, double power)
{
	return Mobot_setJointPower(_comms, id, power);
}

int CMobot::setMovementStateNB( robotJointState_t dir1,
                                robotJointState_t dir2,
                                robotJointState_t dir3,
                                robotJointState_t dir4)
{
  return Mobot_setMovementStateNB(_comms, dir1, dir2, dir3, dir4);
}

int CMobot::setMovementStateTime( robotJointState_t dir1,
                                  robotJointState_t dir2,
                                  robotJointState_t dir3,
                                  robotJointState_t dir4,
                                  double seconds)
{
  return Mobot_setMovementStateTime(_comms, dir1, dir2, dir3, dir4, seconds);
}

int CMobot::setMovementStateTimeNB( robotJointState_t dir1,
                                  robotJointState_t dir2,
                                  robotJointState_t dir3,
                                  robotJointState_t dir4,
                                  double seconds)
{
  return Mobot_setMovementStateTimeNB(_comms, dir1, dir2, dir3, dir4, seconds);
}

/*int CMobot::setTwoWheelRobotSpeed(double speed, double radius)
{
  return Mobot_setTwoWheelRobotSpeed(_comms, speed, radius);
}*/

int CMobot::setTwoWheelRobotSpeed(double speed, double radius)
{
	int form;
	Mobot_getFormFactor(_comms, &form);
	if (form == MOBOTFORM_L)
	{
		printf("Function setTwoWheelRobotSpeed() not applicable to Linkbot L\n");
		return 0;
	}
	return Mobot_setTwoWheelRobotSpeed(_comms, speed, radius);
}

int CMobot::setSpeed(double speed, double radius)
{
	int form;
	Mobot_getFormFactor(_comms, &form);
	if (form == MOBOTFORM_L)
	{
		printf("Function setSpeed() not applicable to Linkbot L\n");
		return 0;
	}
	return Mobot_setTwoWheelRobotSpeed(_comms, speed, radius);
}

int CMobot::holdJointsAtExit()
{
  return Mobot_setExitState(_comms, ROBOT_HOLD);
}


