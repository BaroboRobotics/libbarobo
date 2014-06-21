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

int CMobot::driveJointToDirect(robotJointId_t id, double angle)
{
  return Mobot_driveJointToDirect(_comms, id, DEG2RAD(angle));
}

int CMobot::driveJointTo(robotJointId_t id, double angle)
{
  return Mobot_driveJointToDirect(_comms, id, DEG2RAD(angle));
}

int CMobot::driveJointToDirectNB(robotJointId_t id, double angle)
{
  return Mobot_driveJointToDirectNB(_comms, id, DEG2RAD(angle));
}

int CMobot::driveJointToNB(robotJointId_t id, double angle)
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

int CMobot::moveBackward(double angle)
{
  return Mobot_moveBackward(_comms, DEG2RAD(angle));
}

int CMobot::moveBackwardNB(double angle)
{
  return Mobot_moveBackwardNB(_comms, DEG2RAD(angle));
}

int CMobot::moveContinuousNB( robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3, robotJointState_t dir4)
{
  DEPRECATED("moveContinuousNB", "setMovementStateNB");
  return Mobot_moveContinuousNB(_comms, dir1, dir2, dir3, dir4);
}

int CMobot::moveContinuousTime( robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3, robotJointState_t dir4, double seconds)
{
  DEPRECATED("moveContinuousTime", "setMovementStateTime");
  return Mobot_moveContinuousTime(_comms, dir1, dir2, dir3, dir4, seconds);
}

int CMobot::moveDistance(double distance, double radius)
{
  return Mobot_moveDistance(_comms, distance, radius);
}

int CMobot::moveDistanceNB(double distance, double radius)
{
  return Mobot_moveDistanceNB(_comms, distance, radius);
}

int CMobot::moveForward(double angle)
{
  return Mobot_moveForward(_comms, DEG2RAD(angle));
}

int CMobot::moveForwardNB(double angle)
{
  return Mobot_moveForwardNB(_comms, DEG2RAD(angle));
}

int CMobot::moveJointContinuousNB(robotJointId_t id, robotJointState_t dir)
{
  DEPRECATED("moveJointContinuousNB", "setJointMovementStateNB");
  return Mobot_moveJointContinuousNB(_comms, id, dir);
}

int CMobot::moveJointContinuousTime(robotJointId_t id, robotJointState_t dir, double seconds)
{
  DEPRECATED("moveJointContinuousTime", "setJointMovementStateTime");
  return Mobot_moveJointContinuousTime(_comms, id, dir, seconds);
}

int CMobot::moveJoint(robotJointId_t id, double angle)
{
  return Mobot_moveJoint(_comms, id, DEG2RAD(angle));
}

int CMobot::moveJointNB(robotJointId_t id, double angle)
{
  return Mobot_moveJointNB(_comms, id, DEG2RAD(angle));
}

int CMobot::moveJointTo(robotJointId_t id, double angle)
{
  return Mobot_moveJointTo(_comms, id, DEG2RAD(angle));
}

int CMobot::moveJointToDirect(robotJointId_t id, double angle)
{
  return Mobot_moveJointToDirect(_comms, id, DEG2RAD(angle));
}

int CMobot::moveJointToNB(robotJointId_t id, double angle)
{
  return Mobot_moveJointToNB(_comms, id, DEG2RAD(angle));
}

int CMobot::moveJointToDirectNB(robotJointId_t id, double angle)
{
  return Mobot_moveJointToDirectNB(_comms, id, DEG2RAD(angle));
}

int CMobot::moveJointWait(robotJointId_t id)
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

int CMobot::movexy(double x, double y, double radius, double trackwidth)
{
  return Mobot_movexy(_comms, x, y, radius, trackwidth);
}

int CMobot::movexyNB(double x, double y, double radius, double trackwidth)
{
  return Mobot_movexyNB(_comms, x, y, radius, trackwidth);
}

int CMobot::stop()
{
  return Mobot_stop(_comms);
}

int CMobot::stopOneJoint(robotJointId_t id)
{
  return Mobot_stopOneJoint(_comms, id);
}

int CMobot::stopTwoJoints(robotJointId_t id1, robotJointId_t id2)
{
  return Mobot_stopTwoJoints(_comms, id1, id2);
}

int CMobot::stopThreeJoints(robotJointId_t id1, robotJointId_t id2, robotJointId_t id3)
{
  return Mobot_stopThreeJoints(_comms, id1, id2, id3);
}

int CMobot::stopAllJoints()
{
  return Mobot_stopAllJoints(_comms);
}

int CMobot::turnLeft(double angle, double radius, double tracklength)
{
  return Mobot_turnLeft(_comms, DEG2RAD(angle), radius, tracklength);
}

int CMobot::turnLeftNB(double angle, double radius, double tracklength)
{
  return Mobot_turnLeftNB(_comms, DEG2RAD(angle), radius, tracklength);
}

int CMobot::turnRight(double angle, double radius, double tracklength)
{
  return Mobot_turnRight(_comms, DEG2RAD(angle), radius, tracklength);
}

int CMobot::turnRightNB(double angle, double radius, double tracklength)
{
  return Mobot_turnRightNB(_comms, DEG2RAD(angle), radius, tracklength);
}

int CMobot::moveTime(double time)
{
	robotJointState_t dir;
	int form;
	Mobot_getJointDirection(_comms, ROBOT_JOINT1, &dir);
	Mobot_getFormFactor(_comms, &form);

	if(form == MOBOTFORM_L) 
	{
		switch(dir)
	   {
		case ROBOT_FORWARD:
		    Mobot_setMovementStateTimeNB(_comms, ROBOT_FORWARD, ROBOT_FORWARD, ROBOT_NEUTRAL, ROBOT_NEUTRAL, time);
			return Mobot_moveWait(_comms);
		    break;
		
		case ROBOT_BACKWARD:
		    Mobot_setMovementStateTimeNB(_comms, ROBOT_BACKWARD, ROBOT_BACKWARD, ROBOT_NEUTRAL, ROBOT_NEUTRAL, time);
			return Mobot_moveWait(_comms);
		    break;
		
		default:
		    Mobot_setMovementStateTimeNB(_comms, ROBOT_FORWARD, ROBOT_FORWARD, ROBOT_NEUTRAL, ROBOT_NEUTRAL, time);
			return Mobot_moveWait(_comms);
		    break;
	     }
	}

	else
	{
		switch(dir)
	    {
		case ROBOT_FORWARD:
		    Mobot_setMovementStateTimeNB(_comms, ROBOT_FORWARD, ROBOT_NEUTRAL, ROBOT_FORWARD, ROBOT_NEUTRAL, time);
			return Mobot_moveWait(_comms);
		    break;
		
		case ROBOT_BACKWARD:
		    Mobot_setMovementStateTimeNB(_comms, ROBOT_BACKWARD, ROBOT_NEUTRAL, ROBOT_BACKWARD, ROBOT_NEUTRAL, time);
			return Mobot_moveWait(_comms);
		    break;
		
		default:
		    Mobot_setMovementStateTimeNB(_comms, ROBOT_FORWARD, ROBOT_NEUTRAL, ROBOT_FORWARD, ROBOT_NEUTRAL, time);
			return Mobot_moveWait(_comms);
		    break;
	    }
	}
}

int CMobot::moveTimeNB(double time)
{
	robotJointState_t dir;
	int form;
	Mobot_getJointDirection(_comms, ROBOT_JOINT1, &dir);
	Mobot_getFormFactor(_comms, &form);

	if(form == MOBOTFORM_L) 
	{
		switch(dir)
	   {
		case ROBOT_FORWARD:
		    return Mobot_setMovementStateTimeNB(_comms, ROBOT_FORWARD, ROBOT_FORWARD, ROBOT_NEUTRAL, ROBOT_NEUTRAL, time);
		    break;
		
		case ROBOT_BACKWARD:
		    return Mobot_setMovementStateTimeNB(_comms, ROBOT_BACKWARD, ROBOT_BACKWARD, ROBOT_NEUTRAL, ROBOT_NEUTRAL, time);
		    break;
		
		default:
		    return Mobot_setMovementStateTimeNB(_comms, ROBOT_FORWARD, ROBOT_FORWARD, ROBOT_NEUTRAL, ROBOT_NEUTRAL, time);
		    break;
	     }
	}

	else
	{
		switch(dir)
	    {
		case ROBOT_FORWARD:
		    return Mobot_setMovementStateTimeNB(_comms, ROBOT_FORWARD, ROBOT_NEUTRAL, ROBOT_FORWARD, ROBOT_NEUTRAL, time);
		    break;
		
		case ROBOT_BACKWARD:
		    return Mobot_setMovementStateTimeNB(_comms, ROBOT_BACKWARD, ROBOT_NEUTRAL, ROBOT_BACKWARD, ROBOT_NEUTRAL, time);
		    break;
		
		default:
		    return Mobot_setMovementStateTimeNB(_comms, ROBOT_FORWARD, ROBOT_NEUTRAL, ROBOT_FORWARD, ROBOT_NEUTRAL, time);
		    break;
	    }
	}
}


int CMobot::moveJointTimeNB(robotJointId_t id, double time)
{
	robotJointState_t dir, dirs[4];
	int i, ret, form;
    
	ret=Mobot_getJointDirection(_comms, id, &dir);
	if (ret == 0)
	{
		if(dir == ROBOT_HOLD)
		{
			dir = ROBOT_FORWARD;
		}
		for (i=1; i<5; i++)
		{
			if (i == id)
			{
				dirs[i-1]=dir;
			}
			else
			{
				dirs[i-1]=ROBOT_HOLD;
			}
		}

	}
	else 
	{
		dirs[id-1]=ROBOT_FORWARD;
	}

	return Mobot_setMovementStateTimeNB(_comms, dirs[0], dirs[1], dirs[2], dirs[3], time);
}

int CMobot::moveJointTime(robotJointId_t id, double time)
{
	robotJointState_t dir, dirs[4];
	int i, ret, form;
    
	ret=Mobot_getJointDirection(_comms, id, &dir);
	if (ret == 0)
	{
		if(dir == ROBOT_HOLD || dir == ROBOT_NEUTRAL)
		{
			dir = ROBOT_FORWARD;
		}
		for (i=1; i<5; i++)
		{
			if (i == id)
			{
				dirs[i-1]=dir;
			}
			else
			{
				dirs[i-1]=ROBOT_HOLD;
			}
		}

	}
	else 
	{
		dirs[id-1]=ROBOT_FORWARD;
	}

	Mobot_setMovementStateTimeNB(_comms, dirs[0], dirs[1], dirs[2], dirs[3], time);
	return Mobot_moveWait(_comms);
}

int CMobot::moveForeverNB()
{
	robotJointState_t dirs[4];
	int i;

	for (i=0; i<4; i++)
	{
		Mobot_getJointDirection(_comms, (robotJointId_t)(i+1), &dirs[i]);
		if (dirs[i] == ROBOT_HOLD || dirs[i] == ROBOT_NEUTRAL)
		{
			dirs[i]=ROBOT_FORWARD;
		}
	}

	return Mobot_setMovementStateNB(_comms, dirs[0], dirs[1], dirs[2], dirs[3]);
}

int CMobot::moveJointForeverNB(robotJointId_t id)
{
	robotJointState_t dir;
	int i;

	Mobot_getJointDirection(_comms, id, &dir);
	if (dir == ROBOT_HOLD || dir == ROBOT_NEUTRAL)
	{
		dir = ROBOT_FORWARD;
	}
	return Mobot_setJointMovementStateNB(_comms, id, dir);
}

int CMobot::holdJoints()
{
  return Mobot_stopAllJoints(_comms);
}

int CMobot::holdJoint(robotJointId_t id)
{
  return Mobot_stopOneJoint(_comms, id);
}

int CMobot::relaxJoints()
{
  return Mobot_setMovementStateNB(_comms, ROBOT_NEUTRAL, ROBOT_NEUTRAL, ROBOT_NEUTRAL, ROBOT_NEUTRAL);
}

int CMobot::relaxJoint(robotJointId_t id)
{
  return Mobot_setJointMovementStateNB(_comms, id, ROBOT_NEUTRAL);
}



	



	
		
