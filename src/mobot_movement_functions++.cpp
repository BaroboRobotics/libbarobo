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
#include "commands.h"
#include <string.h>

#define DEPRECATED(from, to) \
  fprintf(stderr, "Warning: The function \"%s()\" is deprecated. Please use \"%s()\"\n" , from, to)

int CMobot::driveJointToDirect(robotJointId_t id, double angle)
{
  return Mobot_driveJointToDirect(_comms, id, DEG2RAD(angle));
}

int CMobot::driveJointTo(robotJointId_t id, double angle)
{
    DEPRECATED("driveJointTo", "jumpJointTo");
	return Mobot_driveJointToDirect(_comms, id, DEG2RAD(angle));
}

int CMobot::driveJointToDirectNB(robotJointId_t id, double angle)
{
  return Mobot_driveJointToDirectNB(_comms, id, DEG2RAD(angle));
}

int CMobot::driveJointToNB(robotJointId_t id, double angle)
{
    DEPRECATED("driveJointToNB", "jumpJointToNB");
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
  DEPRECATED("driveTo", "jumpTo");
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
  DEPRECATED("driveToNB", "jumpToNB");
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
    DEPRECATED("moveBackward", "driveBackward");
	return Mobot_moveBackward(_comms, DEG2RAD(angle));
}

int CMobot::moveBackwardNB(double angle)
{
    DEPRECATED("moveBackwardNB", "driveBackwardNB");
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
    DEPRECATED("moveDistance", "driveDistance");
	return Mobot_moveDistance(_comms, distance, radius);
}

int CMobot::moveDistanceNB(double distance, double radius)
{
    DEPRECATED("moveDistanceNB", "driveDistanceNB");
	return Mobot_moveDistanceNB(_comms, distance, radius);
}

int CMobot::moveForward(double angle)
{
  DEPRECATED("moveForward", "driveForward");
	return Mobot_moveForward(_comms, DEG2RAD(angle));
}

int CMobot::moveForwardNB(double angle)
{
    DEPRECATED("moveForwardNB", "driveForwardNB");
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
    DEPRECATED("movexy", "drivexy");
	return Mobot_movexy(_comms, x, y, radius, trackwidth);
}

int CMobot::movexyNB(double x, double y, double radius, double trackwidth)
{
    DEPRECATED("movexyNB", "drivexyNB");
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
	moveTimeNB(time);
    return Mobot_moveWait(_comms);
}

int CMobot::moveTimeNB(double time)
{
	return Mobot_setMovementStateTimeNB(_comms, ROBOT_POSITIVE, ROBOT_POSITIVE,
        ROBOT_BACKWARD, ROBOT_NEUTRAL, time);
}


int CMobot::moveJointTimeNB(robotJointId_t id, double time)
{
    /* Compose a "TIMED_ACTION" message */
    uint8_t buf[32];
    int i; 
    unsigned int millis;
    millis = time * 1000;
    i = ((int)id)-1;
    buf[0] = 1<<i;
    if(id == ROBOT_JOINT3) {
        buf[1] = ROBOT_POSITIVE;
    } else {
        buf[1] = ROBOT_FORWARD;
    }
    buf[2] = ROBOT_HOLD;
    memcpy(&buf[3], &millis, 4);
    return MobotMsgTransaction(_comms, BTCMD(CMD_TIMEDACTION), buf, 7);
}

int CMobot::moveJointTime(robotJointId_t id, double time)
{
    moveJointTimeNB(id, time);
    return moveWait();
}

int CMobot::moveForeverNB()
{
	return Mobot_setMovementStateNB(_comms, 
        ROBOT_POSITIVE,
        ROBOT_POSITIVE,
        ROBOT_BACKWARD,
        ROBOT_POSITIVE);
}

int CMobot::moveJointForeverNB(robotJointId_t id)
{
    /* Compose a "TIMED_ACTION" message */
    uint8_t buf[32];
    int i; 
    int millis;
    millis = -1;
    i = id-1;
    buf[0] = 1<<i;
    if(id == ROBOT_JOINT3) {
        buf[1] = ROBOT_POSITIVE;
    } else {
        buf[1] = ROBOT_FORWARD;
    }
    buf[2] = ROBOT_HOLD;
    memcpy(&buf[3], &millis, 4);
    return MobotMsgTransaction(_comms, BTCMD(CMD_TIMEDACTION), buf, 7);
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


/*Two wheel car functions*/
int CMobot::driveBackward(double angle)
{
	return Mobot_driveBackward(_comms, DEG2RAD(angle));
}
int CMobot::driveBackwardNB(double angle)
{
	return Mobot_driveBackwardNB(_comms, DEG2RAD(angle));
}
int CMobot::driveDistance(double distance, double radius)
{
	return Mobot_driveDistance(_comms, distance, radius);
}
int CMobot::driveDistanceNB(double distance, double radius)
{
	return Mobot_driveDistanceNB(_comms, distance, radius);
}
int CMobot::driveForeverNB()
{
	return Mobot_setMovementStateNB(_comms, 
        ROBOT_POSITIVE,
        ROBOT_POSITIVE,
        ROBOT_FORWARD,
        ROBOT_POSITIVE);
}
int CMobot::driveForward(double angle)
{
	return Mobot_moveForward(_comms, DEG2RAD(angle));
}
int CMobot::driveForwardNB(double angle)
{
	return Mobot_moveForwardNB(_comms, DEG2RAD(angle));
}
int CMobot::driveTime(double time)
{
	driveTimeNB(time);
    return Mobot_moveWait(_comms);
}

int CMobot::driveTimeNB(double time)
{
	return Mobot_setMovementStateTimeNB(_comms, ROBOT_POSITIVE, ROBOT_POSITIVE,
        ROBOT_FORWARD, ROBOT_NEUTRAL, time);
}
int CMobot::drivexy(double x, double y, double radius, double trackwidth)
{
	return Mobot_drivexy(_comms, x, y, radius, trackwidth);
}

int CMobot::drivexyNB(double x, double y, double radius, double trackwidth)
{
	return Mobot_drivexyNB(_comms, x, y, radius, trackwidth);
}
int CMobot::jumpTo( double angle1,
                          double angle2,
                          double angle3,
                          double angle4)
{
	DEPRECATED("jumpTo", "moveToByTrackPos");
	return Mobot_driveToDirect(
      _comms, 
      DEG2RAD(angle1), 
      DEG2RAD(angle2), 
      DEG2RAD(angle3), 
      DEG2RAD(angle4));
}

int CMobot::jumpToNB( double angle1,
                          double angle2,
                          double angle3,
                          double angle4)
{
	DEPRECATED("jumpToNB", "moveToByTrackPosNB");
	return Mobot_driveToDirectNB(
      _comms, 
      DEG2RAD(angle1), 
      DEG2RAD(angle2), 
      DEG2RAD(angle3), 
      DEG2RAD(angle4));
}

int CMobot::jumpJointTo(robotJointId_t id, double angle)
{
	DEPRECATED("jumpJointTo", "moveJointToByTrackPos");
	return Mobot_driveJointToDirect(_comms, id, DEG2RAD(angle));
}

int CMobot::jumpJointToNB(robotJointId_t id, double angle)
{
	DEPRECATED("jumpJointToNB", "moveJointToByTrackPosNB");
	return Mobot_driveJointToDirectNB(_comms, id, DEG2RAD(angle));
}

/*Gripper functions*/
int CMobot::openGripper(double angle)
{
	return moveTo(-angle/2.0, 0, -angle/2.0, 0);
}

int CMobot::openGripperNB(double angle)
{
	return moveToNB(-angle/2.0, 0, -angle/2.0, 0);
}

int CMobot::closeGripper(void)
{
	
	return Mobot_closeGripper(_comms);
}

int CMobot::closeGripperNB(void)
{
	return Mobot_closeGripperNB(_comms);
}

/*rename of jump functions*/
int CMobot::moveToByTrackPos( double angle1,
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

int CMobot::moveToByTrackPosNB( double angle1,
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

int CMobot::moveJointToByTrackPos(robotJointId_t id, double angle)
{
	return Mobot_driveJointToDirect(_comms, id, DEG2RAD(angle));
}

int CMobot::moveJointToByTrackPosNB(robotJointId_t id, double angle)
{
	return Mobot_driveJointToDirectNB(_comms, id, DEG2RAD(angle));
}

	



	
		
