
#include "mobot.h"
#include "mobot_internal.h"

#define DEPRECATED(from, to) \
  fprintf(stderr, "Warning: The function \"%s()\" is deprecated. Please use \"%s()\"\n" , from, to)

int CMobotI::driveJointToDirect(mobotJointId_t id, double angle)
{
  return Mobot_driveJointToDirect(_comms, id, DEG2RAD(angle));
}

int CMobotI::driveJointTo(mobotJointId_t id, double angle)
{
  return Mobot_driveJointToDirect(_comms, id, DEG2RAD(angle));
}

int CMobotI::driveJointToDirectNB(mobotJointId_t id, double angle)
{
  return Mobot_driveJointToDirectNB(_comms, id, DEG2RAD(angle));
}

int CMobotI::driveJointToNB(mobotJointId_t id, double angle)
{
  return Mobot_driveJointToDirectNB(_comms, id, DEG2RAD(angle));
}

int CMobotI::driveToDirect( double angle1,
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

int CMobotI::driveTo( double angle1,
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

int CMobotI::driveToDirectNB( double angle1,
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

int CMobotI::driveToNB( double angle1,
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

int CMobotI::isMoving()
{
  return Mobot_isMoving(_comms);
}

int CMobotI::move( double angle1,
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

int CMobotI::moveNB( double angle1,
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

int CMobotI::moveBackward(double angle)
{
  return Mobot_moveBackward(_comms, DEG2RAD(angle));
}

int CMobotI::moveBackwardNB(double angle)
{
  return Mobot_moveBackwardNB(_comms, DEG2RAD(angle));
}

int CMobotI::moveContinuousNB( mobotJointState_t dir1, mobotJointState_t dir2, mobotJointState_t dir3)
{
  DEPRECATED("moveContinuousNB", "setMovementStateNB");
  return Mobot_moveContinuousNB(_comms, dir1, dir2, dir3, MOBOT_NEUTRAL);
}

int CMobotI::moveContinuousTime( mobotJointState_t dir1, mobotJointState_t dir2, mobotJointState_t dir3, double seconds)
{
  DEPRECATED("moveContinuousTime", "setMovementStateTime");
  return Mobot_moveContinuousTime(_comms, dir1, dir2, dir3, MOBOT_NEUTRAL, seconds);
}

int CMobotI::moveDistance(double distance, double radius)
{
  return Mobot_moveDistance(_comms, distance, radius);
}

int CMobotI::moveDistanceNB(double distance, double radius)
{
  return Mobot_moveDistanceNB(_comms, distance, radius);
}

int CMobotI::moveForward(double angle)
{
  return Mobot_moveForward(_comms, DEG2RAD(angle));
}

int CMobotI::moveForwardNB(double angle)
{
  return Mobot_moveForwardNB(_comms, DEG2RAD(angle));
}

int CMobotI::moveJointContinuousNB(mobotJointId_t id, mobotJointState_t dir)
{
  DEPRECATED("moveJointContinuousNB", "setJointMovementStateNB");
  return Mobot_moveJointContinuousNB(_comms, id, dir);
}

int CMobotI::moveJointContinuousTime(mobotJointId_t id, mobotJointState_t dir, double seconds)
{
  DEPRECATED("moveJointContinuousTime", "setJointMovementStateTime");
  return Mobot_moveJointContinuousTime(_comms, id, dir, seconds);
}

int CMobotI::moveJoint(mobotJointId_t id, double angle)
{
  return Mobot_moveJoint(_comms, id, DEG2RAD(angle));
}

int CMobotI::moveJointNB(mobotJointId_t id, double angle)
{
  return Mobot_moveJointNB(_comms, id, DEG2RAD(angle));
}

int CMobotI::moveJointTo(mobotJointId_t id, double angle)
{
  return Mobot_moveJointTo(_comms, id, DEG2RAD(angle));
}

int CMobotI::moveJointToDirect(mobotJointId_t id, double angle)
{
  return Mobot_moveJointToDirect(_comms, id, DEG2RAD(angle));
}

int CMobotI::moveJointToNB(mobotJointId_t id, double angle)
{
  return Mobot_moveJointToNB(_comms, id, DEG2RAD(angle));
}

int CMobotI::moveJointToDirectNB(mobotJointId_t id, double angle)
{
  return Mobot_moveJointToDirectNB(_comms, id, DEG2RAD(angle));
}

int CMobotI::moveJointWait(mobotJointId_t id)
{
  return Mobot_moveJointWait(_comms, id);
}

int CMobotI::moveTo( double angle1,
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

int CMobotI::moveToDirect( double angle1,
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

int CMobotI::moveToNB( double angle1,
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

int CMobotI::moveToDirectNB( double angle1,
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

int CMobotI::moveWait()
{
  return Mobot_moveWait(_comms);
}

int CMobotI::moveToZero()
{
  return Mobot_moveToZero(_comms);
}

int CMobotI::moveToZeroNB()
{
  return Mobot_moveToZeroNB(_comms);
}

int CMobotI::stop()
{
  return Mobot_stop(_comms);
}

int CMobotI::stopOneJoint(mobotJointId_t id)
{
  return Mobot_stopOneJoint(_comms, id);
}

int CMobotI::stopAllJoints()
{
  return Mobot_stopAllJoints(_comms);
}

int CMobotI::turnLeft(double angle)
{
  return Mobot_turnLeft(_comms, DEG2RAD(angle));
}

int CMobotI::turnLeftNB(double angle)
{
  return Mobot_turnLeftNB(_comms, DEG2RAD(angle));
}

int CMobotI::turnRight(double angle)
{
  return Mobot_turnRight(_comms, DEG2RAD(angle));
}

int CMobotI::turnRightNB(double angle)
{
  return Mobot_turnRightNB(_comms, DEG2RAD(angle));
}
