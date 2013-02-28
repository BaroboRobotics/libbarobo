
#include "mobot.h"
#include "mobot_internal.h"

#define DEPRECATED(from, to) \
  fprintf(stderr, "Warning: The function \"%s()\" is deprecated. Please use \"%s()\"\n" , from, to)

int CMobotL::driveJointToDirect(mobotJointId_t id, double angle)
{
  return Mobot_driveJointToDirect(_comms, id, DEG2RAD(angle));
}

int CMobotL::driveJointTo(mobotJointId_t id, double angle)
{
  return Mobot_driveJointToDirect(_comms, id, DEG2RAD(angle));
}

int CMobotL::driveJointToDirectNB(mobotJointId_t id, double angle)
{
  return Mobot_driveJointToDirectNB(_comms, id, DEG2RAD(angle));
}

int CMobotL::driveJointToNB(mobotJointId_t id, double angle)
{
  return Mobot_driveJointToDirectNB(_comms, id, DEG2RAD(angle));
}

int CMobotL::driveToDirect( double angle1,
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

int CMobotL::driveTo( double angle1,
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

int CMobotL::driveToDirectNB( double angle1,
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

int CMobotL::driveToNB( double angle1,
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

int CMobotL::isMoving()
{
  return Mobot_isMoving(_comms);
}

int CMobotL::move( double angle1,
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

int CMobotL::moveNB( double angle1,
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

int CMobotL::moveContinuousNB( mobotJointState_t dir1, mobotJointState_t dir2, mobotJointState_t dir3)
{
  DEPRECATED("moveContinuousNB", "setMovementStateNB");
  return Mobot_moveContinuousNB(_comms, dir1, dir2, dir3, MOBOT_NEUTRAL);
}

int CMobotL::moveContinuousTime( mobotJointState_t dir1, mobotJointState_t dir2, mobotJointState_t dir3, double seconds)
{
  DEPRECATED("moveContinuousTime", "setMovementStateTime");
  return Mobot_moveContinuousTime(_comms, dir1, dir2, dir3, MOBOT_NEUTRAL, seconds);
}

int CMobotL::moveJointContinuousNB(mobotJointId_t id, mobotJointState_t dir)
{
  DEPRECATED("moveJointContinuousNB", "setJointMovementStateNB");
  return Mobot_moveJointContinuousNB(_comms, id, dir);
}

int CMobotL::moveJointContinuousTime(mobotJointId_t id, mobotJointState_t dir, double seconds)
{
  DEPRECATED("moveJointContinuousTime", "setJointMovementStateTime");
  return Mobot_moveJointContinuousTime(_comms, id, dir, seconds);
}

int CMobotL::moveJoint(mobotJointId_t id, double angle)
{
  return Mobot_moveJoint(_comms, id, DEG2RAD(angle));
}

int CMobotL::moveJointNB(mobotJointId_t id, double angle)
{
  return Mobot_moveJointNB(_comms, id, DEG2RAD(angle));
}

int CMobotL::moveJointTo(mobotJointId_t id, double angle)
{
  return Mobot_moveJointTo(_comms, id, DEG2RAD(angle));
}

int CMobotL::moveJointToDirect(mobotJointId_t id, double angle)
{
  return Mobot_moveJointToDirect(_comms, id, DEG2RAD(angle));
}

int CMobotL::moveJointToNB(mobotJointId_t id, double angle)
{
  return Mobot_moveJointToNB(_comms, id, DEG2RAD(angle));
}

int CMobotL::moveJointToDirectNB(mobotJointId_t id, double angle)
{
  return Mobot_moveJointToDirectNB(_comms, id, DEG2RAD(angle));
}

int CMobotL::moveJointWait(mobotJointId_t id)
{
  return Mobot_moveJointWait(_comms, id);
}

int CMobotL::moveTo( double angle1,
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

int CMobotL::moveToDirect( double angle1,
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

int CMobotL::moveToNB( double angle1,
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

int CMobotL::moveToDirectNB( double angle1,
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

int CMobotL::moveWait()
{
  return Mobot_moveWait(_comms);
}

int CMobotL::moveToZero()
{
  return Mobot_moveToZero(_comms);
}

int CMobotL::moveToZeroNB()
{
  return Mobot_moveToZeroNB(_comms);
}

int CMobotL::stop()
{
  return Mobot_stop(_comms);
}

int CMobotL::stopOneJoint(mobotJointId_t id)
{
  return Mobot_stopOneJoint(_comms, id);
}

int CMobotL::stopAllJoints()
{
  return Mobot_stopAllJoints(_comms);
}

