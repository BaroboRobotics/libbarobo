
#include "mobot.h"
#include "mobot_internal.h"
#include "linkbot.h"

#define DEPRECATED(from, to) \
  fprintf(stderr, "Warning: The function \"%s()\" is deprecated. Please use \"%s()\"\n" , from, to)

int CLinkbotI::driveToDirect( double angle1,
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

int CLinkbotI::driveTo( double angle1,
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

int CLinkbotI::driveToDirectNB( double angle1,
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

int CLinkbotI::driveToNB( double angle1,
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

int CLinkbotI::move( double angle1,
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

int CLinkbotI::moveNB( double angle1,
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

int CLinkbotI::moveContinuousNB( mobotJointState_t dir1, mobotJointState_t dir2, mobotJointState_t dir3)
{
  DEPRECATED("moveContinuousNB", "setMovementStateNB");
  return Mobot_moveContinuousNB(_comms, dir1, dir2, dir3, ROBOT_NEUTRAL);
}

int CLinkbotI::moveContinuousTime( mobotJointState_t dir1, mobotJointState_t dir2, mobotJointState_t dir3, double seconds)
{
  DEPRECATED("moveContinuousTime", "setMovementStateTime");
  return Mobot_moveContinuousTime(_comms, dir1, dir2, dir3, ROBOT_NEUTRAL, seconds);
}

int CLinkbotI::moveTo( double angle1,
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

int CLinkbotI::moveToDirect( double angle1,
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

int CLinkbotI::moveToNB( double angle1,
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

int CLinkbotI::moveToDirectNB( double angle1,
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

