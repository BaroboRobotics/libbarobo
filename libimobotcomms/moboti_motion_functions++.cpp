
#include "mobot.h"
#include "mobot_internal.h"
#include "thread_macros.h"

int CMobotI::motionDistance(double distance, double radius)
{
  return Mobot_motionDistance(_comms, distance, radius);
}

int CMobotI::motionDistanceNB(double distance, double radius)
{
  return Mobot_motionDistanceNB(_comms, distance, radius);
}

int CMobotI::motionRollBackward(double angle)
{
  return Mobot_motionRollBackward(_comms, DEG2RAD(angle));
}

int CMobotI::motionRollBackwardNB(double angle)
{
  return Mobot_motionRollBackwardNB(_comms, DEG2RAD(angle));
}

int CMobotI::motionRollForward(double angle)
{
  return Mobot_motionRollForward(_comms, DEG2RAD(angle));
}

int CMobotI::motionRollForwardNB(double angle)
{
  return Mobot_motionRollForwardNB(_comms, DEG2RAD(angle));
}

int CMobotI::motionTurnLeft(double angle)
{
  return Mobot_motionTurnLeft(_comms, DEG2RAD(angle));
}

int CMobotI::motionTurnLeftNB(double angle)
{
  return Mobot_motionTurnLeftNB(_comms, DEG2RAD(angle));
}

int CMobotI::motionTurnRight(double angle)
{
  return Mobot_motionTurnRight(_comms, DEG2RAD(angle));
}

int CMobotI::motionTurnRightNB(double angle)
{
  return Mobot_motionTurnRightNB(_comms, DEG2RAD(angle));
}

int CMobotI::motionWait()
{
  return Mobot_motionWait(_comms);
}

