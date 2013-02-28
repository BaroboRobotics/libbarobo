
#include "mobot.h"
#include "mobot_internal.h"
#include "thread_macros.h"

int CMobot::motionArch(double angle)
{
  return Mobot_motionArch(_comms, DEG2RAD(angle));
}

int CMobot::motionDistance(double distance, double radius)
{
  return Mobot_motionDistance(_comms, distance, radius);
}

int CMobot::motionDistanceNB(double distance, double radius)
{
  return Mobot_motionDistanceNB(_comms, distance, radius);
}

int CMobot::motionArchNB(double angle)
{
  return Mobot_motionArchNB(_comms, DEG2RAD(angle));
}

int CMobot::motionInchwormLeft(int num)
{
  return Mobot_motionInchwormLeft(_comms, num);
}

int CMobot::motionInchwormLeftNB(int num)
{
  return Mobot_motionInchwormLeftNB(_comms, num);
}

int CMobot::motionInchwormRight(int num)
{
  return Mobot_motionInchwormRight(_comms, num);
}

int CMobot::motionInchwormRightNB(int num)
{
  return Mobot_motionInchwormRightNB(_comms, num);
}

int CMobot::motionRollBackward(double angle)
{
  return Mobot_motionRollBackward(_comms, DEG2RAD(angle));
}

int CMobot::motionRollBackwardNB(double angle)
{
  return Mobot_motionRollBackwardNB(_comms, DEG2RAD(angle));
}

int CMobot::motionRollForward(double angle)
{
  return Mobot_motionRollForward(_comms, DEG2RAD(angle));
}

int CMobot::motionRollForwardNB(double angle)
{
  return Mobot_motionRollForwardNB(_comms, DEG2RAD(angle));
}

int CMobot::motionSkinny(double angle)
{
  return Mobot_motionSkinny(_comms, DEG2RAD(angle));
}

int CMobot::motionSkinnyNB(double angle)
{
  return Mobot_motionSkinnyNB(_comms, DEG2RAD(angle));
}

int CMobot::motionStand()
{
  return Mobot_motionStand(_comms);
}

int CMobot::motionStandNB()
{
  return Mobot_motionStandNB(_comms);
}

int CMobot::motionTumbleRight(int num)
{
  return Mobot_motionTumbleRight(_comms, num);
}

int CMobot::motionTumbleRightNB(int num)
{
  return Mobot_motionTumbleRightNB(_comms, num);
}

int CMobot::motionTumbleLeft(int num)
{
  return Mobot_motionTumbleLeft(_comms, num);
}

int CMobot::motionTumbleLeftNB(int num)
{
  return Mobot_motionTumbleLeftNB(_comms, num);
}

int CMobot::motionTurnLeft(double angle)
{
  return Mobot_motionTurnLeft(_comms, DEG2RAD(angle));
}

int CMobot::motionTurnLeftNB(double angle)
{
  return Mobot_motionTurnLeftNB(_comms, DEG2RAD(angle));
}

int CMobot::motionTurnRight(double angle)
{
  return Mobot_motionTurnRight(_comms, DEG2RAD(angle));
}

int CMobot::motionTurnRightNB(double angle)
{
  return Mobot_motionTurnRightNB(_comms, DEG2RAD(angle));
}

int CMobot::motionUnstand()
{
  return Mobot_motionUnstand(_comms);
}

int CMobot::motionUnstandNB()
{
  return Mobot_motionUnstandNB(_comms);
}

int CMobot::motionWait()
{
  return Mobot_motionWait(_comms);
}

