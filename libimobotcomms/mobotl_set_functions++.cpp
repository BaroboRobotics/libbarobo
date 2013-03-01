#include "mobot.h"
#include "mobot_internal.h"

#define DEPRECATED(from, to) \
  fprintf(stderr, "Warning: The function \"%s()\" is deprecated. Please use \"%s()\"\n" , from, to)

int CMobotL::setBuzzerFrequency(int frequency, double time)
{
  return Mobot_setBuzzerFrequency(_comms, frequency, time);
}

int CMobotL::setBuzzerFrequencyOn(int frequency)
{
  return Mobot_setBuzzerFrequencyOn(_comms, frequency);
}

int CMobotL::setBuzzerFrequencyOff()
{
  return Mobot_setBuzzerFrequencyOff(_comms);
}

int CMobotL::setColorRGB(int r, int g, int b)
{
  return Mobot_setColorRGB(_comms, r, g, b);
}

int CMobotL::setExitState(mobotJointState_t exitState)
{
  return Mobot_setExitState(_comms, exitState);
}

int CMobotL::setJointSafetyAngle(double angle)
{
  angle = DEG2RAD(angle);
  return Mobot_setJointSafetyAngle(_comms, angle);
}

int CMobotL::setJointSafetyAngleTimeout(double seconds)
{
  return Mobot_setJointSafetyAngleTimeout(_comms, seconds);
}

int CMobotL::setJointDirection(mobotJointId_t id, mobotJointState_t dir)
{
  return Mobot_setJointDirection(_comms, id, dir);
}

int CMobotL::setJointMovementStateNB(mobotJointId_t id, mobotJointState_t dir)
{
  return Mobot_setJointMovementStateNB(_comms, id, dir);
}

int CMobotL::setJointMovementStateTime(mobotJointId_t id, mobotJointState_t dir, double seconds)
{
  return Mobot_setJointMovementStateTime(_comms, id, dir, seconds);
}

int CMobotL::setJointSpeed(mobotJointId_t id, double speed)
{
  return Mobot_setJointSpeed(_comms, id, DEG2RAD(speed));
}

int CMobotL::setJointSpeeds(double speed1, double speed2, double speed3)
{
  return Mobot_setJointSpeeds(
      _comms, 
      DEG2RAD(speed1), 
      DEG2RAD(speed2), 
      DEG2RAD(speed3), 
      DEG2RAD(0));
}

int CMobotL::setJointSpeedRatio(mobotJointId_t id, double ratio)
{
  return Mobot_setJointSpeedRatio(_comms, id, ratio);
}

int CMobotL::setJointSpeedRatios(double ratio1, double ratio2, double ratio3)
{
  return Mobot_setJointSpeedRatios(_comms, ratio1, ratio2, ratio3, 0);
}

int CMobotL::setMotorPower(mobotJointId_t id, int power)
{
  return Mobot_setMotorPower(_comms, id, power);
}

int CMobotL::setMovementStateNB( mobotJointState_t dir1,
                                mobotJointState_t dir2,
                                mobotJointState_t dir3)
{
  return Mobot_setMovementStateNB(_comms, dir1, dir2, dir3, MOBOT_NEUTRAL);
}

int CMobotL::setMovementStateTime( mobotJointState_t dir1,
                                  mobotJointState_t dir2,
                                  mobotJointState_t dir3,
                                  double seconds)
{
  return Mobot_setMovementStateTime(_comms, dir1, dir2, dir3, MOBOT_NEUTRAL, seconds);
}

int CMobotL::setMovementStateTimeNB( mobotJointState_t dir1,
                                  mobotJointState_t dir2,
                                  mobotJointState_t dir3,
                                  double seconds)
{
  return Mobot_setMovementStateTimeNB(_comms, dir1, dir2, dir3, MOBOT_NEUTRAL, seconds);
}

int CMobotL::setTwoWheelRobotSpeed(double speed, double radius)
{
  return Mobot_setTwoWheelRobotSpeed(_comms, speed, radius);
}

