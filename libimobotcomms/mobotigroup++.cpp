#include <stdlib.h>
#include "mobot.h"
#include "mobot_internal.h"

#define DEPRECATED(from, to) \
  fprintf(stderr, "Warning: The function \"%s()\" is deprecated. Please use \"%s()\"\n" , from, to)

CMobotIGroup::CMobotIGroup()
{
  _numRobots = 0;
  _motionInProgress = 0;
  _thread = (THREAD_T*)malloc(sizeof(THREAD_T));
  _robots = (CMobotI**)CMobotGroup::_robots;
}

CMobotIGroup::~CMobotIGroup()
{
}

int CMobotIGroup::addRobot(CMobotI& mobot)
{
  int rc;
  rc = CMobotGroup::addRobot((CMobot&)mobot);
  _robots = (CMobotI**)CMobotGroup::_robots;
  return rc;
}

int CMobotIGroup::addRobots(CMobotI mobots[], int numMobots)
{
  int rc = CMobotGroup::addRobots(mobots, numMobots);
  _robots = (CMobotI**)CMobotGroup::_robots;
  return rc;
}

int CMobotIGroup::driveToDirect(double angle1, double angle2, double angle3)
{
  driveToDirectNB(angle1, angle2, angle3);
  return moveWait();
}

int CMobotIGroup::driveTo(double angle1, double angle2, double angle3)
{
  driveToDirectNB(angle1, angle2, angle3);
  return moveWait();
}

int CMobotIGroup::driveToDirectNB(double angle1, double angle2, double angle3)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->driveToDirectNB(angle1, angle2, angle3);
  }
  return 0;
}

int CMobotIGroup::driveToNB(double angle1, double angle2, double angle3)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->driveToDirectNB(angle1, angle2, angle3);
  }
  return 0;
}

int CMobotIGroup::move(double angle1, double angle2, double angle3)
{
  moveNB(angle1, angle2, angle3);
  return moveWait();
}

int CMobotIGroup::moveNB(double angle1, double angle2, double angle3)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->moveNB(angle1, angle2, angle3);
  }
  return 0;
} 

int CMobotIGroup::moveContinuousNB(mobotJointState_t dir1, 
                       mobotJointState_t dir2, 
                       mobotJointState_t dir3)
{
  DEPRECATED("moveContinuousNB", "setMovementStateNB");
  return setMovementStateNB(dir1, dir2, dir3);
}

int CMobotIGroup::moveContinuousTime(mobotJointState_t dir1, 
                           mobotJointState_t dir2, 
                           mobotJointState_t dir3, 
                           double seconds)
{
  DEPRECATED("moveContinuousTime", "setMovementStateTime");
  return setMovementStateTime(dir1, dir2, dir3, seconds);
}

int CMobotIGroup::moveTo(double angle1, double angle2, double angle3)
{
  moveToNB(angle1, angle2, angle3);
  return moveWait();
}

int CMobotIGroup::moveToDirect(double angle1, double angle2, double angle3)
{
  moveToDirectNB(angle1, angle2, angle3);
  return moveWait();
}

int CMobotIGroup::moveToNB(double angle1, double angle2, double angle3)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->moveToNB(angle1, angle2, angle3);
  }
  return 0;
}

int CMobotIGroup::moveToDirectNB(double angle1, double angle2, double angle3)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->moveToDirectNB(angle1, angle2, angle3);
  }
  return 0;
}

int CMobotIGroup::setJointSpeeds(double speed1, double speed2, double speed3)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->setJointSpeeds(speed1, speed2, speed3);
  }
  return 0;
}

int CMobotIGroup::setJointSpeedRatios(double ratio1, double ratio2, double ratio3)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->setJointSpeedRatios(ratio1, ratio2, ratio3);
  }
  return 0;
}

int CMobotIGroup::setMovementStateNB(mobotJointState_t dir1, 
                       mobotJointState_t dir2, 
                       mobotJointState_t dir3)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->setMovementStateNB(dir1, dir2, dir3);
  }
  return 0;
}

int CMobotIGroup::setMovementStateTime(mobotJointState_t dir1, 
                           mobotJointState_t dir2, 
                           mobotJointState_t dir3, 
                           double seconds)
{
  int msecs = seconds * 1000.0;
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->setMovementStateNB(dir1, dir2, dir3);
  }
#ifdef _WIN32
  Sleep(msecs);
#else
  usleep(msecs*1000);
#endif
  for(int i = 0; i < _numRobots; i++) {
    //_robots[i]->stop();
    _robots[i]->setMovementStateNB(MOBOT_HOLD, MOBOT_HOLD, MOBOT_HOLD);
  }
  return 0;
}

int CMobotIGroup::setMovementStateTimeNB(mobotJointState_t dir1, 
                           mobotJointState_t dir2, 
                           mobotJointState_t dir3, 
                           double seconds)
{
  int msecs = seconds * 1000.0;
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->setMovementStateNB(dir1, dir2, dir3);
  }
  return 0;
}

