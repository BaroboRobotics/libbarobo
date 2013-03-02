#include <stdlib.h>
#include "mobot.h"
#include "mobot_internal.h"

#define DEPRECATED(from, to) \
  fprintf(stderr, "Warning: The function \"%s()\" is deprecated. Please use \"%s()\"\n" , from, to)

CMobotLGroup::CMobotLGroup()
{
  _numRobots = 0;
  _motionInProgress = 0;
  _thread = (THREAD_T*)malloc(sizeof(THREAD_T));
}

CMobotLGroup::~CMobotLGroup()
{
}

int CMobotLGroup::addRobot(CMobotL& robot)
{
  _robots[_numRobots] = &robot;
  _numRobots++;
  return 0;
}

int CMobotLGroup::reset()
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->reset();
  }
  return 0;
}

int CMobotLGroup::resetToZero()
{
  resetToZeroNB();
  return moveWait();
}

int CMobotLGroup::resetToZeroNB()
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->resetToZeroNB();
  }
  return 0;
}

int CMobotLGroup::driveJointToDirect(mobotJointId_t id, double angle)
{
  driveJointToDirectNB(id, angle);
  return moveWait();
}

int CMobotLGroup::driveJointTo(mobotJointId_t id, double angle)
{
  driveJointToDirectNB(id, angle);
  return moveWait();
}

int CMobotLGroup::driveJointToDirectNB(mobotJointId_t id, double angle)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->driveJointToDirectNB(id, angle);
  }
  return 0;
}

int CMobotLGroup::driveJointToNB(mobotJointId_t id, double angle)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->driveJointToDirectNB(id, angle);
  }
  return 0;
}

int CMobotLGroup::driveToDirect(double angle1, double angle2, double angle3)
{
  driveToDirectNB(angle1, angle2, angle3);
  return moveWait();
}

int CMobotLGroup::driveTo(double angle1, double angle2, double angle3)
{
  driveToDirectNB(angle1, angle2, angle3);
  return moveWait();
}

int CMobotLGroup::driveToDirectNB(double angle1, double angle2, double angle3)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->driveToDirectNB(angle1, angle2, angle3);
  }
  return 0;
}

int CMobotLGroup::driveToNB(double angle1, double angle2, double angle3)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->driveToDirectNB(angle1, angle2, angle3);
  }
  return 0;
}

int CMobotLGroup::isMoving()
{
  for(int i = 0; i < _numRobots; i++) {
    if(_robots[i]->isMoving()) {
      return 1;
    }
  }
  return 0;
}

int CMobotLGroup::move(double angle1, double angle2, double angle3)
{
  moveNB(angle1, angle2, angle3);
  return moveWait();
}

int CMobotLGroup::moveNB(double angle1, double angle2, double angle3)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->moveNB(angle1, angle2, angle3);
  }
  return 0;
} 

int CMobotLGroup::moveContinuousNB(mobotJointState_t dir1, 
                       mobotJointState_t dir2, 
                       mobotJointState_t dir3)
{
  DEPRECATED("moveContinuousNB", "setMovementStateNB");
  return setMovementStateNB(dir1, dir2, dir3);
}

int CMobotLGroup::moveContinuousTime(mobotJointState_t dir1, 
                           mobotJointState_t dir2, 
                           mobotJointState_t dir3, 
                           double seconds)
{
  DEPRECATED("moveContinuousTime", "setMovementStateTime");
  return setMovementStateTime(dir1, dir2, dir3, seconds);
}

int CMobotLGroup::moveJointContinuousNB(mobotJointId_t id, mobotJointState_t dir)
{
  DEPRECATED("moveJointContinuousNB", "setJointMovementStateNB");
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->moveJointContinuousNB(id, dir);
  }
  return 0;
}

int CMobotLGroup::moveJointContinuousTime(mobotJointId_t id, mobotJointState_t dir, double seconds)
{
  DEPRECATED("moveJointContinuousTime", "setJointMovementStateTime");
  return setJointMovementStateTime(id, dir, seconds);
}

int CMobotLGroup::moveJointTo(mobotJointId_t id, double angle)
{
  moveJointToNB(id, angle);
  return moveWait();
}

int CMobotLGroup::moveJointToDirect(mobotJointId_t id, double angle)
{
  moveJointToDirectNB(id, angle);
  return moveWait();
}

int CMobotLGroup::moveJointToNB(mobotJointId_t id, double angle)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->moveJointToNB(id, angle);
  }
  return 0;
}

int CMobotLGroup::moveJointToDirectNB(mobotJointId_t id, double angle)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->moveJointToDirectNB(id, angle);
  }
  return 0;
}

int CMobotLGroup::moveJointWait(mobotJointId_t id)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->moveJointWait(id);
  }
  return 0;
}

int CMobotLGroup::moveTo(double angle1, double angle2, double angle3)
{
  moveToNB(angle1, angle2, angle3);
  return moveWait();
}

int CMobotLGroup::moveToDirect(double angle1, double angle2, double angle3)
{
  moveToDirectNB(angle1, angle2, angle3);
  return moveWait();
}

int CMobotLGroup::moveToNB(double angle1, double angle2, double angle3)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->moveToNB(angle1, angle2, angle3);
  }
  return 0;
}

int CMobotLGroup::moveToDirectNB(double angle1, double angle2, double angle3)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->moveToDirectNB(angle1, angle2, angle3);
  }
  return 0;
}

int CMobotLGroup::moveWait()
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->moveWait();
  }
  return 0;
}

int CMobotLGroup::moveToZeroNB()
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->moveToZeroNB();
  }
  return 0;
}

int CMobotLGroup::moveToZero()
{
  moveToZeroNB();
  return moveWait();
}

int CMobotLGroup::stopAllJoints()
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->stopAllJoints();
  }
  return 0;
}

int CMobotLGroup::stopOneJoint(mobotJointId_t id)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->stopOneJoint(id);
  }
  return 0;
}

int CMobotLGroup::setExitState(mobotJointState_t exitState)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->setExitState(exitState);
  }
  return 0;
}

int CMobotLGroup::setJointMovementStateNB(mobotJointId_t id, mobotJointState_t dir)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->setJointMovementStateNB(id, dir);
  }
  return 0;
}

int CMobotLGroup::setJointMovementStateTime(mobotJointId_t id, mobotJointState_t dir, double seconds)
{
  int msecs = seconds * 1000.0;
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->setJointMovementStateNB(id, dir);
  }
#ifdef _WIN32
  Sleep(msecs);
#else
  usleep(msecs * 1000);
#endif
  return 0;
}

int CMobotLGroup::setJointMovementStateTimeNB(mobotJointId_t id, mobotJointState_t dir, double seconds)
{
  int msecs = seconds * 1000.0;
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->setJointMovementStateNB(id, dir);
  }
  return 0;
}

int CMobotLGroup::setJointSafetyAngle(double angle)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->setJointSafetyAngle(angle);
  }
  return 0;
}

int CMobotLGroup::setJointSafetyAngleTimeout(double seconds)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->setJointSafetyAngleTimeout(seconds);
  }
  return 0;
}

int CMobotLGroup::setJointSpeed(mobotJointId_t id, double speed)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->setJointSpeed(id, speed);
  }
  return 0;
}

int CMobotLGroup::setJointSpeeds(double speed1, double speed2, double speed3)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->setJointSpeeds(speed1, speed2, speed3);
  }
  return 0;
}

int CMobotLGroup::setJointSpeedRatio(mobotJointId_t id, double ratio)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->setJointSpeedRatio(id, ratio);
  }
  return 0;
}

int CMobotLGroup::setJointSpeedRatios(double ratio1, double ratio2, double ratio3)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->setJointSpeedRatios(ratio1, ratio2, ratio3);
  }
  return 0;
}

int CMobotLGroup::setMovementStateNB(mobotJointState_t dir1, 
                       mobotJointState_t dir2, 
                       mobotJointState_t dir3)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->setMovementStateNB(dir1, dir2, dir3);
  }
  return 0;
}

int CMobotLGroup::setMovementStateTime(mobotJointState_t dir1, 
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

int CMobotLGroup::setMovementStateTimeNB(mobotJointState_t dir1, 
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

int CMobotLGroup::setTwoWheelRobotSpeed(double speed, double radius)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->setTwoWheelRobotSpeed(speed, radius);
  }
  return 0;
}

