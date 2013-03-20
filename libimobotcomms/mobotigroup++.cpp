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
}

CMobotIGroup::~CMobotIGroup()
{
}

int CMobotIGroup::addRobot(CMobotI& robot)
{
  _robots[_numRobots] = &robot;
  _numRobots++;
  return 0;
}

int CMobotIGroup::reset()
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->reset();
  }
  return 0;
}

int CMobotIGroup::resetToZero()
{
  resetToZeroNB();
  return moveWait();
}

int CMobotIGroup::resetToZeroNB()
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->resetToZeroNB();
  }
  return 0;
}

int CMobotIGroup::motionDistance(double distance, double radius)
{
  argDouble = distance / radius;
  _motionInProgress++;
  motionDistanceThread(this);
  return 0;
}

int CMobotIGroup::motionDistanceNB(double distance, double radius)
{
  argDouble = distance / radius;
  _motionInProgress++;
  THREAD_CREATE(_thread, motionDistanceThread, this);
  return 0;
}

void* CMobotIGroup::motionDistanceThread(void* arg)
{
  CMobotIGroup *cmg = (CMobotIGroup*)arg;
  cmg->move(RAD2DEG(cmg->argDouble), 0, -RAD2DEG(cmg->argDouble));
  cmg->_motionInProgress--;
  return NULL;
}

int CMobotIGroup::motionRollBackward(double angle)
{
  argDouble = angle;
  _motionInProgress++;
  motionRollBackwardThread(this);
  return 0;
}

int CMobotIGroup::motionRollBackwardNB(double angle)
{
  argDouble = angle;
  _motionInProgress++;
  THREAD_CREATE(_thread, motionRollBackwardThread, this);
  return 0;
}

void* CMobotIGroup::motionRollBackwardThread(void* arg)
{
  CMobotIGroup *cmg = (CMobotIGroup*)arg;
  cmg->move(-cmg->argDouble, 0, cmg->argDouble);
  cmg->_motionInProgress--;
  return NULL;
}

int CMobotIGroup::motionRollForward(double angle)
{
  argDouble = angle;
  _motionInProgress++;
  motionRollForwardThread(this);
  return 0;
}

int CMobotIGroup::motionRollForwardNB(double angle)
{
  argDouble = angle;
  _motionInProgress++;
  THREAD_CREATE(_thread, motionRollForwardThread, this);
  return 0;
}

void* CMobotIGroup::motionRollForwardThread(void* arg)
{
  CMobotIGroup *cmg = (CMobotIGroup*)arg;
  cmg->move(cmg->argDouble, 0, -cmg->argDouble);
  cmg->_motionInProgress--;
  return NULL;
}

int CMobotIGroup::motionTurnLeft(double angle)
{
  argDouble = angle;
  _motionInProgress++;
  motionTurnLeftThread(this);
  return 0;
}

int CMobotIGroup::motionTurnLeftNB(double angle)
{
  argDouble = angle;
  _motionInProgress++;
  THREAD_CREATE(_thread, motionTurnLeftThread, this);
  return 0;
}

void* CMobotIGroup::motionTurnLeftThread(void* arg)
{
  CMobotIGroup* cmg = (CMobotIGroup*)arg;
  cmg->move(-cmg->argDouble, 0, -cmg->argDouble);
  cmg->_motionInProgress--;
  return NULL;
}

int CMobotIGroup::motionTurnRight(double angle)
{
  argDouble = angle;
  _motionInProgress++;
  motionTurnRightThread(this);
  return 0;
}

int CMobotIGroup::motionTurnRightNB(double angle)
{
  argDouble = angle;
  _motionInProgress++;
  THREAD_CREATE(_thread, motionTurnRightThread, this);
  return 0;
}

void* CMobotIGroup::motionTurnRightThread(void* arg)
{
  CMobotIGroup* cmg = (CMobotIGroup*)arg;
  cmg->move(cmg->argDouble, 0, cmg->argDouble);
  cmg->_motionInProgress--;
  return NULL;
}

int CMobotIGroup::motionWait()
{
  while(_motionInProgress > 0) {
#ifdef _WIN32
    Sleep(200);
#else
    usleep(200000);
#endif
  }
  return 0;
}

int CMobotIGroup::driveJointToDirect(mobotJointId_t id, double angle)
{
  driveJointToDirectNB(id, angle);
  return moveWait();
}

int CMobotIGroup::driveJointTo(mobotJointId_t id, double angle)
{
  driveJointToDirectNB(id, angle);
  return moveWait();
}

int CMobotIGroup::driveJointToDirectNB(mobotJointId_t id, double angle)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->driveJointToDirectNB(id, angle);
  }
  return 0;
}

int CMobotIGroup::driveJointToNB(mobotJointId_t id, double angle)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->driveJointToDirectNB(id, angle);
  }
  return 0;
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

int CMobotIGroup::isMoving()
{
  for(int i = 0; i < _numRobots; i++) {
    if(_robots[i]->isMoving()) {
      return 1;
    }
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

int CMobotIGroup::moveBackward(double angle)
{
  int rc;
  rc = moveBackwardNB(angle);
  if(rc) return rc;
  return moveWait();
}

int CMobotIGroup::moveBackwardNB(double angle)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->moveBackwardNB(angle);
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

int CMobotIGroup::moveDistance(double distance, double radius)
{
  int rc;
  rc = moveDistanceNB(distance, radius);
  if(rc) return rc;
  return moveWait();
}

int CMobotIGroup::moveDistanceNB(double distance, double radius)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->moveDistanceNB(distance, radius);
  }
  return 0;
}

int CMobotIGroup::moveForward(double angle)
{
  int rc;
  rc = moveForwardNB(angle);
  if(rc) return rc;
  return moveWait();
}

int CMobotIGroup::moveForwardNB(double angle)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->moveForwardNB(angle);
  }
  return 0;
}

int CMobotIGroup::moveJointContinuousNB(mobotJointId_t id, mobotJointState_t dir)
{
  DEPRECATED("moveJointContinuousNB", "setJointMovementStateNB");
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->moveJointContinuousNB(id, dir);
  }
  return 0;
}

int CMobotIGroup::moveJointContinuousTime(mobotJointId_t id, mobotJointState_t dir, double seconds)
{
  DEPRECATED("moveJointContinuousTime", "setJointMovementStateTime");
  return setJointMovementStateTime(id, dir, seconds);
}

int CMobotIGroup::moveJointTo(mobotJointId_t id, double angle)
{
  moveJointToNB(id, angle);
  return moveWait();
}

int CMobotIGroup::moveJointToDirect(mobotJointId_t id, double angle)
{
  moveJointToDirectNB(id, angle);
  return moveWait();
}

int CMobotIGroup::moveJointToNB(mobotJointId_t id, double angle)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->moveJointToNB(id, angle);
  }
  return 0;
}

int CMobotIGroup::moveJointToDirectNB(mobotJointId_t id, double angle)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->moveJointToDirectNB(id, angle);
  }
  return 0;
}

int CMobotIGroup::moveJointWait(mobotJointId_t id)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->moveJointWait(id);
  }
  return 0;
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

int CMobotIGroup::moveWait()
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->moveWait();
  }
  return 0;
}

int CMobotIGroup::moveToZeroNB()
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->moveToZeroNB();
  }
  return 0;
}

int CMobotIGroup::moveToZero()
{
  moveToZeroNB();
  return moveWait();
}

int CMobotIGroup::stopAllJoints()
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->stopAllJoints();
  }
  return 0;
}

int CMobotIGroup::stopOneJoint(mobotJointId_t id)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->stopOneJoint(id);
  }
  return 0;
}

int CMobotIGroup::setExitState(mobotJointState_t exitState)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->setExitState(exitState);
  }
  return 0;
}

int CMobotIGroup::setJointMovementStateNB(mobotJointId_t id, mobotJointState_t dir)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->setJointMovementStateNB(id, dir);
  }
  return 0;
}

int CMobotIGroup::setJointMovementStateTime(mobotJointId_t id, mobotJointState_t dir, double seconds)
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

int CMobotIGroup::setJointMovementStateTimeNB(mobotJointId_t id, mobotJointState_t dir, double seconds)
{
  int msecs = seconds * 1000.0;
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->setJointMovementStateNB(id, dir);
  }
  return 0;
}

int CMobotIGroup::setJointSafetyAngle(double angle)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->setJointSafetyAngle(angle);
  }
  return 0;
}

int CMobotIGroup::setJointSafetyAngleTimeout(double seconds)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->setJointSafetyAngleTimeout(seconds);
  }
  return 0;
}

int CMobotIGroup::setJointSpeed(mobotJointId_t id, double speed)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->setJointSpeed(id, speed);
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

int CMobotIGroup::setJointSpeedRatio(mobotJointId_t id, double ratio)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->setJointSpeedRatio(id, ratio);
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

int CMobotIGroup::setTwoWheelRobotSpeed(double speed, double radius)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->setTwoWheelRobotSpeed(speed, radius);
  }
  return 0;
}

int CMobotIGroup::turnLeft(double angle)
{
  int rc;
  rc = turnLeftNB(angle);
  if(rc) return rc;
  return moveWait();
}

int CMobotIGroup::turnLeftNB(double angle)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->turnLeftNB(angle);
  }
  return 0;
}

int CMobotIGroup::turnRight(double angle)
{
  int rc;
  rc = turnRightNB(angle);
  if(rc) return rc;
  return moveWait();
}

int CMobotIGroup::turnRightNB(double angle)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->turnRightNB(angle);
  }
  return 0;
}

