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

#include <stdlib.h>
#include "mobot.h"
#include "mobot_internal.h"
#include "linkbot.h"

#define DEPRECATED(from, to) \
  fprintf(stderr, "Warning: The function \"%s()\" is deprecated. Please use \"%s()\"\n" , from, to)

CLinkbotLGroup::CLinkbotLGroup()
{
  _numRobots = 0;
  _motionInProgress = 0;
  _thread = (THREAD_T*)malloc(sizeof(THREAD_T));
  _robots = NULL;
}

CLinkbotLGroup::~CLinkbotLGroup()
{
}

int CLinkbotLGroup::addRobot(CLinkbotL& robot)
{
  int rc = CLinkbotIGroup::addRobot((CLinkbotI&)robot);
  return 0;
}

int CLinkbotLGroup::addRobots(CLinkbotL robots[], int numRobots)
{
  int rc = CLinkbotIGroup::addRobots((CLinkbotI*)robots, numRobots);
  return rc;
}

int CLinkbotLGroup::connect()
{
  for(int i = 0; i < _numRobots; i++) {
    ((CLinkbotL*)_robots[i])->connect();
  }
  return 0;
}

int CLinkbotLGroup::moveForeverNB()
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->moveForeverNB();
  }
  return 0;
} 

int CLinkbotLGroup::moveJointForeverNB(robotJointId_t id)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->moveJointForeverNB(id);
  }
  return 0;
} 

int CLinkbotLGroup::holdJoints()
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->holdJoints();
  }
  return 0;
}

int CLinkbotLGroup::holdJoint(robotJointId_t id)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->holdJoint(id);
  }
  return 0;
}

int CLinkbotLGroup::relaxJoints()
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->relaxJoints();
  }
  return 0;
}

int CLinkbotLGroup::relaxJoint(robotJointId_t id)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->relaxJoint(id);
  }
  return 0;
}

int CLinkbotLGroup::holdJointsAtExit()
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->holdJointsAtExit();
  }
  return 0;
}

int CLinkbotLGroup::moveJoint(robotJointId_t id, double angle)
{
  moveJointNB(id, angle);
  return moveWait();
}

int CLinkbotLGroup::moveJointNB(robotJointId_t id, double angle)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->moveJointNB(id, angle);
  }
  return 0;
}

int CLinkbotLGroup::moveTime(double time)
{
  moveTimeNB(time);
  return moveWait();
}

int CLinkbotLGroup::moveTimeNB(double time)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->moveTimeNB(time);
  }
  return 0;
}

int CLinkbotLGroup::moveJointTime(robotJointId_t id, double time)
{
  moveJointTimeNB(id, time);
  return moveWait();
}

int CLinkbotLGroup::moveJointTimeNB(robotJointId_t id, double time)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->moveJointTimeNB(id, time);
  }
  return 0;
}

int CLinkbotLGroup::setSpeed(double speed, double radius)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->setSpeed(speed, radius);
  }
  return 0;
}

int CLinkbotLGroup::jumpTo(double angle1, double angle2, double angle3)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->jumpToNB(angle1, angle2, angle3);
  }
  return moveWait();
}
int CLinkbotLGroup::jumpToNB(double angle1, double angle2, double angle3)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->jumpToNB(angle1, angle2, angle3);
  }
  return 0;
}
int CLinkbotLGroup::jumpJointTo(robotJointId_t id, double angle)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->jumpJointToNB(id, angle);
  }
  return moveWait();
}
int CLinkbotLGroup::jumpJointToNB(robotJointId_t id, double angle)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->jumpJointToNB(id, angle);
  }
  return 0;
}