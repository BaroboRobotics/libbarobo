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
  _robots = NULL;
}

CMobotLGroup::~CMobotLGroup()
{
}

int CMobotLGroup::addRobot(CMobotL& robot)
{
  int rc = CMobotGroup::addRobot((CMobot&)robot);
  _robots = (CMobotL**)CMobotGroup::_robots;
  return 0;
}

int CMobotLGroup::addRobots(CMobotL robots[], int numRobots)
{
  int rc = CMobotGroup::addRobots(robots, numRobots);
  _robots = (CMobotL**)CMobotGroup::_robots;
  return rc;
}

