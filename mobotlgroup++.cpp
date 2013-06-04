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

