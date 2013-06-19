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

