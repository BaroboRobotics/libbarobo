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

CLinkbotL::CLinkbotL()
{
}

CLinkbotL::~CLinkbotL() 
{
}

int CLinkbotL::connect()
{
  int rc = Mobot_connect(_comms);
  if(rc) {
    return rc;
  }
  if(_comms->formFactor != MOBOTFORM_L) {
    fprintf(stderr, "Error: Connected robot is not a Linkbot-L.\n");
    Mobot_disconnect(_comms);
    return -1;
  }
}

int CLinkbotL::connectWithSerialID(const char* serialID)
{
  return Mobot_connectWithSerialID(_comms, serialID);
}

