#include <stdlib.h>
#include "mobot.h"
#include "mobot_internal.h"

CMobotI::CMobotI()
{
}

CMobotI::~CMobotI() 
{
}

int CMobotI::connect()
{
  int rc = Mobot_connect(_comms);
  if(rc) {
    return rc;
  }
  if(_comms->formFactor != MOBOTFORM_I) {
    fprintf(stderr, "Error: Connected Mobot is not a Mobot-I.\n");
    Mobot_disconnect(_comms);
    return -1;
  }
}

int CMobotI::connectWithSerialID(const char* serialID)
{
  return Mobot_connectWithSerialID(_comms, serialID);
}

