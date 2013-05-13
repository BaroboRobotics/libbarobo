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
    fprintf(stderr, "Error: Connected Mobot is not a Mobot-L.\n");
    Mobot_disconnect(_comms);
    return -1;
  }
}

int CLinkbotL::connectWithSerialID(const char* serialID)
{
  return Mobot_connectWithSerialID(_comms, serialID);
}

