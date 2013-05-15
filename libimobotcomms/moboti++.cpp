#include <stdlib.h>
#include "mobot.h"
#include "mobot_internal.h"
#include "linkbot.h"

CLinkbotI::CLinkbotI()
{
}

CLinkbotI::~CLinkbotI() 
{
}

int CLinkbotI::connect()
{
  int rc = Mobot_connect(_comms);
  if(rc) {
    return rc;
  }
  if(_comms->formFactor != MOBOTFORM_I) {
    fprintf(stderr, "Error: Connected Mobot is not a Linkbot-I.\n");
    Mobot_disconnect(_comms);
    return -1;
  }
}

int CLinkbotI::connectWithSerialID(const char* serialID)
{
  return Mobot_connectWithSerialID(_comms, serialID);
}

