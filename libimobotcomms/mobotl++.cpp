#include <stdlib.h>
#include "mobot.h"
#include "mobot_internal.h"

CMobotL::CMobotL()
{
  _comms = (mobot_t*)malloc(sizeof(mobot_t));
  Mobot_init(_comms);
}

CMobotL::~CMobotL() 
{
  if(_comms->exitState == MOBOT_HOLD) {
    setMovementStateNB(MOBOT_HOLD, MOBOT_HOLD, MOBOT_HOLD);
  } else {
    stop();
  }
  if(_comms->connected) {
    disconnect();
  }
  /* Free stuff that should be freed */
  int i;
  for(i = 0; i < _comms->numItemsToFreeOnExit; i++) {
    free(_comms->itemsToFreeOnExit[i]);
  }
}

int CMobotL::connectWithSerialID(const char* serialID)
{
  return Mobot_connectWithSerialID(_comms, serialID);
}

