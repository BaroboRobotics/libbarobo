#include <stdlib.h>
#include "mobot.h"
#include "mobot_internal.h"

CMobotI::CMobotI()
{
  _comms = (mobot_t*)malloc(sizeof(mobot_t));
  Mobot_init(_comms);
}

CMobotI::~CMobotI() 
{
  if(_comms->connected) {
    if(_comms->exitState == MOBOT_HOLD) {
      setMovementStateNB(MOBOT_HOLD, MOBOT_HOLD, MOBOT_HOLD);
    } else {
      stop();
    }
    disconnect();
  }
  /* Free stuff that should be freed */
  int i;
  for(i = 0; i < _comms->numItemsToFreeOnExit; i++) {
    free(_comms->itemsToFreeOnExit[i]);
  }
}

int CMobotI::connectWithSerialID(const char* serialID)
{
  return Mobot_connectWithSerialID(_comms, serialID);
}

