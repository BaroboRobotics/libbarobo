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

int CMobotI::blinkLED(double delay, int numBlinks)
{
  return Mobot_blinkLED(_comms, delay, numBlinks);
}

int CMobotI::connect()
{
  return Mobot_connect(_comms);
}

int CMobotI::connectWithAddress(const char* address, int channel)
{
  return Mobot_connectWithAddress(_comms, address, channel);
}

int CMobotI::connectWithIPAddress(const char* address, const char port[])
{
  return Mobot_connectWithIPAddress(_comms, address, port);
}

#ifndef _WIN32
int CMobotI::connectWithTTY(const char* ttyfilename)
{
  return Mobot_connectWithTTY(_comms, ttyfilename);
}
#endif

int CMobotI::connectWithSerialID(const char* serialID)
{
  return Mobot_connectWithSerialID(_comms, serialID);
}

int CMobotI::disconnect()
{
  return Mobot_disconnect(_comms);
}

int CMobotI::enableButtonCallback(void (*buttonCallback)(CMobotI* mobot, int button, int buttonDown))
{
  return Mobot_enableButtonCallback(
      _comms,
      this,
      (void(*)(void*,int,int))buttonCallback);
}

int CMobotI::disableButtonCallback()
{
  return Mobot_disableButtonCallback(_comms);
}

int CMobotI::isConnected()
{
  return Mobot_isConnected(_comms);
}

int CMobotI::reset()
{
  return Mobot_reset(_comms);
}

int CMobotI::resetToZero()
{
  return Mobot_resetToZero(_comms);
}

int CMobotI::resetToZeroNB()
{
  return Mobot_resetToZeroNB(_comms);
}

