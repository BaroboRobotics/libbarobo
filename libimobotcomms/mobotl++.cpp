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

int CMobotL::blinkLED(double delay, int numBlinks)
{
  return Mobot_blinkLED(_comms, delay, numBlinks);
}

int CMobotL::connect()
{
  return Mobot_connect(_comms);
}

int CMobotL::connectWithAddress(const char* address, int channel)
{
  return Mobot_connectWithAddress(_comms, address, channel);
}

int CMobotL::connectWithIPAddress(const char* address, const char port[])
{
  return Mobot_connectWithIPAddress(_comms, address, port);
}

#ifndef _WIN32
int CMobotL::connectWithTTY(const char* ttyfilename)
{
  return Mobot_connectWithTTY(_comms, ttyfilename);
}
#endif

int CMobotL::connectWithSerialID(const char* serialID)
{
  return Mobot_connectWithSerialID(_comms, serialID);
}

int CMobotL::disconnect()
{
  return Mobot_disconnect(_comms);
}

int CMobotL::enableButtonCallback(void (*buttonCallback)(CMobotL* mobot, int button, int buttonDown))
{
  return Mobot_enableButtonCallback(
      _comms,
      this,
      (void(*)(void*,int,int))buttonCallback);
}

int CMobotL::disableButtonCallback()
{
  return Mobot_disableButtonCallback(_comms);
}

int CMobotL::isConnected()
{
  return Mobot_isConnected(_comms);
}

int CMobotL::reset()
{
  return Mobot_reset(_comms);
}

int CMobotL::resetToZero()
{
  return Mobot_resetToZero(_comms);
}

int CMobotL::resetToZeroNB()
{
  return Mobot_resetToZeroNB(_comms);
}

