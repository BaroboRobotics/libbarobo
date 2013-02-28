#include <stdlib.h>
#include "mobot.h"
#include "mobot_internal.h"

CMobot::CMobot()
{
  _comms = (mobot_t*)malloc(sizeof(mobot_t));
  Mobot_init(_comms);
}

CMobot::~CMobot() 
{
  if(_comms->exitState == MOBOT_HOLD) {
    setMovementStateNB(MOBOT_HOLD, MOBOT_HOLD, MOBOT_HOLD, MOBOT_HOLD);
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

int CMobot::blinkLED(double delay, int numBlinks)
{
  return Mobot_blinkLED(_comms, delay, numBlinks);
}

int CMobot::connect()
{
  return Mobot_connect(_comms);
}

int CMobot::connectWithAddress(const char* address, int channel)
{
  return Mobot_connectWithAddress(_comms, address, channel);
}

int CMobot::connectWithBluetoothAddress(const char* address, int channel)
{
  return Mobot_connectWithBluetoothAddress(_comms, address, channel);
}

int CMobot::connectWithIPAddress(const char* address, const char port[])
{
  return Mobot_connectWithIPAddress(_comms, address, port);
}

#ifndef _WIN32
int CMobot::connectWithTTY(const char* ttyfilename)
{
  return Mobot_connectWithTTY(_comms, ttyfilename);
}
#endif

int CMobot::disconnect()
{
  return Mobot_disconnect(_comms);
}

int CMobot::enableButtonCallback(void (*buttonCallback)(CMobot* mobot, int button, int buttonDown))
{
  return Mobot_enableButtonCallback(
      _comms,
      this,
      (void(*)(void*,int,int))buttonCallback);
}

int CMobot::disableButtonCallback()
{
  return Mobot_disableButtonCallback(_comms);
}

int CMobot::isConnected()
{
  return Mobot_isConnected(_comms);
}

int CMobot::reset()
{
  return Mobot_reset(_comms);
}

int CMobot::resetToZero()
{
  return Mobot_resetToZero(_comms);
}

int CMobot::resetToZeroNB()
{
  return Mobot_resetToZeroNB(_comms);
}

/* CMelody */
CMelody::CMelody()
{
  _head = (mobotMelodyNote_t*)malloc(sizeof(mobotMelodyNote_t));
  _head->tempo = 120;
}

CMelody::~CMelody()
{
  /* Free everything... */
  mobotMelodyNote_t *iter, *next;
  for(iter = _head; iter != NULL; iter = next)
  {
    next = iter->next;
    free(iter);
  }
}

void CMelody::setTempo(int bpm)
{
  _head->tempo = bpm;
}

void CMelody::addNote(const char* note, int divider)
{
  Mobot_melodyAddNote(_head, note, divider);
}

