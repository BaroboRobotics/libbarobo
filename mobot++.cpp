/*
   Copyright 2013 Barobo, Inc.

   This file is part of libbarobo.

   Foobar is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Foobar is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdlib.h>
#include "mobot.h"
#include "mobot_internal.h"
#include "linkbot.h"

CMobot::CMobot()
{
  _comms = (mobot_t*)malloc(sizeof(mobot_t));
  Mobot_init(_comms);
}

CMobot::~CMobot() 
{
  if(_comms->connected) {
    if(_comms->exitState == ROBOT_HOLD) {
      setMovementStateNB(ROBOT_HOLD, ROBOT_HOLD, ROBOT_HOLD, ROBOT_HOLD);
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

int CMobot::blinkLED(double delay, int numBlinks)
{
  return Mobot_blinkLED(_comms, delay, numBlinks);
}

int CMobot::connect()
{
  int rc = Mobot_connect(_comms);
  if(rc) {
    return rc;
  }
  if(_comms->formFactor != MOBOTFORM_ORIGINAL) {
    fprintf(stderr, "Error: Connected Mobot is not a Mobot-A.\n");
    Mobot_disconnect(_comms);
    return -1;
  }
  return 0;
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

