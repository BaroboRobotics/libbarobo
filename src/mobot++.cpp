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

#include <string>

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

int CMobot::accelTimeNB(double radius, double acceleration, double time)
{
  return Mobot_accelTimeNB(_comms, radius, deg2rad(acceleration), time);
}

int CMobot::accelToVelocityNB(double radius, double acceleration, double velocity)
{
  return Mobot_accelToVelocityNB(_comms, radius, deg2rad(acceleration), deg2rad(velocity));
}

int CMobot::accelToMaxSpeedNB(double radius, double acceleration)
{
  return Mobot_accelToMaxSpeedNB(_comms, radius, deg2rad(acceleration));
}

int CMobot::accelAngularTimeNB(robotJointId_t id, double acceleration, double time)
{
  return Mobot_accelAngularTimeNB(_comms, id, deg2rad(acceleration), time);
}

int CMobot::accelAngularToVelocityNB(robotJointId_t id, double acceleration, double speed)
{
  return Mobot_accelAngularToVelocityNB(_comms, id, deg2rad(acceleration), deg2rad(speed));
}

int CMobot::accelAngularAngleNB(robotJointId_t id, double acceleration, double angle)
{
  return Mobot_accelAngularAngleNB(_comms, id, deg2rad(acceleration), deg2rad(angle));
}

int CMobot::smoothMoveToNB(
    robotJointId_t id,
    double accel0,
    double accelf,
    double vmax,
    double angle)
{
  return Mobot_smoothMoveToNB(
      _comms,
      id,
      deg2rad(accel0),
      deg2rad(accelf),
      deg2rad(vmax),
      deg2rad(angle));
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
    fprintf(stderr, "Error: Connected robot is not a Mobot-A. %d\n", _comms->formFactor);
    Mobot_disconnect(_comms);
    return -1;
  }
  return 0;
}

int CMobot::connectWithAddress(const char* address, int channel)
{
  return Mobot_connectWithAddress(_comms, address, channel);
}

#ifdef ENABLE_BLUETOOTH
int CMobot::connectWithBluetoothAddress(const char* address, int channel)
{
  return Mobot_connectWithBluetoothAddress(_comms, address, channel);
}
#endif

int CMobot::connectWithIPAddress(const char* address, const char port[])
{
  return Mobot_connectWithIPAddress(_comms, address, port);
}

int CMobot::connectWithTTY(const char* ttyfilename)
{
  return Mobot_connectWithTTY(_comms, ttyfilename);
}

int CMobot::disconnect()
{
  return Mobot_disconnect(_comms);
}

int CMobot::delaySeconds(int seconds)
{
#ifdef _WIN32
  Sleep(seconds*1000);
#else
  sleep(seconds);
#endif
  return 0;
}

int CMobot::enableButtonCallback(void* userdata, void (*buttonCallback)(void* data, int button, int buttonDown))
{
  return Mobot_enableButtonCallback(
      _comms,
      userdata,
      (void(*)(void*,int,int))buttonCallback);
}

int CMobot::disableButtonCallback()
{
  return Mobot_disableButtonCallback(_comms);
}

int CMobot::enableJointEventCallback(void *userdata, 
    void (*jointCallback)(int millis, double j1, double j2, double j3, double j4, int mask, void *userdata))
{
  return Mobot_enableJointEventCallback(_comms, userdata, jointCallback);
}

int CMobot::disableJointEventCallback()
{
  return Mobot_disableJointEventCallback(_comms);
}

int CMobot::enableEventCallback(void (*eventCallback)(const uint8_t *buf, int size, void* userdata),
        void* userdata)
{
  return Mobot_enableEventCallback(_comms, eventCallback, userdata);
}

int CMobot::disableEventCallback()
{
  return Mobot_disableEventCallback(_comms);
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

int CMobot::setAccelEventThreshold(double threshold)
{
  return Mobot_setAccelEventThreshold(_comms, threshold);
}

int CMobot::setJointEventThreshold(int joint, double threshold)
{
  return Mobot_setJointEventThreshold(_comms, joint, DEG2RAD(threshold));
}

int CMobot::systemTime(double &time)
{
  time = ::systemTime();
  return 0;
}

int CMobot::transactMessage(int cmd, void* buf, int size)
{
  return MobotMsgTransaction(_comms, cmd, buf, size);
}

bool CMobot::canFlashFirmware () {
  return Mobot_canFlashFirmware(_comms);
}

int CMobot::flashFirmwareAsync (std::string hexfile,
    Mobot_progressCallbackFunc progressCallback,
    Mobot_completionCallbackFunc completionCallback,
    void* user_data) {
  return Mobot_flashFirmwareAsync(_comms, hexfile.c_str(), progressCallback, completionCallback, user_data);
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
