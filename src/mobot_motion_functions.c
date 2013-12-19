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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <ctype.h>
#include "mobot.h"
#include "mobot_internal.h"
#ifndef _WIN32
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <libgen.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <termios.h>
#else
#include <windows.h>
#include <shlobj.h>
#endif

#ifdef _CH_
#include <stdarg.h>
#endif

#ifdef __MACH__
#include <mach/clock.h>
#include <mach/mach.h>
#endif

#ifndef _WIN32
#include <netdb.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <arpa/inet.h>
#else
#include <ws2tcpip.h>
#endif

#include "commands.h"

#define ABS(x) ((x)<0?-(x):(x))

#define DEPRECATED(from, to) \
  fprintf(stderr, "Warning: The function \"%s()\" is deprecated. Please use \"%s()\"\n" , from, to)

int Mobot_motionArch(mobot_t* comms, double angle)
{
  Mobot_moveJointToNB(comms, ROBOT_JOINT2, -angle/2.0);
  Mobot_moveJointToNB(comms, ROBOT_JOINT3, angle/2.0);
  Mobot_moveJointWait(comms, ROBOT_JOINT2);
  Mobot_moveJointWait(comms, ROBOT_JOINT3);
  return 0;
}

#define INIT_MARG \
  motionArg_t* marg; \
  marg = (motionArg_t*)malloc(sizeof(motionArg_t)); \
  marg->mobot = comms;

void* motionArchThread(void* arg)
{
  motionArg_t* marg = (motionArg_t*)arg;
  Mobot_motionArch(marg->mobot, marg->d);
  marg->mobot->motionInProgress--;
  free(marg);
  return NULL;
}

int Mobot_motionArchNB(mobot_t* comms, double angle)
{
  INIT_MARG
  marg->d = angle;
  comms->motionInProgress++;
  THREAD_CREATE(comms->thread, motionArchThread, marg);
  return 0;
}

int Mobot_motionDistance(mobot_t* comms, double distance, double radius)
{
  double theta = distance/radius;
  return Mobot_motionRollForward(comms, theta);
}

void* motionDistanceThread(void* arg)
{
  motionArg_t* marg = (motionArg_t*)arg;
  Mobot_motionRollForward(marg->mobot, marg->d);
  marg->mobot->motionInProgress--;
  free(marg);
  return NULL;
}

int Mobot_motionDistanceNB(mobot_t* comms, double distance, double radius)
{
  INIT_MARG
  marg->d = distance / radius;
  comms->motionInProgress++;
  THREAD_CREATE(comms->thread, motionDistanceThread, marg);
  return 0;
}

int Mobot_motionInchwormLeft(mobot_t* comms, int num)
{
  int i;
  Mobot_moveJointToNB(comms, ROBOT_JOINT2, 0);
  Mobot_moveJointToNB(comms, ROBOT_JOINT3, 0);
  Mobot_moveWait(comms);

  for(i = 0; i < num; i++) {
    Mobot_moveJointTo(comms, ROBOT_JOINT2, DEG2RAD(-50));
    Mobot_moveJointTo(comms, ROBOT_JOINT3, DEG2RAD(50));
    Mobot_moveJointTo(comms, ROBOT_JOINT2, 0);
    Mobot_moveJointTo(comms, ROBOT_JOINT3, 0);
  }

  return 0;
}

void* motionInchwormLeftThread(void* arg)
{
  motionArg_t* marg = (motionArg_t*)arg;
  Mobot_motionInchwormLeft(marg->mobot, marg->i);
  marg->mobot->motionInProgress--;
  free(marg);
  return NULL;
}

int Mobot_motionInchwormLeftNB(mobot_t* comms, int num)
{
  INIT_MARG
  marg->i = num;
  comms->motionInProgress++;
  THREAD_CREATE(comms->thread, motionInchwormLeftThread, marg);
  return 0;
}

int Mobot_motionInchwormRight(mobot_t* comms, int num)
{
  int i;
  Mobot_moveJointToNB(comms, ROBOT_JOINT2, 0);
  Mobot_moveJointToNB(comms, ROBOT_JOINT3, 0);
  Mobot_moveWait(comms);

  for(i = 0; i < num; i++) {
    Mobot_moveJointTo(comms, ROBOT_JOINT3, DEG2RAD(50));
    Mobot_moveJointTo(comms, ROBOT_JOINT2, DEG2RAD(-50));
    Mobot_moveJointTo(comms, ROBOT_JOINT3, 0);
    Mobot_moveJointTo(comms, ROBOT_JOINT2, 0);
  }

  return 0;
}

void* motionInchwormRightThread(void* arg)
{
  motionArg_t *marg = (motionArg_t*)arg;
  Mobot_motionInchwormRight(marg->mobot, marg->i);
  marg->mobot->motionInProgress--;
  free(marg);
  return NULL;
}

int Mobot_motionInchwormRightNB(mobot_t* comms, int num)
{
  INIT_MARG
  marg->i = num;
  comms->motionInProgress++;
  THREAD_CREATE(comms->thread, motionInchwormRightThread, marg);
  return 0;
}

int Mobot_motionRollBackward(mobot_t* comms, double angle)
{
  if(
      (comms->formFactor == MOBOTFORM_I) ||
      (comms->formFactor == MOBOTFORM_L) ||
      (comms->formFactor == MOBOTFORM_T) 
    )
  {
    Mobot_move(comms, -angle, 0, angle, 0 );
  } else {
    Mobot_move(comms, -angle, 0, 0, -angle);
  }
  return 0;
}

void* motionRollBackwardThread(void* arg)
{
  motionArg_t* marg = (motionArg_t*)arg;
  Mobot_motionRollBackward(marg->mobot, marg->d);
  marg->mobot->motionInProgress--;
  free(marg);
  return NULL;
}

int Mobot_motionRollBackwardNB(mobot_t* comms, double angle)
{
  INIT_MARG
  marg->d = angle;
  comms->motionInProgress++;
  THREAD_CREATE(comms->thread, motionRollBackwardThread, marg);
  return 0;
}

int Mobot_motionRollForward(mobot_t* comms, double angle)
{
  /*
  double motorPosition[2];
  Mobot_getJointAngle(comms, ROBOT_JOINT1, &motorPosition[0]);
  Mobot_getJointAngle(comms, ROBOT_JOINT4, &motorPosition[1]);
  Mobot_moveJointToNB(comms, ROBOT_JOINT1, motorPosition[0] + angle);
  Mobot_moveJointToNB(comms, ROBOT_JOINT4, motorPosition[1] + angle);
  Mobot_moveWait(comms);
  */
  if(
      (comms->formFactor == MOBOTFORM_I) ||
      (comms->formFactor == MOBOTFORM_L) ||
      (comms->formFactor == MOBOTFORM_T) 
    )
  {
    Mobot_move(comms, angle, 0, -angle, 0 );
  } else {
    Mobot_move(comms, angle, 0, 0, angle);
  }
  return 0;
}

void* motionRollForwardThread(void* arg)
{
  motionArg_t* marg = (motionArg_t*)arg;
  Mobot_motionRollForward(marg->mobot, marg->d);
  marg->mobot->motionInProgress--;
  free(marg);
  return NULL;
}

int Mobot_motionRollForwardNB(mobot_t* comms, double angle)
{
  INIT_MARG
  marg->d = angle;
  comms->motionInProgress++;
  THREAD_CREATE(comms->thread, motionRollForwardThread, marg);
  return 0;
}

int Mobot_motionStand(mobot_t* comms)
{
  double speed;
  Mobot_resetToZero(comms);
  Mobot_moveJointTo(comms, ROBOT_JOINT2, DEG2RAD(-85));
  Mobot_moveJointTo(comms, ROBOT_JOINT3, DEG2RAD(70));
  Mobot_moveWait(comms);
  Mobot_moveJointTo(comms, ROBOT_JOINT1, DEG2RAD(45));
  /* Sleep for a second, wait for it to settle down */
#ifdef _WIN32
  Sleep(1000);
#else
  sleep(1);
#endif
  Mobot_moveJointTo(comms, ROBOT_JOINT2, DEG2RAD(20));
  //Mobot_setJointSpeed(comms, ROBOT_JOINT2, speed);
  return 0;
}

void* motionStandThread(void* arg)
{
  motionArg_t* marg = (motionArg_t*)arg;
  Mobot_motionStand(marg->mobot);
  marg->mobot->motionInProgress--;
  free(marg);
  return NULL;
}

int Mobot_motionStandNB(mobot_t* comms)
{
  INIT_MARG
  comms->motionInProgress++;
  THREAD_CREATE(comms->thread, motionStandThread, marg);
  return 0;
}

int Mobot_motionSkinny(mobot_t* comms, double angle)
{
  Mobot_moveJointToNB(comms, ROBOT_JOINT2, angle);
  Mobot_moveJointToNB(comms, ROBOT_JOINT3, angle);
  Mobot_moveWait(comms);
  return 0;
}

void* motionSkinnyThread(void* arg)
{
  motionArg_t* marg = (motionArg_t*)arg;
  Mobot_motionSkinny(marg->mobot, marg->d);
  marg->mobot->motionInProgress--;
  free(marg);
  return NULL;
}

int Mobot_motionSkinnyNB(mobot_t* comms, double angle)
{
  INIT_MARG
  marg->d = angle;
  comms->motionInProgress++;
  THREAD_CREATE(comms->thread, motionSkinnyThread, marg);
  return 0;
}

int Mobot_motionTurnLeft(mobot_t* comms, double angle)
{
  if(
      (comms->formFactor == MOBOTFORM_I) ||
      (comms->formFactor == MOBOTFORM_L) ||
      (comms->formFactor == MOBOTFORM_T) 
    )
  {
    Mobot_move(comms, -angle, 0, -angle, 0 );
  } else {
    Mobot_move(comms, -angle, 0, 0, angle);
  }
  return 0;
}

void* motionTurnLeftThread(void* arg)
{
  motionArg_t* marg = (motionArg_t*)arg;
  Mobot_motionTurnLeft(marg->mobot, marg->d);
  marg->mobot->motionInProgress--;
  free(marg);
  return NULL;
}

int Mobot_motionTurnLeftNB(mobot_t* comms, double angle)
{
  INIT_MARG
  marg->d = angle;
  comms->motionInProgress++;
  THREAD_CREATE(comms->thread, motionTurnLeftThread, marg);
  return 0;
}

int Mobot_motionTurnRight(mobot_t* comms, double angle)
{
  if(
      (comms->formFactor == MOBOTFORM_I) ||
      (comms->formFactor == MOBOTFORM_L) ||
      (comms->formFactor == MOBOTFORM_T) 
    )
  {
    Mobot_move(comms, angle, 0, angle, 0 );
  } else {
    Mobot_move(comms, angle, 0, 0, -angle);
  }
  return 0;
}

void* motionTurnRightThread(void* arg)
{
  motionArg_t* marg = (motionArg_t*)arg;
  Mobot_motionTurnRight(marg->mobot, marg->d);
  marg->mobot->motionInProgress--;
  free(marg);
  return NULL;
}

int Mobot_motionTurnRightNB(mobot_t* comms, double angle)
{
  INIT_MARG
  marg->d = angle;
  comms->motionInProgress++;
  THREAD_CREATE(comms->thread, motionTurnRightThread, marg);
  return 0;
}

int Mobot_motionTumbleRight(mobot_t* comms, int num)
{
  int i;
  Mobot_resetToZero(comms);
#ifndef _WIN32
  sleep(1);
#else
  Sleep(1000);
#endif

  for(i = 0; i < num; i++) {
    Mobot_moveJointTo(comms, ROBOT_JOINT3, DEG2RAD(85));
    Mobot_moveJointTo(comms, ROBOT_JOINT2, DEG2RAD(-80));
    Mobot_moveJointTo(comms, ROBOT_JOINT3, DEG2RAD(0));
    Mobot_moveJointTo(comms, ROBOT_JOINT2, DEG2RAD(0));
    Mobot_moveJointTo(comms, ROBOT_JOINT3, DEG2RAD(-80));
    Mobot_moveJointTo(comms, ROBOT_JOINT3, DEG2RAD(-45));
    Mobot_moveJointTo(comms, ROBOT_JOINT2, DEG2RAD(85));
    Mobot_moveJointTo(comms, ROBOT_JOINT3, DEG2RAD(-80));
    Mobot_moveJointTo(comms, ROBOT_JOINT2, DEG2RAD(0));
    Mobot_moveJointTo(comms, ROBOT_JOINT3, DEG2RAD(0));
    Mobot_moveJointTo(comms, ROBOT_JOINT2, DEG2RAD(-80));
    if(i != (num-1)) {
      Mobot_moveJointTo(comms, ROBOT_JOINT2, DEG2RAD(-45));
    }
  }
  Mobot_moveJointToNB(comms, ROBOT_JOINT3, 0);
  Mobot_moveJointToNB(comms, ROBOT_JOINT2, 0);
  Mobot_moveWait(comms);
  return 0;
}

void* motionTumbleRightThread(void* arg)
{
  motionArg_t* marg = (motionArg_t*)arg;
  Mobot_motionTumbleRight(marg->mobot, marg->i);
  marg->mobot->motionInProgress--;
  free(marg);
  return NULL;
}

int Mobot_motionTumbleRightNB(mobot_t* comms, int num)
{
  INIT_MARG
  marg->i = num;
  comms->motionInProgress++;
  THREAD_CREATE(comms->thread, motionTumbleRightThread, marg);
  return 0;
}

int Mobot_motionTumbleLeft(mobot_t* comms, int num)
{
  int i;
  Mobot_resetToZero(comms);
#ifndef _WIN32
  sleep(1);
#else
  Sleep(1000);
#endif

  for(i = 0; i < num; i++) {
    Mobot_moveJointTo(comms, ROBOT_JOINT2, DEG2RAD(-85));
    Mobot_moveJointTo(comms, ROBOT_JOINT3, DEG2RAD(80));
    Mobot_moveJointTo(comms, ROBOT_JOINT2, DEG2RAD(0));
    Mobot_moveJointTo(comms, ROBOT_JOINT3, DEG2RAD(0));
    Mobot_moveJointTo(comms, ROBOT_JOINT2, DEG2RAD(80));
    Mobot_moveJointTo(comms, ROBOT_JOINT2, DEG2RAD(45));
    Mobot_moveJointTo(comms, ROBOT_JOINT3, DEG2RAD(-85));
    Mobot_moveJointTo(comms, ROBOT_JOINT2, DEG2RAD(80));
    Mobot_moveJointTo(comms, ROBOT_JOINT3, DEG2RAD(0));
    Mobot_moveJointTo(comms, ROBOT_JOINT2, DEG2RAD(0));
    Mobot_moveJointTo(comms, ROBOT_JOINT3, DEG2RAD(80));
    if(i != (num-1)) {
      Mobot_moveJointTo(comms, ROBOT_JOINT3, DEG2RAD(45));
    }
  }
  Mobot_moveJointToNB(comms, ROBOT_JOINT2, 0);
  Mobot_moveJointToNB(comms, ROBOT_JOINT3, 0);
  Mobot_moveWait(comms);
  return 0;
}

void* motionTumbleLeftThread(void* arg)
{
  motionArg_t* marg = (motionArg_t*)arg;
  Mobot_motionTumbleLeft(marg->mobot, marg->i);
  marg->mobot->motionInProgress--;
  free(marg);
  return NULL;
}

int Mobot_motionTumbleLeftNB(mobot_t* comms, int num)
{
  INIT_MARG
  marg->i = num;
  comms->motionInProgress++;
  THREAD_CREATE(comms->thread, motionTumbleLeftThread, marg);
  return 0;
}

int Mobot_motionUnstand(mobot_t* comms)
{
  double speed;
  Mobot_moveToDirect(comms, 0, 0, 0, 0);
  Mobot_moveJointToNB(comms, ROBOT_JOINT3, DEG2RAD(45));
  Mobot_moveJointToNB(comms, ROBOT_JOINT2, DEG2RAD(-85));
  Mobot_moveWait(comms);
  Mobot_moveToDirect(comms, 0, 0, 0, 0);
  return 0;
}

void* motionUnstandThread(void* arg)
{
  motionArg_t* marg = (motionArg_t*)arg;
  Mobot_motionUnstand(marg->mobot);
  marg->mobot->motionInProgress--;
  free(marg);
  return NULL;
}

int Mobot_motionUnstandNB(mobot_t* comms)
{
  INIT_MARG
  comms->motionInProgress++;
  THREAD_CREATE(comms->thread, motionUnstandThread, marg);
  return 0;
}

int Mobot_motionWait(mobot_t* comms)
{
  while(comms->motionInProgress) {
#ifdef _WIN32
    Sleep(200);
#else
    usleep(200000);
#endif
  }
  Mobot_moveWait(comms);
  return 0;
}
