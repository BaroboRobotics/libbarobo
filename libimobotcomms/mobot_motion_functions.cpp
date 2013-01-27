
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
  Mobot_moveJointToNB(comms, MOBOT_JOINT2, -angle/2.0);
  Mobot_moveJointToNB(comms, MOBOT_JOINT3, angle/2.0);
  Mobot_moveJointWait(comms, MOBOT_JOINT2);
  Mobot_moveJointWait(comms, MOBOT_JOINT3);
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
  Mobot_moveJointToNB(comms, MOBOT_JOINT2, 0);
  Mobot_moveJointToNB(comms, MOBOT_JOINT3, 0);
  Mobot_moveWait(comms);

  for(i = 0; i < num; i++) {
    Mobot_moveJointTo(comms, MOBOT_JOINT2, DEG2RAD(-50));
    Mobot_moveJointTo(comms, MOBOT_JOINT3, DEG2RAD(50));
    Mobot_moveJointTo(comms, MOBOT_JOINT2, 0);
    Mobot_moveJointTo(comms, MOBOT_JOINT3, 0);
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
  Mobot_moveJointToNB(comms, MOBOT_JOINT2, 0);
  Mobot_moveJointToNB(comms, MOBOT_JOINT3, 0);
  Mobot_moveWait(comms);

  for(i = 0; i < num; i++) {
    Mobot_moveJointTo(comms, MOBOT_JOINT3, DEG2RAD(50));
    Mobot_moveJointTo(comms, MOBOT_JOINT2, DEG2RAD(-50));
    Mobot_moveJointTo(comms, MOBOT_JOINT3, 0);
    Mobot_moveJointTo(comms, MOBOT_JOINT2, 0);
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
  Mobot_move(comms, -angle, 0, 0, -angle);
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
  Mobot_getJointAngle(comms, MOBOT_JOINT1, &motorPosition[0]);
  Mobot_getJointAngle(comms, MOBOT_JOINT4, &motorPosition[1]);
  Mobot_moveJointToNB(comms, MOBOT_JOINT1, motorPosition[0] + angle);
  Mobot_moveJointToNB(comms, MOBOT_JOINT4, motorPosition[1] + angle);
  Mobot_moveWait(comms);
  */
  Mobot_move(comms, angle, 0, 0, angle);
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
  Mobot_moveJointTo(comms, MOBOT_JOINT2, DEG2RAD(-85));
  Mobot_moveJointTo(comms, MOBOT_JOINT3, DEG2RAD(70));
  Mobot_moveWait(comms);
  Mobot_moveJointTo(comms, MOBOT_JOINT1, DEG2RAD(45));
  /* Sleep for a second, wait for it to settle down */
#ifdef _WIN32
  Sleep(1000);
#else
  sleep(1);
#endif
  Mobot_moveJointTo(comms, MOBOT_JOINT2, DEG2RAD(20));
  //Mobot_setJointSpeed(comms, MOBOT_JOINT2, speed);
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
  Mobot_moveJointToNB(comms, MOBOT_JOINT2, angle);
  Mobot_moveJointToNB(comms, MOBOT_JOINT3, angle);
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
  Mobot_move(comms, -angle, 0, 0, angle);
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
  Mobot_move(comms, angle, 0, 0, -angle);
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
    Mobot_moveJointTo(comms, MOBOT_JOINT3, DEG2RAD(85));
    Mobot_moveJointTo(comms, MOBOT_JOINT2, DEG2RAD(-80));
    Mobot_moveJointTo(comms, MOBOT_JOINT3, DEG2RAD(0));
    Mobot_moveJointTo(comms, MOBOT_JOINT2, DEG2RAD(0));
    Mobot_moveJointTo(comms, MOBOT_JOINT3, DEG2RAD(-80));
    Mobot_moveJointTo(comms, MOBOT_JOINT3, DEG2RAD(-45));
    Mobot_moveJointTo(comms, MOBOT_JOINT2, DEG2RAD(85));
    Mobot_moveJointTo(comms, MOBOT_JOINT3, DEG2RAD(-80));
    Mobot_moveJointTo(comms, MOBOT_JOINT2, DEG2RAD(0));
    Mobot_moveJointTo(comms, MOBOT_JOINT3, DEG2RAD(0));
    Mobot_moveJointTo(comms, MOBOT_JOINT2, DEG2RAD(-80));
    if(i != (num-1)) {
      Mobot_moveJointTo(comms, MOBOT_JOINT2, DEG2RAD(-45));
    }
  }
  Mobot_moveJointToNB(comms, MOBOT_JOINT3, 0);
  Mobot_moveJointToNB(comms, MOBOT_JOINT2, 0);
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
    Mobot_moveJointTo(comms, MOBOT_JOINT2, DEG2RAD(-85));
    Mobot_moveJointTo(comms, MOBOT_JOINT3, DEG2RAD(80));
    Mobot_moveJointTo(comms, MOBOT_JOINT2, DEG2RAD(0));
    Mobot_moveJointTo(comms, MOBOT_JOINT3, DEG2RAD(0));
    Mobot_moveJointTo(comms, MOBOT_JOINT2, DEG2RAD(80));
    Mobot_moveJointTo(comms, MOBOT_JOINT2, DEG2RAD(45));
    Mobot_moveJointTo(comms, MOBOT_JOINT3, DEG2RAD(-85));
    Mobot_moveJointTo(comms, MOBOT_JOINT2, DEG2RAD(80));
    Mobot_moveJointTo(comms, MOBOT_JOINT3, DEG2RAD(0));
    Mobot_moveJointTo(comms, MOBOT_JOINT2, DEG2RAD(0));
    Mobot_moveJointTo(comms, MOBOT_JOINT3, DEG2RAD(80));
    if(i != (num-1)) {
      Mobot_moveJointTo(comms, MOBOT_JOINT3, DEG2RAD(45));
    }
  }
  Mobot_moveJointToNB(comms, MOBOT_JOINT2, 0);
  Mobot_moveJointToNB(comms, MOBOT_JOINT3, 0);
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
  Mobot_moveJointToNB(comms, MOBOT_JOINT3, DEG2RAD(45));
  Mobot_moveJointToNB(comms, MOBOT_JOINT2, DEG2RAD(-85));
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
#ifdef __cplusplus
int CMobot::motionArch(double angle)
{
  return Mobot_motionArch(_comms, DEG2RAD(angle));
}

int CMobot::motionDistance(double distance, double radius)
{
  return Mobot_motionDistance(_comms, distance, radius);
}

int CMobot::motionDistanceNB(double distance, double radius)
{
  return Mobot_motionDistanceNB(_comms, distance, radius);
}

int CMobot::motionArchNB(double angle)
{
  return Mobot_motionArchNB(_comms, DEG2RAD(angle));
}

int CMobot::motionInchwormLeft(int num)
{
  return Mobot_motionInchwormLeft(_comms, num);
}

int CMobot::motionInchwormLeftNB(int num)
{
  return Mobot_motionInchwormLeftNB(_comms, num);
}

int CMobot::motionInchwormRight(int num)
{
  return Mobot_motionInchwormRight(_comms, num);
}

int CMobot::motionInchwormRightNB(int num)
{
  return Mobot_motionInchwormRightNB(_comms, num);
}

int CMobot::motionRollBackward(double angle)
{
  return Mobot_motionRollBackward(_comms, DEG2RAD(angle));
}

int CMobot::motionRollBackwardNB(double angle)
{
  return Mobot_motionRollBackwardNB(_comms, DEG2RAD(angle));
}

int CMobot::motionRollForward(double angle)
{
  return Mobot_motionRollForward(_comms, DEG2RAD(angle));
}

int CMobot::motionRollForwardNB(double angle)
{
  return Mobot_motionRollForwardNB(_comms, DEG2RAD(angle));
}

int CMobot::motionSkinny(double angle)
{
  return Mobot_motionSkinny(_comms, DEG2RAD(angle));
}

int CMobot::motionSkinnyNB(double angle)
{
  return Mobot_motionSkinnyNB(_comms, DEG2RAD(angle));
}

int CMobot::motionStand()
{
  return Mobot_motionStand(_comms);
}

int CMobot::motionStandNB()
{
  return Mobot_motionStandNB(_comms);
}

int CMobot::motionTumbleRight(int num)
{
  return Mobot_motionTumbleRight(_comms, num);
}

int CMobot::motionTumbleRightNB(int num)
{
  return Mobot_motionTumbleRightNB(_comms, num);
}

int CMobot::motionTumbleLeft(int num)
{
  return Mobot_motionTumbleLeft(_comms, num);
}

int CMobot::motionTumbleLeftNB(int num)
{
  return Mobot_motionTumbleLeftNB(_comms, num);
}

int CMobot::motionTurnLeft(double angle)
{
  return Mobot_motionTurnLeft(_comms, DEG2RAD(angle));
}

int CMobot::motionTurnLeftNB(double angle)
{
  return Mobot_motionTurnLeftNB(_comms, DEG2RAD(angle));
}

int CMobot::motionTurnRight(double angle)
{
  return Mobot_motionTurnRight(_comms, DEG2RAD(angle));
}

int CMobot::motionTurnRightNB(double angle)
{
  return Mobot_motionTurnRightNB(_comms, DEG2RAD(angle));
}

int CMobot::motionUnstand()
{
  return Mobot_motionUnstand(_comms);
}

int CMobot::motionUnstandNB()
{
  return Mobot_motionUnstandNB(_comms);
}

int CMobot::motionWait()
{
  return Mobot_motionWait(_comms);
}

int CMobotGroup::motionArch(double angle) 
{
  argDouble = angle;
  _motionInProgress++;
  motionArchThread(this);
  return 0;
}

int CMobotGroup::motionArchNB(double angle) 
{
  argDouble = angle;
  _motionInProgress++;
  THREAD_CREATE(_thread, motionArchThread, this);
  return 0;
}

void* CMobotGroup::motionArchThread(void* arg) 
{
  CMobotGroup *cmg = (CMobotGroup*)arg;
  cmg->moveJointToNB(MOBOT_JOINT2, -cmg->argDouble/2);
  cmg->moveJointToNB(MOBOT_JOINT3, cmg->argDouble/2);
  cmg->moveJointWait(MOBOT_JOINT2);
  cmg->moveJointWait(MOBOT_JOINT3);
  cmg->_motionInProgress--;
  return NULL;
}

int CMobotGroup::motionDistance(double distance, double radius)
{
  argDouble = distance / radius;
  _motionInProgress++;
  motionDistanceThread(this);
  return 0;
}

int CMobotGroup::motionDistanceNB(double distance, double radius)
{
  argDouble = distance / radius;
  _motionInProgress++;
  THREAD_CREATE(_thread, motionDistanceThread, this);
  return 0;
}

void* CMobotGroup::motionDistanceThread(void* arg)
{
  CMobotGroup *cmg = (CMobotGroup*)arg;
  cmg->move(RAD2DEG(cmg->argDouble), 0, 0, RAD2DEG(cmg->argDouble));
  cmg->_motionInProgress--;
  return NULL;
}

int CMobotGroup::motionInchwormLeft(int num)
{
  argInt = num;
  _motionInProgress++;
  motionInchwormLeftThread(this);
  return 0;
}

int CMobotGroup::motionInchwormLeftNB(int num)
{
  argInt = num;
  _motionInProgress++;
  THREAD_CREATE(_thread, motionInchwormLeftThread, this);
  return 0;
}

void* CMobotGroup::motionInchwormLeftThread(void* arg)
{
  int i;
  CMobotGroup *cmg = (CMobotGroup*)arg;
  cmg->moveJointToNB(MOBOT_JOINT2, 0);
  cmg->moveJointToNB(MOBOT_JOINT3, 0);
  cmg->moveWait();
  for(i = 0; i < cmg->argInt ; i++) {
    cmg->moveJointTo(MOBOT_JOINT2, -50);
    cmg->moveJointTo(MOBOT_JOINT3, 50);
    cmg->moveJointTo(MOBOT_JOINT2, 0);
    cmg->moveJointTo(MOBOT_JOINT3, 0);
  }
  cmg->_motionInProgress--;
  return NULL;
}

int CMobotGroup::motionInchwormRight(int num)
{
  argInt = num;
  _motionInProgress++;
  motionInchwormRightThread(this);
  return 0;
}

int CMobotGroup::motionInchwormRightNB(int num)
{
  argInt = num;
  _motionInProgress++;
  THREAD_CREATE(_thread, motionInchwormRightThread, this);
  return 0;
}

void* CMobotGroup::motionInchwormRightThread(void* arg)
{
  int i;
  CMobotGroup *cmg = (CMobotGroup*)arg;

  cmg->moveJointToNB(MOBOT_JOINT2, 0);
  cmg->moveJointToNB(MOBOT_JOINT3, 0);
  cmg->moveWait();
  for(i = 0; i < cmg->argInt; i++) {
    cmg->moveJointTo(MOBOT_JOINT3, 50);
    cmg->moveJointTo(MOBOT_JOINT2, -50);
    cmg->moveJointTo(MOBOT_JOINT3, 0);
    cmg->moveJointTo(MOBOT_JOINT2, 0);
  }
  cmg->_motionInProgress--;
  return NULL;
}

int CMobotGroup::motionRollBackward(double angle)
{
  argDouble = angle;
  _motionInProgress++;
  motionRollBackwardThread(this);
  return 0;
}

int CMobotGroup::motionRollBackwardNB(double angle)
{
  argDouble = angle;
  _motionInProgress++;
  THREAD_CREATE(_thread, motionRollBackwardThread, this);
  return 0;
}

void* CMobotGroup::motionRollBackwardThread(void* arg)
{
  CMobotGroup *cmg = (CMobotGroup*)arg;
  cmg->move(-cmg->argDouble, 0, 0, -cmg->argDouble);
  cmg->_motionInProgress--;
  return NULL;
}

int CMobotGroup::motionRollForward(double angle)
{
  argDouble = angle;
  _motionInProgress++;
  motionRollForwardThread(this);
  return 0;
}

int CMobotGroup::motionRollForwardNB(double angle)
{
  argDouble = angle;
  _motionInProgress++;
  THREAD_CREATE(_thread, motionRollForwardThread, this);
  return 0;
}

void* CMobotGroup::motionRollForwardThread(void* arg)
{
  CMobotGroup *cmg = (CMobotGroup*)arg;
  cmg->move(cmg->argDouble, 0, 0, cmg->argDouble);
  cmg->_motionInProgress--;
  return NULL;
}

int CMobotGroup::motionSkinny(double angle)
{
  argDouble = angle;
  _motionInProgress++;
  motionSkinnyThread(this);
  return 0;
}

int CMobotGroup::motionSkinnyNB(double angle)
{
  argDouble = angle;
  _motionInProgress++;
  THREAD_CREATE(_thread, motionSkinnyThread, this);
  return 0;
}

void* CMobotGroup::motionSkinnyThread(void* arg)
{
  CMobotGroup *cmg = (CMobotGroup*)arg;
  cmg->moveJointToNB(MOBOT_JOINT2, cmg->argDouble);
  cmg->moveJointToNB(MOBOT_JOINT3, cmg->argDouble);
  cmg->_motionInProgress--;
  return NULL;
}

int CMobotGroup::motionStand()
{
  _motionInProgress++;
  motionStandThread(this);
  return 0;
}

int CMobotGroup::motionStandNB()
{
  _motionInProgress++;
  THREAD_CREATE(_thread, motionStandThread, NULL);
  return 0;
}

void* CMobotGroup::motionStandThread(void* arg)
{
  CMobotGroup* cmg = (CMobotGroup*)arg;
  cmg->resetToZero();
  cmg->moveJointTo(MOBOT_JOINT2, -85);
  cmg->moveJointTo(MOBOT_JOINT3, 70);
  cmg->moveWait();
  cmg->moveJointTo(MOBOT_JOINT1, 45);
  cmg->moveJointTo(MOBOT_JOINT2, 20);
  cmg->_motionInProgress--;
  return 0;
}

int CMobotGroup::motionTurnLeft(double angle)
{
  argDouble = angle;
  _motionInProgress++;
  motionTurnLeftThread(this);
  return 0;
}

int CMobotGroup::motionTurnLeftNB(double angle)
{
  argDouble = angle;
  _motionInProgress++;
  THREAD_CREATE(_thread, motionTurnLeftThread, this);
  return 0;
}

void* CMobotGroup::motionTurnLeftThread(void* arg)
{
  CMobotGroup* cmg = (CMobotGroup*)arg;
  cmg->move(-cmg->argDouble, 0, 0, cmg->argDouble);
  cmg->_motionInProgress--;
  return NULL;
}

int CMobotGroup::motionTurnRight(double angle)
{
  argDouble = angle;
  _motionInProgress++;
  motionTurnRightThread(this);
  return 0;
}

int CMobotGroup::motionTurnRightNB(double angle)
{
  argDouble = angle;
  _motionInProgress++;
  THREAD_CREATE(_thread, motionTurnRightThread, this);
  return 0;
}

void* CMobotGroup::motionTurnRightThread(void* arg)
{
  CMobotGroup* cmg = (CMobotGroup*)arg;
  cmg->move(cmg->argDouble, 0, 0, -cmg->argDouble);
  cmg->_motionInProgress--;
  return NULL;
}

int CMobotGroup::motionTumbleRight(int num)
{
  argInt = num;
  _motionInProgress++;
  motionTumbleRightThread(this);
  return 0;
}

int CMobotGroup::motionTumbleRightNB(int num)
{
  argInt = num;
  _motionInProgress++;
  THREAD_CREATE(_thread, motionTumbleRightThread, this);
  return 0;
}

void* CMobotGroup::motionTumbleRightThread(void* arg)
{
  int i;
  CMobotGroup* cmg = (CMobotGroup*)arg;
  int num = cmg->argInt;

  cmg->resetToZero();
#ifndef _WIN32
  sleep(1);
#else
  Sleep(1000);
#endif

  for(i = 0; i < num; i++) {
    cmg->moveJointTo(MOBOT_JOINT3, DEG2RAD(85));
    cmg->moveJointTo(MOBOT_JOINT2, DEG2RAD(-80));
    cmg->moveJointTo(MOBOT_JOINT3, DEG2RAD(0));
    cmg->moveJointTo(MOBOT_JOINT2, DEG2RAD(0));
    cmg->moveJointTo(MOBOT_JOINT3, DEG2RAD(-80));
    cmg->moveJointTo(MOBOT_JOINT3, DEG2RAD(-45));
    cmg->moveJointTo(MOBOT_JOINT2, DEG2RAD(85));
    cmg->moveJointTo(MOBOT_JOINT3, DEG2RAD(-80));
    cmg->moveJointTo(MOBOT_JOINT2, DEG2RAD(0));
    cmg->moveJointTo(MOBOT_JOINT3, DEG2RAD(0));
    cmg->moveJointTo(MOBOT_JOINT2, DEG2RAD(-80));
    if(i != (num-1)) {
      cmg->moveJointTo(MOBOT_JOINT2, DEG2RAD(-45));
    }
  }
  cmg->moveJointToNB(MOBOT_JOINT3, 0);
  cmg->moveJointToNB(MOBOT_JOINT2, 0);
  cmg->moveWait();

  cmg->_motionInProgress--;
  return NULL;
}

int CMobotGroup::motionTumbleLeft(int num)
{
  argInt = num;
  _motionInProgress++;
  motionTumbleLeftThread(this);
  return 0;
}

int CMobotGroup::motionTumbleLeftNB(int num)
{
  argInt = num;
  _motionInProgress++;
  THREAD_CREATE(_thread, motionTumbleLeftThread, this);
  return 0;
}

void* CMobotGroup::motionTumbleLeftThread(void* arg)
{
  int i;
  CMobotGroup* cmg = (CMobotGroup*)arg;
  int num = cmg->argInt;

  cmg->resetToZero();
#ifndef _WIN32
  sleep(1);
#else
  Sleep(1000);
#endif

  for(i = 0; i < num; i++) {
    cmg->moveJointTo(MOBOT_JOINT2, DEG2RAD(-85));
    cmg->moveJointTo(MOBOT_JOINT3, DEG2RAD(80));
    cmg->moveJointTo(MOBOT_JOINT2, DEG2RAD(0));
    cmg->moveJointTo(MOBOT_JOINT3, DEG2RAD(0));
    cmg->moveJointTo(MOBOT_JOINT2, DEG2RAD(80));
    cmg->moveJointTo(MOBOT_JOINT2, DEG2RAD(45));
    cmg->moveJointTo(MOBOT_JOINT3, DEG2RAD(-85));
    cmg->moveJointTo(MOBOT_JOINT2, DEG2RAD(80));
    cmg->moveJointTo(MOBOT_JOINT3, DEG2RAD(0));
    cmg->moveJointTo(MOBOT_JOINT2, DEG2RAD(0));
    cmg->moveJointTo(MOBOT_JOINT3, DEG2RAD(80));
    if(i != (num-1)) {
      cmg->moveJointTo(MOBOT_JOINT3, DEG2RAD(45));
    }
  }
  cmg->moveJointToNB(MOBOT_JOINT2, 0);
  cmg->moveJointToNB(MOBOT_JOINT3, 0);
  cmg->moveWait();
  cmg->_motionInProgress--;
  return NULL;
}

int CMobotGroup::motionUnstand()
{
  _motionInProgress++;
  motionUnstandThread(this);
  return 0;
}

int CMobotGroup::motionUnstandNB()
{
  _motionInProgress++;
  THREAD_CREATE(_thread, motionUnstandThread, NULL);
  return 0;
}

void* CMobotGroup::motionUnstandThread(void* arg)
{
  CMobotGroup* cmg = (CMobotGroup*)arg;
  cmg->moveToDirect(0, 0, 0, 0);
  cmg->moveJointTo(MOBOT_JOINT3, 45);
  cmg->moveJointTo(MOBOT_JOINT2, -85);
  cmg->moveWait();
  cmg->moveToDirect(0, 0, 0, 0);
  cmg->moveJointTo(MOBOT_JOINT2, 20);
  cmg->_motionInProgress--;
  return 0;
}

int CMobotGroup::motionWait()
{
  while(_motionInProgress > 0) {
#ifdef _WIN32
    Sleep(200);
#else
    usleep(200000);
#endif
  }
  return 0;
}
#endif //__cplusplus
