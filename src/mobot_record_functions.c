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

#define MAX_RETRIES 3

void* Mobot_recordAngleThread(void* arg);
int Mobot_recordAngle(mobot_t* comms, robotJointId_t id, double* time, double* angle, int num, double timeInterval, int shiftData)
{
  THREAD_T thread;
  recordAngleArg_t *rArg;
  int msecs = timeInterval * 1000;
  if(comms->recordingEnabled[id-1]) {
    return -1;
  }
  rArg = (recordAngleArg_t*)malloc(sizeof(recordAngleArg_t));
  rArg->comms = comms;
  rArg->id = id;
  rArg->time = time;
  rArg->angle = angle;
  rArg->num = num;
  rArg->msecs = msecs;
  comms->recordingEnabled[id-1] = 1;
  comms->recordedAngles[0] = &angle;
  comms->recordedTimes = &time;
  comms->shiftData = shiftData;
  THREAD_CREATE(&thread, Mobot_recordAngleThread, rArg);
  return 0;
}

#ifndef _WIN32
unsigned int diff_msecs(struct timespec t1, struct timespec t2)
{
  unsigned int t;
  t = (t2.tv_sec - t1.tv_sec) * 1000;
  t += (t2.tv_nsec - t1.tv_nsec) / 1000000;
  return t;
}
#endif

int Mobot_enableRecordDataShift(mobot_t* comms)
{
  comms->shiftDataGlobalEnable = 1;
  comms->shiftDataGlobal = 1;
  return 0;
}

int Mobot_disableRecordDataShift(mobot_t* comms)
{
  comms->shiftDataGlobalEnable = 1;
  comms->shiftDataGlobal = 0;
  return 0;
}

void* Mobot_recordAngleThread(void* arg)
{
  double angles[4];
  int rc;
  int retries = 0;
  int *isMoving;
  recordAngleArg_t *rArg = (recordAngleArg_t*) arg;
  isMoving = (int*)malloc(sizeof(int) * rArg->num);
#ifndef _WIN32
  int i;
  struct timespec cur_time, itime;
  unsigned int dt;
  double start_time;
  for(i = 0; i < rArg->num; i++) {
#ifndef __MACH__
    clock_gettime(CLOCK_REALTIME, &cur_time);
#else
    clock_serv_t cclock;
    mach_timespec_t mts;
    host_get_clock_service(mach_host_self(), CALENDAR_CLOCK, &cclock);
    clock_get_time(cclock, &mts);
    mach_port_deallocate(mach_task_self(), cclock);
    cur_time.tv_sec = mts.tv_sec;
    cur_time.tv_nsec = mts.tv_nsec;
#endif
    //Mobot_getJointAngleTime(rArg->comms, rArg->id, &rArg->time[i], &rArg->angle[i]);
    while(
        (rc = Mobot_getJointAnglesTimeIsMoving(
                                               rArg->comms, 
                                               &rArg->time[i],
                                               &angles[0],
                                               &angles[1],
                                               &angles[2],
                                               &angles[3],
                                               &isMoving[i])) &&
        retries <= MAX_RETRIES)
    {
      retries++;
    }
    if(rc) break;
    retries = 0;
    rArg->angle[i] = angles[rArg->id-1];
    if(i == 0) {
      start_time = rArg->time[i];
    }
    rArg->time[i] = rArg->time[i] - start_time;
    /* Convert angle to degrees */
    rArg->angle[i] = RAD2DEG(rArg->angle[i]);
#ifndef __MACH__
    clock_gettime(CLOCK_REALTIME, &itime);
#else
    host_get_clock_service(mach_host_self(), CALENDAR_CLOCK, &cclock);
    clock_get_time(cclock, &mts);
    mach_port_deallocate(mach_task_self(), cclock);
    cur_time.tv_sec = mts.tv_sec;
    cur_time.tv_nsec = mts.tv_nsec;
#endif
    dt = diff_msecs(cur_time, itime);
    if(dt < (rArg->msecs)) {
      usleep(rArg->msecs*1000 - dt*1000);
    }
  }
#else
  int i;
  DWORD cur_time, itime;
  unsigned int dt;
  double start_time;
  for(i = 0; i < rArg->num; i++) {
    cur_time = GetTickCount();
    while(
        (rc = Mobot_getJointAnglesTimeIsMoving(
                                               rArg->comms, 
                                               &rArg->time[i],
                                               &angles[0],
                                               &angles[1],
                                               &angles[2],
                                               &angles[3],
                                               &isMoving[i])) &&
        retries <= MAX_RETRIES)
    {
      retries++;
    }
    if(rc) break;
    retries = 0;
    rArg->angle[i] = angles[rArg->id-1];
    if(i == 0) {
      start_time = rArg->time[i];
    }
    rArg->time[i] = rArg->time[i] - start_time;
    /* Convert angle to degrees */
    rArg->angle[i] = RAD2DEG(rArg->angle[i]);
    itime = GetTickCount();
    dt = itime - cur_time;
    if(dt < (rArg->msecs)) {
      Sleep(rArg->msecs - dt);
    }
  }
#endif
  double shiftTime;
  int shiftTimeIndex;
  if( shiftDataIsEnabled(rArg->comms) ) {
    for(i = 0; i < rArg->num; i++) {
      if( isMoving[i] )
      {
        shiftTime = rArg->time[i];
        shiftTimeIndex = i;
        break;
      }
    }
    for(i = 0; i < rArg->num; i++) {
      if(i < shiftTimeIndex) {
        rArg->time[i] = 0;
        rArg->angle[i] = rArg->angle[shiftTimeIndex];
      } else {
        rArg->time[i] = rArg->time[i] - shiftTime;
      }
    }
  }
  rArg->comms->recordingEnabled[rArg->id-1] = 0;
  free(isMoving);
  return NULL;
}

#define RECORD_ANGLE_ALLOC_SIZE 16
void* Mobot_recordAngleBeginThread(void* arg)
{
  double angles[4];
  recordAngleArg_t *rArg = (recordAngleArg_t*) arg;
  int i; int rc; int retries = 0;
  int isMoving;
  MUTEX_LOCK(rArg->comms->recordingActive_lock);
  rArg->comms->recordingActive[rArg->id-1] = 1;
  COND_SIGNAL(rArg->comms->recordingActive_cond);
  MUTEX_UNLOCK(rArg->comms->recordingActive_lock);
#ifndef _WIN32
  struct timespec cur_time, itime;
  unsigned int dt;
  double start_time;
  MUTEX_LOCK(rArg->comms->recordingLock);
  for(i = 0; rArg->comms->recordingEnabled[rArg->id-1] ; i++) {
    MUTEX_UNLOCK(rArg->comms->recordingLock);
    rArg->i = i;
    rArg->comms->recordingNumValues[rArg->id-1] = i;
    /* Make sure we have enough space left in the buffer */
    if(i >= rArg->num) {
      rArg->num += RECORD_ANGLE_ALLOC_SIZE;
      double *newBuf;
      newBuf = (double*)malloc(sizeof(double) * rArg->num);
      memcpy(newBuf, *rArg->time_p, sizeof(double)*i);
      free(*rArg->time_p);
      *rArg->time_p = newBuf;
      newBuf = (double*)malloc(sizeof(double) * rArg->num);
      memcpy(newBuf, *rArg->angle_p, sizeof(double)*i);
      free(*rArg->angle_p);
      *rArg->angle_p = newBuf;
    }
#ifndef __MACH__
    clock_gettime(CLOCK_REALTIME, &cur_time);
#else
    clock_serv_t cclock;
    mach_timespec_t mts;
    host_get_clock_service(mach_host_self(), CALENDAR_CLOCK, &cclock);
    clock_get_time(cclock, &mts);
    mach_port_deallocate(mach_task_self(), cclock);
    cur_time.tv_sec = mts.tv_sec;
    cur_time.tv_nsec = mts.tv_nsec;
#endif
    //Mobot_getJointAngleTime(rArg->comms, rArg->id, &((*rArg->time_p)[i]), &((*rArg->angle_p)[i]));
    while(
        (rc = Mobot_getJointAnglesTimeIsMoving(
                                               rArg->comms, 
                                               &((*rArg->time_p)[i]),
                                               &angles[0],
                                               &angles[1],
                                               &angles[2],
                                               &angles[3],
                                               &isMoving)) &&
        retries <= MAX_RETRIES)
    {
      retries++;
    }
    if(rc) break;
    retries = 0;
    (*rArg->angle_p)[i] = angles[rArg->id-1];
    if(i == 0) {
      start_time = (*rArg->time_p)[i];
    }
    (*rArg->time_p)[i] = (*rArg->time_p)[i] - start_time;
    /* Convert angle to degrees */
    (*rArg->angle_p)[i] = RAD2DEG((*rArg->angle_p)[i]);
#ifndef __MACH__
    clock_gettime(CLOCK_REALTIME, &itime);
#else
    host_get_clock_service(mach_host_self(), CALENDAR_CLOCK, &cclock);
    clock_get_time(cclock, &mts);
    mach_port_deallocate(mach_task_self(), cclock);
    cur_time.tv_sec = mts.tv_sec;
    cur_time.tv_nsec = mts.tv_nsec;
#endif
    dt = diff_msecs(cur_time, itime);
    if(dt < (rArg->msecs)) {
      usleep(rArg->msecs*1000 - dt*1000);
    }
    if(!isMoving && shiftDataIsEnabled(rArg->comms)) {
      i--;
    }
    MUTEX_LOCK(rArg->comms->recordingLock);
  }
  MUTEX_UNLOCK(rArg->comms->recordingLock);
#else
  DWORD cur_time, itime;
  unsigned int dt;
  double start_time;
  for(i = 0; rArg->comms->recordingEnabled[rArg->id-1] ; i++) {
    MUTEX_LOCK(rArg->comms->recordingLock);
    rArg->i = i;
    rArg->comms->recordingNumValues[rArg->id-1] = i;
    /* Make sure we have enough space left in the buffer */
    if(i >= rArg->num) {
      rArg->num += RECORD_ANGLE_ALLOC_SIZE;
      double *newBuf;
      newBuf = (double*)malloc(sizeof(double) * rArg->num);
      memcpy(newBuf, *rArg->time_p, sizeof(double)*i);
      free(*rArg->time_p);
      *rArg->time_p = newBuf;
      newBuf = (double*)malloc(sizeof(double) * rArg->num);
      memcpy(newBuf, *rArg->angle_p, sizeof(double)*i);
      free(*rArg->angle_p);
      *rArg->angle_p = newBuf;
    }
    cur_time = GetTickCount();
    //Mobot_getJointAngleTime(rArg->comms, rArg->id, &((*rArg->time_p)[i]), &((*rArg->angle_p)[i]));
    while(
        (rc = Mobot_getJointAnglesTimeIsMoving(
                                               rArg->comms, 
                                               &((*rArg->time_p)[i]),
                                               &angles[0],
                                               &angles[1],
                                               &angles[2],
                                               &angles[3],
                                               &isMoving)) &&
        retries <= MAX_RETRIES)
    {
      retries++;
    }
    if(rc) break;
    retries = 0;
    (*rArg->angle_p)[i] = angles[rArg->id-1];
    if(i == 0) {
      start_time = (*rArg->time_p)[i];
    }
    (*rArg->time_p)[i] = (*rArg->time_p)[i] - start_time;
    /* Convert angle to degrees */
    (*rArg->angle_p)[i] = RAD2DEG((*rArg->angle_p)[i]);
    itime = GetTickCount();
    dt = itime - cur_time;
    if(dt < (rArg->msecs)) {
      Sleep(rArg->msecs - dt);
    }
    if(!isMoving && shiftDataIsEnabled(rArg->comms)) {
      i--;
    }
    MUTEX_UNLOCK(rArg->comms->recordingLock);
  }
#endif
  /* Save the pointers to free on destruct */
  rArg->comms->itemsToFreeOnExit[rArg->comms->numItemsToFreeOnExit] = (*rArg->time_p);
  rArg->comms->numItemsToFreeOnExit++;
  rArg->comms->itemsToFreeOnExit[rArg->comms->numItemsToFreeOnExit] = (*rArg->angle_p);
  rArg->comms->numItemsToFreeOnExit++;
  MUTEX_LOCK(rArg->comms->recordingActive_lock);
  rArg->comms->recordingActive[rArg->id-1] = 0;
  COND_SIGNAL(rArg->comms->recordingActive_cond);
  MUTEX_UNLOCK(rArg->comms->recordingActive_lock);
  return NULL;
  free(rArg);
}

int Mobot_recordAngleBegin(mobot_t* comms,
                                     robotJointId_t id,
                                     double **time,
                                     double **angle,
                                     double timeInterval,
                                     int shiftData)
{
  THREAD_T thread;
  recordAngleArg_t *rArg;
  int msecs = timeInterval * 1000;
  if(comms->recordingEnabled[id-1]) {
    return -1;
  }
  *time = (double*)malloc(sizeof(double) * RECORD_ANGLE_ALLOC_SIZE);
  *angle = (double*)malloc(sizeof(double) * RECORD_ANGLE_ALLOC_SIZE);
  rArg = (recordAngleArg_t*)malloc(sizeof(recordAngleArg_t));
  rArg->comms = comms;
  rArg->id = id;
  rArg->time_p = time;
  rArg->angle_p = angle;
  rArg->num = RECORD_ANGLE_ALLOC_SIZE;
  rArg->msecs = msecs;
  comms->recordingEnabled[id-1] = 1;
  comms->recordedAngles[0] = angle;
  comms->recordedTimes = time;
  comms->shiftData = shiftData;
  THREAD_CREATE(&thread, Mobot_recordAngleBeginThread, rArg);
  return 0;
}

int Mobot_recordAngleEnd(mobot_t* comms, robotJointId_t id, int *num)
{
  int i;
  double timeShift = 0;
  int timeShiftIndex;
  /* Make sure it was recording in the first place */
  if(comms->recordingEnabled[id-1] == 0) {
    return -1;
  }
  MUTEX_LOCK(comms->recordingLock);
  comms->recordingEnabled[id-1] = 0;
  MUTEX_UNLOCK(comms->recordingLock);
  /* Wait for recording to finish */
  MUTEX_LOCK(comms->recordingActive_lock);
  while(comms->recordingActive[id-1] != 0) {
    COND_WAIT(comms->recordingActive_cond, comms->recordingActive_lock);
  }
  MUTEX_UNLOCK(comms->recordingActive_lock);
  *num = comms->recordingNumValues[id-1];
  return 0;
}

int Mobot_recordDistanceBegin(mobot_t* comms,
                                     robotJointId_t id,
                                     double **time,
                                     double **distance,
                                     double radius,
                                     double timeInterval,
                                     int shiftTime)
{
  comms->wheelRadius = radius;
  return Mobot_recordAngleBegin(comms, id, time, distance, timeInterval, shiftTime);
}

int Mobot_recordDistanceEnd(mobot_t* comms, robotJointId_t id, int *num)
{
  int i;
  Mobot_recordAngleEnd(comms, id, num);
  for(i = 0; i < *num; i++){ 
    (*comms->recordedAngles[0])[i] = DEG2RAD((*comms->recordedAngles[0])[i]) * comms->wheelRadius;
    (*comms->recordedAngles[0])[i] += comms->distanceOffset;
  }
  return 0;
}

void* recordAnglesThread(void* arg);
int Mobot_recordAngles(mobot_t* comms, 
                      double *time, 
                      double* angle1, 
                      double* angle2,
                      double* angle3,
                      double* angle4,
                      int num,
                      double timeInterval,
                      int shiftData)
{
  int i;
  THREAD_T thread;
  recordAngleArg_t *rArg;
  int msecs = timeInterval * 1000;
  for(i = 0; i < 4; i++) {
    if(comms->recordingEnabled[i]) {
      return -1;
    }
  }
  rArg = (recordAngleArg_t*)malloc(sizeof(recordAngleArg_t));
  rArg->comms = comms;
  rArg->time = time;
  rArg->angle = angle1;
  rArg->angle2 = angle2;
  rArg->angle3 = angle3;
  rArg->angle4 = angle4;
  rArg->num = num;
  rArg->msecs = msecs;
  comms->shiftData = shiftData;
  for(i = 0; i < 4; i++) {
    comms->recordingEnabled[i] = 1;
  }
  THREAD_CREATE(&thread, recordAnglesThread, rArg);
  return 0;
}

void* recordAnglesThread(void* arg)
{
  int *isMoving;
  int rc;
  recordAngleArg_t *rArg = (recordAngleArg_t*) arg;
  isMoving = (int*)malloc(sizeof(int)*rArg->num);
  int retries = 0;
#ifndef _WIN32
  int i;
  struct timespec cur_time, itime;
  unsigned int dt;
  double start_time;
  for(i = 0; i < rArg->num; i++) {
#ifndef __MACH__
    clock_gettime(CLOCK_REALTIME, &cur_time);
#else
    clock_serv_t cclock;
    mach_timespec_t mts;
    host_get_clock_service(mach_host_self(), CALENDAR_CLOCK, &cclock);
    clock_get_time(cclock, &mts);
    mach_port_deallocate(mach_task_self(), cclock);
    cur_time.tv_sec = mts.tv_sec;
    cur_time.tv_nsec = mts.tv_nsec;
#endif
    while( 
        (rc = Mobot_getJointAnglesTimeIsMoving(
                                               rArg->comms, 
                                               &rArg->time[i], 
                                               &rArg->angle[i], 
                                               &rArg->angle2[i], 
                                               &rArg->angle3[i], 
                                               &rArg->angle4[i],
                                               &isMoving[i])) &&
        (retries <= MAX_RETRIES)
        )
    {
      retries++;
    }
    if(rc) {
      break;
    }
    retries = 0;
    if(i == 0) {
      start_time = rArg->time[i];
    }
    rArg->time[i] = rArg->time[i] - start_time;
    /* Convert angle to degrees */
    rArg->angle[i] = RAD2DEG(rArg->angle[i]);
    rArg->angle2[i] = RAD2DEG(rArg->angle2[i]);
    rArg->angle3[i] = RAD2DEG(rArg->angle3[i]);
    rArg->angle4[i] = RAD2DEG(rArg->angle4[i]);
#ifndef __MACH__
    clock_gettime(CLOCK_REALTIME, &itime);
#else
    host_get_clock_service(mach_host_self(), CALENDAR_CLOCK, &cclock);
    clock_get_time(cclock, &mts);
    mach_port_deallocate(mach_task_self(), cclock);
    cur_time.tv_sec = mts.tv_sec;
    cur_time.tv_nsec = mts.tv_nsec;
#endif
    dt = diff_msecs(cur_time, itime);
    if(dt < (rArg->msecs)) {
      usleep(rArg->msecs*1000 - dt*1000);
    }
  }
#else
  int i;
  DWORD cur_time, itime;
  unsigned int dt;
  double start_time;
  for(i = 0; i < rArg->num; i++) {
    cur_time = GetTickCount();
    while(
        (rc = Mobot_getJointAnglesTimeIsMoving(
                                               rArg->comms, 
                                               &rArg->time[i], 
                                               &rArg->angle[i], 
                                               &rArg->angle2[i], 
                                               &rArg->angle3[i], 
                                               &rArg->angle4[i],
                                               &isMoving[i])) &&
        (retries <= MAX_RETRIES)
        )
    {
      retries++;
    }
    if(rc) {
      break;
    }
    retries = 0;
    if(i == 0) {
      start_time = rArg->time[i];
    }
    rArg->time[i] = rArg->time[i] - start_time;
    /* Convert angle to degrees */
    rArg->angle[i] = RAD2DEG(rArg->angle[i]);
    rArg->angle2[i] = RAD2DEG(rArg->angle2[i]);
    rArg->angle3[i] = RAD2DEG(rArg->angle3[i]);
    rArg->angle4[i] = RAD2DEG(rArg->angle4[i]);
    itime = GetTickCount();
    dt = itime - cur_time;
    if(dt < (rArg->msecs)) {
      Sleep(rArg->msecs - dt);
    }
  }
#endif
  double shiftTime;
  int shiftTimeIndex;
  int j;
  int done = 0;
  if(shiftDataIsEnabled(rArg->comms)) {
    rArg->comms->recordedAngles[0] = &rArg->angle;
    rArg->comms->recordedAngles[1] = &rArg->angle2;
    rArg->comms->recordedAngles[2] = &rArg->angle3;
    rArg->comms->recordedAngles[3] = &rArg->angle4;
    rArg->comms->recordedTimes = &rArg->time;
    for(i = 0; i < rArg->num; i++) {
      if(isMoving[i]) {
          shiftTime = (*rArg->comms->recordedTimes)[i];
          shiftTimeIndex = i;
          break;
      }
    }
    for(i = 0; i < rArg->num; i++) {
      if(i < shiftTimeIndex) {
        (*rArg->comms->recordedTimes)[i] = 0;
        for(j = 0; j < 4; j++) {
          (*rArg->comms->recordedAngles[j])[i] = (*rArg->comms->recordedAngles[j])[shiftTimeIndex];
        }
      } else {
        (*rArg->comms->recordedTimes)[i] = (*rArg->comms->recordedTimes)[i] - shiftTime;
      }
    }
  }
  for(i = 0; i < 4; i++) {
    rArg->comms->recordingEnabled[i] = 0;
  }
  free(isMoving);
  return NULL;
}

void* Mobot_recordAnglesBeginThread(void* arg)
{
  int i;
  int rc;
  int retries = 0;
  recordAngleArg_t *rArg = (recordAngleArg_t*) arg;
  MUTEX_LOCK(rArg->comms->recordingActive_lock);
  for(i = 0; i < 4; i++) {
    rArg->comms->recordingActive[i] = 1;
  }
  COND_SIGNAL(rArg->comms->recordingActive_cond);
  MUTEX_UNLOCK(rArg->comms->recordingActive_lock);
#ifndef _WIN32
  struct timespec cur_time, itime;
  unsigned int dt;
  double start_time;
  MUTEX_LOCK(rArg->comms->recordingLock);
  for(i = 0; rArg->comms->recordingEnabled[0] ; i++) {
    MUTEX_UNLOCK(rArg->comms->recordingLock);
    rArg->i = i;
    rArg->comms->recordingNumValues[0] = i;
    /* Make sure we have enough space left in the buffer */
    if(i >= rArg->num) {
      rArg->num += RECORD_ANGLE_ALLOC_SIZE;
      double *newBuf;
      newBuf = (double*)malloc(sizeof(double) * rArg->num);
      memcpy(newBuf, *rArg->time_p, sizeof(double)*i);
      free(*rArg->time_p);
      *rArg->time_p = newBuf;

      newBuf = (double*)malloc(sizeof(double) * rArg->num);
      memcpy(newBuf, *rArg->angle_p, sizeof(double)*i);
      free(*rArg->angle_p);
      *rArg->angle_p = newBuf;

      newBuf = (double*)malloc(sizeof(double) * rArg->num);
      memcpy(newBuf, *rArg->angle2_p, sizeof(double)*i);
      free(*rArg->angle2_p);
      *rArg->angle2_p = newBuf;

      newBuf = (double*)malloc(sizeof(double) * rArg->num);
      memcpy(newBuf, *rArg->angle3_p, sizeof(double)*i);
      free(*rArg->angle3_p);
      *rArg->angle3_p = newBuf;

      newBuf = (double*)malloc(sizeof(double) * rArg->num);
      memcpy(newBuf, *rArg->angle4_p, sizeof(double)*i);
      free(*rArg->angle4_p);
      *rArg->angle4_p = newBuf;
    }
#ifndef __MACH__
    clock_gettime(CLOCK_REALTIME, &cur_time);
#else
    clock_serv_t cclock;
    mach_timespec_t mts;
    host_get_clock_service(mach_host_self(), CALENDAR_CLOCK, &cclock);
    clock_get_time(cclock, &mts);
    mach_port_deallocate(mach_task_self(), cclock);
    cur_time.tv_sec = mts.tv_sec;
    cur_time.tv_nsec = mts.tv_nsec;
#endif
    while(rc = Mobot_getJointAnglesTime(
        rArg->comms, 
        &((*rArg->time_p)[i]), 
        &((*rArg->angle_p)[i]),
        &((*rArg->angle2_p)[i]),
        &((*rArg->angle3_p)[i]),
        &((*rArg->angle4_p)[i])
        ) &&
        (retries <= MAX_RETRIES)
        )
    {
      retries++;
    }
    if(rc) {
      break;
    }
    retries = 0;
    if(i == 0) {
      start_time = (*rArg->time_p)[i];
    }
    (*rArg->time_p)[i] = (*rArg->time_p)[i] - start_time;
    /* Convert angle to degrees */
    (*rArg->angle_p)[i] = RAD2DEG((*rArg->angle_p)[i]);
    (*rArg->angle2_p)[i] = RAD2DEG((*rArg->angle2_p)[i]);
    (*rArg->angle3_p)[i] = RAD2DEG((*rArg->angle3_p)[i]);
    (*rArg->angle4_p)[i] = RAD2DEG((*rArg->angle4_p)[i]);
#ifndef __MACH__
    clock_gettime(CLOCK_REALTIME, &itime);
#else
    host_get_clock_service(mach_host_self(), CALENDAR_CLOCK, &cclock);
    clock_get_time(cclock, &mts);
    mach_port_deallocate(mach_task_self(), cclock);
    cur_time.tv_sec = mts.tv_sec;
    cur_time.tv_nsec = mts.tv_nsec;
#endif
    dt = diff_msecs(cur_time, itime);
    if(dt < (rArg->msecs)) {
      usleep(rArg->msecs*1000 - dt*1000);
    }
    MUTEX_LOCK(rArg->comms->recordingLock);
  }
  MUTEX_UNLOCK(rArg->comms->recordingLock);
#else
  DWORD cur_time, itime;
  unsigned int dt;
  double start_time;
  for(i = 0; rArg->comms->recordingEnabled[0] ; i++) {
    MUTEX_LOCK(rArg->comms->recordingLock);
    rArg->i = i;
    rArg->comms->recordingNumValues[0] = i;
    /* Make sure we have enough space left in the buffer */
    if(i >= rArg->num) {
      rArg->num += RECORD_ANGLE_ALLOC_SIZE;
      double *newBuf;
      newBuf = (double*)malloc(sizeof(double) * rArg->num);
      memcpy(newBuf, *rArg->time_p, sizeof(double)*i);
      free(*rArg->time_p);
      *rArg->time_p = newBuf;

      newBuf = (double*)malloc(sizeof(double) * rArg->num);
      memcpy(newBuf, *rArg->angle_p, sizeof(double)*i);
      free(*rArg->angle_p);
      *rArg->angle_p = newBuf;

      newBuf = (double*)malloc(sizeof(double) * rArg->num);
      memcpy(newBuf, *rArg->angle2_p, sizeof(double)*i);
      free(*rArg->angle2_p);
      *rArg->angle2_p = newBuf;

      newBuf = (double*)malloc(sizeof(double) * rArg->num);
      memcpy(newBuf, *rArg->angle3_p, sizeof(double)*i);
      free(*rArg->angle3_p);
      *rArg->angle3_p = newBuf;

      newBuf = (double*)malloc(sizeof(double) * rArg->num);
      memcpy(newBuf, *rArg->angle4_p, sizeof(double)*i);
      free(*rArg->angle4_p);
      *rArg->angle4_p = newBuf;
    }
    cur_time = GetTickCount();
    while(
        (rc = Mobot_getJointAnglesTime(
                                       rArg->comms, 
                                       &((*rArg->time_p)[i]), 
                                       &((*rArg->angle_p)[i]),
                                       &((*rArg->angle2_p)[i]),
                                       &((*rArg->angle3_p)[i]),
                                       &((*rArg->angle4_p)[i])
                                      )) &&
        retries <= MAX_RETRIES)
    {
      retries++;
    }
    if(rc) {
      break;
    }
    retries = 0;
    if(i == 0) {
      start_time = (*rArg->time_p)[i];
    }
    (*rArg->time_p)[i] = (*rArg->time_p)[i] - start_time;
    /* Convert angle to degrees */
    (*rArg->angle_p)[i] = RAD2DEG((*rArg->angle_p)[i]);
    (*rArg->angle2_p)[i] = RAD2DEG((*rArg->angle2_p)[i]);
    (*rArg->angle3_p)[i] = RAD2DEG((*rArg->angle3_p)[i]);
    (*rArg->angle4_p)[i] = RAD2DEG((*rArg->angle4_p)[i]);
    itime = GetTickCount();
    dt = itime - cur_time;
    if(dt < (rArg->msecs)) {
      Sleep(rArg->msecs - dt);
    }
    MUTEX_UNLOCK(rArg->comms->recordingLock);
  }
#endif
  /* Save the pointers to free on destruct */
  rArg->comms->itemsToFreeOnExit[rArg->comms->numItemsToFreeOnExit] = (*rArg->time_p);
  rArg->comms->numItemsToFreeOnExit++;
  rArg->comms->itemsToFreeOnExit[rArg->comms->numItemsToFreeOnExit] = (*rArg->angle_p);
  rArg->comms->numItemsToFreeOnExit++;
  rArg->comms->itemsToFreeOnExit[rArg->comms->numItemsToFreeOnExit] = (*rArg->angle2_p);
  rArg->comms->numItemsToFreeOnExit++;
  rArg->comms->itemsToFreeOnExit[rArg->comms->numItemsToFreeOnExit] = (*rArg->angle3_p);
  rArg->comms->numItemsToFreeOnExit++;
  rArg->comms->itemsToFreeOnExit[rArg->comms->numItemsToFreeOnExit] = (*rArg->angle4_p);
  rArg->comms->numItemsToFreeOnExit++;
  MUTEX_LOCK(rArg->comms->recordingActive_lock);
  for(i = 0; i < 4; i++) {
    rArg->comms->recordingActive[i] = 0;
  }
  COND_SIGNAL(rArg->comms->recordingActive_cond);
  MUTEX_UNLOCK(rArg->comms->recordingActive_lock);
  return NULL;
  free(rArg);
}

int Mobot_recordAnglesBegin(mobot_t* comms,
                                     double **time,
                                     double **angle1,
                                     double **angle2,
                                     double **angle3,
                                     double **angle4,
                                     double timeInterval,
                                     int shiftData)
{
  THREAD_T thread;
  recordAngleArg_t *rArg;
  int msecs = timeInterval * 1000;
  int i;
  for(i = 0; i < 4; i++) {
    if(comms->recordingEnabled[i]) {
      return -1;
    }
  }
  *time = (double*)malloc(sizeof(double) * RECORD_ANGLE_ALLOC_SIZE);
  *angle1 = (double*)malloc(sizeof(double) * RECORD_ANGLE_ALLOC_SIZE);
  *angle2 = (double*)malloc(sizeof(double) * RECORD_ANGLE_ALLOC_SIZE);
  *angle3 = (double*)malloc(sizeof(double) * RECORD_ANGLE_ALLOC_SIZE);
  *angle4 = (double*)malloc(sizeof(double) * RECORD_ANGLE_ALLOC_SIZE);
  rArg = (recordAngleArg_t*)malloc(sizeof(recordAngleArg_t));
  rArg->comms = comms;
  rArg->time_p = time;
  rArg->angle_p = angle1;
  rArg->angle2_p = angle2;
  rArg->angle3_p = angle3;
  rArg->angle4_p = angle4;
  rArg->num = RECORD_ANGLE_ALLOC_SIZE;
  rArg->msecs = msecs;
  for(i = 0; i < 4; i++) {
    comms->recordingEnabled[i] = 1;
  }
  comms->shiftData = shiftData;
  comms->recordedAngles[0] = angle1;
  comms->recordedAngles[1] = angle2;
  comms->recordedAngles[2] = angle3;
  comms->recordedAngles[3] = angle4;
  comms->recordedTimes = time;
  THREAD_CREATE(&thread, Mobot_recordAnglesBeginThread, rArg);
  return 0;
}

int Mobot_recordAnglesEnd(mobot_t* comms, int* num)
{
  /* Make sure it was recording in the first place */
  int i, j, done = 0;
  double timeShift = 0;
  int timeShiftIndex;
  for(i = 0; i < 4; i++) {
    if(comms->recordingEnabled[i] == 0) {
      return -1;
    }
  }
  MUTEX_LOCK(comms->recordingLock);
  for(i = 0; i < 4; i++) {
    comms->recordingEnabled[i] = 0;
  }
  MUTEX_UNLOCK(comms->recordingLock);
  /* Wait for recording to finish */
  MUTEX_LOCK(comms->recordingActive_lock);
  int flag = 0;
  for(i = 0; i < 4; i++) {
    if(comms->recordingActive[i]) {
      flag = 1;
    }
  }
  while(flag) {
    COND_WAIT(comms->recordingActive_cond, comms->recordingActive_lock);
    flag = 0;
    for(i = 0; i < 4; i++) {
      if(comms->recordingActive[i]) {
        flag = 1;
      }
    }
  }
  MUTEX_UNLOCK(comms->recordingActive_lock);
  *num = comms->recordingNumValues[0];
  return 0;
}

int Mobot_recordDistancesBegin(mobot_t* comms,
                               double **time,
                               double **distance1,
                               double **distance2,
                               double **distance3,
                               double **distance4,
                               double radius, 
                               double timeInterval,
                               int shiftTime)
{
  comms->wheelRadius = radius;
  return Mobot_recordAnglesBegin(comms, 
      time,
      distance1,
      distance1,
      distance1,
      distance1,
      timeInterval,
      shiftTime);
}

int Mobot_recordDistancesEnd(mobot_t* comms, int *num)
{ 
  int i, j;
  Mobot_recordAnglesEnd(comms, num);
  for(i = 0; i < *num; i++) {
    for(j = 0; j < 4; j++) {
      (*comms->recordedAngles[j])[i] = DEG2RAD((*comms->recordedAngles[j])[i]) * comms->wheelRadius;
      (*comms->recordedAngles[j])[i] += comms->distanceOffset;
    }
  }
  return 0;
}

int Mobot_recordDistanceOffset(mobot_t* comms, double distance)
{
  comms->distanceOffset = distance;
  return 0;
}

int Mobot_recordWait(mobot_t* comms)
{
  int i;
  for(i = 0; i < 4; i++) {
    while(comms->recordingEnabled[i]) {
#ifndef _WIN32
      usleep(100000);
#else
	  Sleep(100);
#endif
    }
  }
  return 0;
}

