#include <stdlib.h>
#include "mobot.h"
#include "mobot_internal.h"
#include "linkbot.h"

int CLinkbotI::recordAngles(double *time, 
    double *angle1, 
    double *angle2, 
    double *angle3, 
    int num, 
    double seconds,
    int shiftData)
{
  int rc;
  double *angle4;
  angle4 = (double*)malloc(sizeof(double)*num);
  rc = Mobot_recordAngles(_comms, time, angle1, angle2, angle3, angle4, num, seconds, shiftData);
  free(angle4);
  return rc;
}

int CLinkbotI::recordAnglesBegin(double* &time, 
    double* &angle1, 
    double* &angle2, 
    double* &angle3, 
    double seconds,
    int shiftData)
{
  double *angle4;
  return Mobot_recordAnglesBegin(_comms, &time, &angle1, &angle2, &angle3, &angle4, seconds, shiftData);
}

int CLinkbotI::recordDistancesBegin(
    double* &time,
    double* &distance1,
    double* &distance2,
    double* &distance3,
    double radius, 
    double timeInterval,
    int shiftData)
{
  double* distance4;
  return Mobot_recordDistancesBegin(_comms,
    &time,
    &distance1,
    &distance2,
    &distance3,
    &distance4,
    radius, 
    timeInterval,
    shiftData);
}

