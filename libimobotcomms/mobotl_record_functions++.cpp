#include <stdlib.h>
#include "mobot.h"
#include "mobot_internal.h"

int CMobotL::recordAngle(mobotJointId_t id, double* time, double* angle, int num, double seconds, int shiftTime)
{
  return Mobot_recordAngle(_comms, id, time, angle, num, seconds, shiftTime);
}

int CMobotL::recordAngleBegin(mobotJointId_t id, double* &time, double* &angle, double seconds, int shiftData)
{
return Mobot_recordAngleBegin(_comms, id, &time, &angle, seconds, shiftData);
}

int CMobotL::recordAngleEnd(mobotJointId_t id, int &num)
{
  return Mobot_recordAngleEnd(_comms, id, &num);
}

int CMobotL::recordAngles(double *time, 
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

int CMobotL::recordAnglesBegin(double* &time, 
    double* &angle1, 
    double* &angle2, 
    double* &angle3, 
    double seconds,
    int shiftData)
{
  double *angle4;
  return Mobot_recordAnglesBegin(_comms, &time, &angle1, &angle2, &angle3, &angle4, seconds, shiftData);
}

int CMobotL::recordAnglesEnd(int &num)
{
  return Mobot_recordAnglesEnd(_comms, &num);
}

int CMobotL::recordDistanceBegin( mobotJointId_t id,
                                     double* &time,
                                     double* &distance,
                                     double radius,
                                     double timeInterval,
                                     int shiftData)
{
  return Mobot_recordDistanceBegin(_comms, 
      id,
      &time,
      &distance,
      radius,
      timeInterval,
      shiftData);
}

int CMobotL::recordDistanceEnd( mobotJointId_t id, int &num)
{
  return Mobot_recordDistanceEnd(_comms, id, &num);
}

int CMobotL::recordDistancesBegin(
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

int CMobotL::recordDistancesEnd(int &num)
{
  return Mobot_recordDistancesEnd(_comms, &num);
}

int CMobotL::recordWait()
{
  return Mobot_recordWait(_comms);
}

