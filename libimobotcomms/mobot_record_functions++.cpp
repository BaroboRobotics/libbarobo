
#include "mobot.h"
#include "mobot_internal.h"

int CMobot::recordAngle(robotJointId_t id, double* time, double* angle, int num, double seconds, int shiftTime)
{
  return Mobot_recordAngle(_comms, id, time, angle, num, seconds, shiftTime);
}

int CMobot::recordAngleBegin(robotJointId_t id, double* &time, double* &angle, double seconds, int shiftData)
{
return Mobot_recordAngleBegin(_comms, id, &time, &angle, seconds, shiftData);
}

int CMobot::recordAngleEnd(robotJointId_t id, int &num)
{
  return Mobot_recordAngleEnd(_comms, id, &num);
}

int CMobot::recordAngles(double *time, 
    double *angle1, 
    double *angle2, 
    double *angle3, 
    double *angle4, 
    int num, 
    double seconds,
    int shiftData)
{
  return Mobot_recordAngles(_comms, time, angle1, angle2, angle3, angle4, num, seconds, shiftData);
}

int CMobot::recordAnglesBegin(double* &time, 
    double* &angle1, 
    double* &angle2, 
    double* &angle3, 
    double* &angle4, 
    double seconds,
    int shiftData)
{
  return Mobot_recordAnglesBegin(_comms, &time, &angle1, &angle2, &angle3, &angle4, seconds, shiftData);
}

int CMobot::recordAnglesEnd(int &num)
{
  return Mobot_recordAnglesEnd(_comms, &num);
}

int CMobot::recordDistanceBegin( robotJointId_t id,
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

int CMobot::recordDistanceEnd( robotJointId_t id, int &num)
{
  return Mobot_recordDistanceEnd(_comms, id, &num);
}

int CMobot::recordDistancesBegin(
    double* &time,
    double* &distance1,
    double* &distance2,
    double* &distance3,
    double* &distance4,
    double radius, 
    double timeInterval,
    int shiftData)
{
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

int CMobot::recordDistancesEnd(int &num)
{
  return Mobot_recordDistancesEnd(_comms, &num);
}

int CMobot::recordWait()
{
  return Mobot_recordWait(_comms);
}

