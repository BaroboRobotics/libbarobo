#include <math.h>
#include "GtkiMobotController.h"

int setMotorDirection(int motor, int direction)
{
  if(g_isConnected) {
    return Mobot_setJointDirection(imobotComms, (mobotJointId_t)motor, (mobotJointDirection_t)direction);
  } else if (g_localInit) {
    return iMobot_setJointDirection(iMobot, (mobotJointId_t)motor, (mobotJointDirection_t)direction);
  } else {
    fprintf(stderr, "Error: Not initialized or connected.\n");
    return -1;
  }
}

int setMotorSpeed(int motor, double speed)
{
  if(g_isConnected) {
    return Mobot_setJointSpeed(imobotComms, (mobotJointId_t)motor, speed);
  } else if (g_localInit) {
    return iMobot_setJointSpeed(iMobot, (mobotJointId_t)motor, speed);
  } else {
    fprintf(stderr, "Error: Not initialized or connected.\n");
    return -1;
  }
}

int stop()
{
  if(g_isConnected) {
    return Mobot_stop(imobotComms);
  } else if (g_localInit) {
    return iMobot_stop(iMobot);
  } else {
    fprintf(stderr, "Error: Not initialized or connected.\n");
    return -1;
  }
}

int setMotorPosition(int motor, double position)
{
  /* Convert the position to radians */
  position = position * M_PI / 180.0;
  if(g_isConnected) {
    return Mobot_moveJointToNB(imobotComms, (mobotJointId_t)motor, position);
  } else if (g_localInit) {
    return iMobot_moveJointToNB(iMobot, (mobotJointId_t)motor, position);
  } else {
    fprintf(stderr, "Error: Not initialized or connected.\n");
    return -1;
  }
}

int setMotorPositionPID(int motor, double position)
{
  /* Convert the position to radians */
  position = position * M_PI / 180.0;
  if(g_isConnected) {
    return Mobot_moveJointToPIDNB(imobotComms, (mobotJointId_t)motor, position);
  } else if (g_localInit) {
    return iMobot_moveJointToNB(iMobot, (mobotJointId_t)motor, position);
  } else {
    fprintf(stderr, "Error: Not initialized or connected.\n");
    return -1;
  }
}

int getMotorPosition(int motor, double *position)
{
  int code;
  if(g_isConnected) {
    code = Mobot_getJointAngle(imobotComms, (mobotJointId_t)motor, position);
  } else if (g_localInit) {
    code = iMobot_getJointAngle(iMobot, (mobotJointId_t)motor, position);
  } else {
    fprintf(stderr, "Error: Not initialized or connected.\n");
    code = -1;
  }
  /* Convert position from radians to degrees */
  *position = *position * 180 / M_PI;
  return code;
}

int waitMotor(int motor)
{
  if(g_isConnected) {
    return Mobot_moveJointWait(imobotComms, (mobotJointId_t)motor);
  } else if (g_localInit) {
    return iMobot_moveJointWait(iMobot, (mobotJointId_t)motor);
  } else {
    fprintf(stderr, "Error: Not initialized or connected.\n");
    return -1;
  }
}
