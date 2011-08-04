#include "GtkiMobotController.h"

int setMotorDirection(int motor, int direction)
{
  if(g_isConnected) {
    return iMobotComms_setMotorDirection(imobotComms, motor, direction);
  } else if (g_localInit) {
    return iMobot_setMotorDirection(iMobot, motor, direction);
  } else {
    fprintf(stderr, "Error: Not initialized or connected.\n");
    return -1;
  }
}

int setMotorSpeed(int motor, int speed)
{
  if(g_isConnected) {
    return iMobotComms_setMotorSpeed(imobotComms, motor, speed);
  } else if (g_localInit) {
    return iMobot_setMotorSpeed(iMobot, motor, speed);
  } else {
    fprintf(stderr, "Error: Not initialized or connected.\n");
    return -1;
  }
}

int stop()
{
  if(g_isConnected) {
    return iMobotComms_stop(imobotComms);
  } else if (g_localInit) {
    return iMobot_stop(iMobot);
  } else {
    fprintf(stderr, "Error: Not initialized or connected.\n");
    return -1;
  }
}

int setMotorPosition(int motor, double position)
{
  if(g_isConnected) {
    return iMobotComms_setMotorPosition(imobotComms, motor, position);
  } else if (g_localInit) {
    return iMobot_setMotorPosition(iMobot, motor, position);
  } else {
    fprintf(stderr, "Error: Not initialized or connected.\n");
    return -1;
  }
}

int getMotorPosition(int motor, double *position)
{
  int code;
  if(g_isConnected) {
    code = iMobotComms_getMotorPosition(imobotComms, motor, position);
    return code;
  } else if (g_localInit) {
    code = iMobot_getMotorPosition(iMobot, motor, position);
    return code;
  } else {
    fprintf(stderr, "Error: Not initialized or connected.\n");
    return -1;
  }
}

int waitMotor(int motor)
{
  if(g_isConnected) {
    return iMobotComms_waitMotor(imobotComms, motor);
  } else if (g_localInit) {
    return iMobot_waitMotor(iMobot, motor);
  } else {
    fprintf(stderr, "Error: Not initialized or connected.\n");
    return -1;
  }
}
