#include "GtkiMobotController.h"

int setMotorDirection(int motor, int direction)
{
  if(g_isConnected) {
    return Mobot_setJointDirection(imobotComms, motor, direction);
  } else if (g_localInit) {
    return iMobot_setJointDirection(iMobot, motor, direction);
  } else {
    fprintf(stderr, "Error: Not initialized or connected.\n");
    return -1;
  }
}

int setMotorSpeed(int motor, int speed)
{
  double lspeed;
  /* Need to convert integer speed to double [0,1] */
  if(speed < -100 || speed > 100) {
    return -1;
  }
  lspeed = (double)speed / 100.0;
  if(g_isConnected) {
    return Mobot_setJointSpeed(imobotComms, motor, lspeed);
  } else if (g_localInit) {
    return iMobot_setJointSpeed(iMobot, motor, lspeed);
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
  if(g_isConnected) {
    return Mobot_moveJointTo(imobotComms, motor, position);
  } else if (g_localInit) {
    return iMobot_moveJointTo(iMobot, motor, position);
  } else {
    fprintf(stderr, "Error: Not initialized or connected.\n");
    return -1;
  }
}

int getMotorPosition(int motor, double *position)
{
  int code;
  if(g_isConnected) {
    code = Mobot_getJointAngle(imobotComms, motor, position);
    return code;
  } else if (g_localInit) {
    code = iMobot_getJointAngle(iMobot, motor, position);
    return code;
  } else {
    fprintf(stderr, "Error: Not initialized or connected.\n");
    return -1;
  }
}

int waitMotor(int motor)
{
  if(g_isConnected) {
    return Mobot_moveJointWait(imobotComms, motor);
  } else if (g_localInit) {
    return iMobot_waitMotor(iMobot, motor);
  } else {
    fprintf(stderr, "Error: Not initialized or connected.\n");
    return -1;
  }
}
