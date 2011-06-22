#include "GtkiMobotController.h"

int setMotorDirection(int motor, int direction)
{
  if(g_isConnected) {
    return BRComms_setMotorDirection(imobotComms, motor, direction);
  } else if (g_localInit) {
    return BR_setMotorDirection(iMobot, motor, direction);
  } else {
    fprintf(stderr, "Error: Not initialized or connected.\n");
    return -1;
  }
}

int setMotorSpeed(int motor, int speed)
{
  if(g_isConnected) {
    return BRComms_setMotorSpeed(imobotComms, motor, speed);
  } else if (g_localInit) {
    return BR_setMotorSpeed(iMobot, motor, speed);
  } else {
    fprintf(stderr, "Error: Not initialized or connected.\n");
    return -1;
  }
}

int stop()
{
  if(g_isConnected) {
    return BRComms_stop(imobotComms);
  } else if (g_localInit) {
    return BR_stop(iMobot);
  } else {
    fprintf(stderr, "Error: Not initialized or connected.\n");
    return -1;
  }
}

int setMotorPosition(int motor, int position)
{
  if(g_isConnected) {
    return BRComms_setMotorPosition(imobotComms, motor, position);
  } else if (g_localInit) {
    return BR_poseJoint(iMobot, motor, position);
  } else {
    fprintf(stderr, "Error: Not initialized or connected.\n");
    return -1;
  }
}

int getMotorPosition(int motor, int *position)
{
  short pos;
  int code;
  if(g_isConnected) {
    return BRComms_getMotorPosition(imobotComms, motor, position);
  } else if (g_localInit) {
    code = BR_getMotorPosition(iMobot, motor, &pos);
    *position = pos;
    return code;
  } else {
    fprintf(stderr, "Error: Not initialized or connected.\n");
    return -1;
  }
}

int waitMotor(int motor)
{
  if(g_isConnected) {
    return BRComms_waitMotor(imobotComms, motor);
  } else if (g_localInit) {
    return BR_waitMotor(iMobot, motor);
  } else {
    fprintf(stderr, "Error: Not initialized or connected.\n");
    return -1;
  }
}
