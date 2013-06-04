#include <mobot.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define ANGLE_TOLERANCE 3.0
#define ABS(x) ((x) < 0? -1*(x) : (x))

int checkAngle(double measuredAngle, double expectedAngle)
{
  if(ABS(measuredAngle-expectedAngle) > ANGLE_TOLERANCE) {
    fprintf(stderr, "Angle Mismatch: %lf , expected %lf\n", measuredAngle, expectedAngle);
    return -1;
  } else {
    return 0;
  }
}

#define ERRMSG(msg) errMsg(msg, __FILE__, __LINE__)

void errMsg(const char* msg, const char* file, int line)
{
  char* buf;
  buf = (char*)malloc(sizeof(char)*(strlen(msg)+100));
  sprintf(buf, "Error: %s %s:%d\n", msg, file, line);
  fprintf(stderr, "%s", buf);
  free(buf);
}

int main()
{
  double angles[4];
  CMobot mobot;
  mobot.connect();
  if(!mobot.isConnected()) {
    ERRMSG("Connect failed");
    return -1;
  }
  mobot.disconnect();
  mobot.connect();
  if(!mobot.isConnected()) {
    ERRMSG("Reconnect failed");
    return -1;
  }

  mobot.setJointSpeeds(45, 45, 45, 45);
  mobot.resetToZero();

  /* Begin testing. First, rotate joint 2 */
  mobot.moveJointToNB(ROBOT_JOINT2, -85);
  /* This should take about 2 seconds */
#ifndef _WIN32
  usleep(1000000);
#else
  Sleep(1000);
#endif
  if (!mobot.isMoving()) {
    ERRMSG("Error: Robot should still be moving.");
    return -1;
  }
#ifndef _WIN32
  usleep(1000000);
#else
  Sleep(1000);
#endif
  if (mobot.isMoving()) {
    ERRMSG("Error: Robot did not reach joint angle in time.");
    return -1;
  }
  mobot.getJointAngle(ROBOT_JOINT2, angles[0]);
  if(checkAngle(angles[0], -85)) {
    ERRMSG("Angle mismatch");
    return -1;
  } 

  /*Now rotate joint 3 */
  mobot.moveJointTo(ROBOT_JOINT3, 70);
  mobot.moveJointTo(ROBOT_JOINT1, 45);
  mobot.moveJointTo(ROBOT_JOINT2, 20);
  
  /* Check the joint angles */
  mobot.getJointAngles(angles[0], angles[1], angles[2], angles[3]);
  if(checkAngle(angles[0], 45)) {
    ERRMSG("Angle Mismatch");
    return -1;
  }
  if(checkAngle(angles[1], 20)) {
    ERRMSG("Angle Mismatch");
    return -1;
  }
  if(checkAngle(angles[2], 70)) {
    ERRMSG("Angle Mismatch");
    return -1;
  }
  if(checkAngle(angles[3], 0)) {
    ERRMSG("Angle Mismatch");
    return -1;
  }

  mobot.disconnect();
  return 0;
}
