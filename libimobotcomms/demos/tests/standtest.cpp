#include <mobot.h>
#include <stdio.h>
#include <stdlib.h>

#define ANGLE_TOLERANCE 2.0
#define ABS(x) ((x) < 0? -1*(x) : (x))

int checkAngle(double measuredAngle, double expectedAngle)
{
  if(ABS(measuredAngle-expectedAngle) > 2.0) {
    return -1;
  } else {
    return 0;
  }
}

void errMsg(const char* msg)
{
  char* buf;
  buf = (char*)malloc(sizeof(char)*(strlen(msg)+100));
  sprintf(buf, "Error: %s %s:%d\n", msg, __FILE__, __LINE__);
  fprintf(stderr, "%s", buf);
  free(buf);
}

int main()
{
  double angles[4];
  CMobot mobot;
  mobot.connect();
  if(!mobot.isConnected()) {
    errMsg("Connect failed");
    return -1;
  }
  mobot.disconnect();
  mobot.connect();
  if(!mobot.isConnected()) {
    errMsg("Reconnect failed");
    return -1;
  }

  mobot.setJointSpeeds(45, 45, 45, 45);
  mobot.resetToZero();

  /* Begin testing. First, rotate joint 2 */
  mobot.moveJointToNB(MOBOT_JOINT2, DEG2RAD(-85));
  /* This should take about 2 seconds */
  usleep(1000000);
  if (!mobot.isMoving()) {
    errMsg("Error: Robot should still be moving.");
    return -1;
  }
  usleep(1000000);
  if (mobot.isMoving()) {
    errMsg("Error: Robot did not reach joint angle in time.");
    return -1;
  }
  mobot.getJointAngle(MOBOT_JOINT2, angles[0]);
  if(checkAngle(angles[0], -85)) {
    errMsg("Angle mismatch");
    return -1;
  } 

  /*Now rotate joint 3 */
  mobot.moveJointTo(MOBOT_JOINT3, 70);
  mobot.moveJointTo(MOBOT_JOINT1, 45);
  mobot.moveJointTo(MOBOT_JOINT2, 20);
  
  /* Check the joint angles */
  mobot.getJointAngles(angles[0], angles[1], angles[2], angles[3]);
  if(checkAngle(angles[0], 45)) {
    errMsg("Angle Mismatch");
    return -1;
  }
  if(checkAngle(angles[1], 20)) {
    errMsg("Angle Mismatch");
    return -1;
  }
  if(checkAngle(angles[2], 70)) {
    errMsg("Angle Mismatch");
    return -1;
  }
  if(checkAngle(angles[3], 0)) {
    errMsg("Angle Mismatch");
    return -1;
  }
  return 0;
}
