/* Filename: simple.cpp
 * Rotate the faceplates by 90 degrees */
#include <mobot.h>

int main()
{
  CMobot mobot;

  /* Connect to the paired MoBot */
  mobot.connect();
  mobot.moveJointContinuousNB(MOBOT_JOINT1, MOBOT_FORWARD);
  sleep(1);
  mobot.stopAllJoints();

  return 0;
}
