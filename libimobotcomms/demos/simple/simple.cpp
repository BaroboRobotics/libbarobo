/* Filename: simple.cpp
 * Rotate the faceplates by 90 degrees */
#include <mobot.h>

int main()
{
  CMobot mobot;

  /* Connect to the paired MoBot */
  mobot.connect();
  mobot.setJointSpeedRatio(MOBOT_JOINT2, 1);
  for(int i = 0; ; i++) {
    if(i % 2) {
      mobot.moveJointContinuousNB(MOBOT_JOINT2, MOBOT_FORWARD);
    } else {
      mobot.moveJointContinuousNB(MOBOT_JOINT2, MOBOT_BACKWARD);
    }
    getchar();
  }

  return 0;
}
