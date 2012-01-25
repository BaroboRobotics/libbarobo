/* Filename: simple.cpp
 * Rotate the faceplates by 90 degrees */
#include <mobot.h>

int main()
{
  CMobot robot;

  /* Connect to the paired MoBot */
  robot.connect();

  robot.setJointSpeed(ROBOT_JOINT1, M_PI);
  robot.moveJointContinuousNB(ROBOT_JOINT1, ROBOT_JOINT_FORWARD);

  return 0;
}
