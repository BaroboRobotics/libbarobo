/* File: inchworm.cpp 
 * Perform the "inchworm" gait four times */

#include <mobot.h>

int main()
{
  CMobot mobot;

  /* Connect to the paired MoBot */
  mobot.connect();

  /* Set mobot motors to speed of 0.50 */
  int i;
  mobot.setJointSpeedRatio(ROBOT_JOINT1, 0.5);
  mobot.setJointSpeedRatio(ROBOT_JOINT2, 0.5);
  mobot.setJointSpeedRatio(ROBOT_JOINT3, 0.5);
  mobot.setJointSpeedRatio(ROBOT_JOINT4, 0.5);
  /* Set the mobot to "home" position, where all joint angles are 0 degrees. */
  mobot.moveToZero();

  /* Do the inchworm gait four times */
  for(i = 0; i < 4; i++) {
    mobot.moveJointTo(ROBOT_JOINT2, deg2rad(-45));
    mobot.moveJointTo(ROBOT_JOINT3, deg2rad(45));
    mobot.moveJointTo(ROBOT_JOINT2, 0);
    mobot.moveJointTo(ROBOT_JOINT3, 0);
  }

  return 0;
}
