/* File: inchworm.cpp 
 * Perform the "inchworm" gait four times */

#include <mobot.h>

int main()
{
  CMobot robot;

  /* Connect to the paired MoBot */
  robot.connect();

  /* Set robot motors to speed of 0.50 */
  int i;
  robot.setJointSpeedRatio(ROBOT_JOINT1, 0.5);
  robot.setJointSpeedRatio(ROBOT_JOINT2, 0.5);
  robot.setJointSpeedRatio(ROBOT_JOINT3, 0.5);
  robot.setJointSpeedRatio(ROBOT_JOINT4, 0.5);
  /* Set the robot to "home" position, where all joint angles are 0 degrees. */
  robot.moveToZero();

  /* Do the inchworm gait four times */
  for(i = 0; i < 4; i++) {
    robot.moveJointTo(ROBOT_JOINT2, deg2rad(-45));
    robot.moveJointTo(ROBOT_JOINT3, deg2rad(45));
    robot.moveJointTo(ROBOT_JOINT2, 0);
    robot.moveJointTo(ROBOT_JOINT3, 0);
  }

  return 0;
}
