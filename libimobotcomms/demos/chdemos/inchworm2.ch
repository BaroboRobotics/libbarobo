/* File: inchworm2.ch 
 * Perform the "inchworm" motion four times */

#include <mobot.h>

CMobot robot;

/* Connect to the paired MoBot */
robot.connect();

/* Set robot motors to speed of 0.50 */
robot.setJointSpeedRatio(ROBOT_JOINT2, 0.50);
robot.setJointSpeedRatio(ROBOT_JOINT3, 0.50);

/* Set the robot to "home" position, where all joint angles are 0 degrees. */
robot.moveToZero();

/* Do the inchworm motion four times */
int i, num = 4;
double angle2 = -45;
double angle3 = 45;
for(i = 0; i < num; i++) {
  robot.moveJointTo(ROBOT_JOINT2, angle2);
  robot.moveJointTo(ROBOT_JOINT3, angle3);
  robot.moveJointTo(ROBOT_JOINT2, 0);
  robot.moveJointTo(ROBOT_JOINT3, 0);
}

