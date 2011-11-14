/* File: inchworm.ch 
 * Perform the "inchworm" gait four times */

#include <mobot.h>

CMobot robot;

/* Connect to the paired MoBot */
robot.connect();

/* Set robot motors to speed of 0.50 */
int i;
for(i = MOBOT_JOINT1; i < MOBOT_NUM_JOINTS; i++) {
  robot.setJointSpeed(i, 0.50);
}

/* Set the robot to "home" position, where all joint angles are 0 degrees. */
robot.moveToZero();
robot.moveWait();

/* Do the inchworm gait four times */
for(i = 0; i < 4; i++) {
  robot.moveJointTo(MOBOT_JOINT2, -45);
  robot.moveJointWait(MOBOT_JOINT1);
  robot.moveJointTo(MOBOT_JOINT3, 45);
  robot.moveJointWait(MOBOT_JOINT2);
  robot.moveJointTo(MOBOT_JOINT2, 0);
  robot.moveJointWait(MOBOT_JOINT1);
  robot.moveJointTo(MOBOT_JOINT3, 0);
  robot.moveJointWait(MOBOT_JOINT2);
}

