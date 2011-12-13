/* File: inchworm.ch 
 * Perform the "inchworm" gait four times */

#include <mobot.h>

CMobot robot;

/* Connect to the paired MoBot */
robot.connect();

/* Set robot motors to speed of 0.50 */
robot.setJointSpeedRatio(MOBOT_JOINT2, 0.50);
robot.setJointSpeedRatio(MOBOT_JOINT3, 0.50);

/* Set the robot to "home" position, where all joint angles are 0 degrees. */
robot.moveToZero();

/* Do the inchworm gait four times */
int i;
for(i = 0; i < 4; i++) {
  robot.motionInchwormLeft();
}

