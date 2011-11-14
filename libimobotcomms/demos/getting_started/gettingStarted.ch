/* Filename: gettingStarted.ch 
 * Move the robot endplates. */

#include <mobot.h>

CMobot robot;

/* Connect to the paired MoBot */
robot.connect();

/* Set the robot to "home" position, where all joint angles are 0 degrees. */
robot.moveToZero();

/* Rotate each of the faceplates by 90 degrees */
robot.moveJointTo(MOBOT_JOINT1, 90);
robot.moveJointTo(MOBOT_JOINT4, 90);

