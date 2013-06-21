/* Filename: unstand.ch 
 * Drop the mobot down while it is standing on faceplate 1. */
#include <mobot.h>
CMobot robot;

/* Connect to the paired Mobot */
robot.connect();
robot.resetToZero();

robot.moveJointToNB(ROBOT_JOINT2, -85);
robot.moveJointToNB(ROBOT_JOINT3, 45);
robot.moveWait();
robot.resetToZero();
