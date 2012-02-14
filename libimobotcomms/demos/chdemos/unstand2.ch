/* Filename: unstand.ch 
 * Drop the robot down from a standing position. */
#include <mobot.h>
CMobot robot;

/* Connect to the paired Mobot */
robot.connect();

robot.moveJointToNB(ROBOT_JOINT2, -85);
robot.moveJointToNB(ROBOT_JOINT3, 45);
robot.moveWait();
robot.moveToZero();
