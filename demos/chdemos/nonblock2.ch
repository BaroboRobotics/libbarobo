/* File: nonblock2.ch
   use the non-blocking functoin moveNB() . */
#include <mobot.h>
CMobot robot;

/* Connect to the paired Mobot */
robot.connect();

robot.resetToZero();

/* Rotate each of the faceplates by 360 degrees */
//robot.moveJoint(ROBOT_JOINT1, 360); // Blocking version
robot.moveJointNB(ROBOT_JOINT1, 360); // Non-Blocking version
robot.moveJoint(ROBOT_JOINT4, 360);
