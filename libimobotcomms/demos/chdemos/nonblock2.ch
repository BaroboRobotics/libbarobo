/* File: nonblock2.ch
   use the non-blocking functoin moveNB() . */
#include <mobot.h>
CMobot robot;

/* Connect to the paired Mobot */
robot.connect();

robot.moveToZero();

/* Rotate each of the faceplates by 360 degrees */
//robot.moveJoint(MOBOT_JOINT1, 360); // Blocking version
robot.moveJointNB(MOBOT_JOINT1, 360); // Non-Blocking version
robot.moveJoint(MOBOT_JOINT4, 360);
