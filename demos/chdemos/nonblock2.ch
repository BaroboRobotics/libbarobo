/* File: nonblock2.ch
   use the non-blocking functoin moveNB() . */
#include <mobot.h>
CMobot mobot;

/* Connect to the paired Mobot */
mobot.connect();

mobot.resetToZero();

/* Rotate each of the faceplates by 360 degrees */
//mobot.moveJoint(ROBOT_JOINT1, 360); // Blocking version
mobot.moveJointNB(ROBOT_JOINT1, 360); // Non-Blocking version
mobot.moveJoint(ROBOT_JOINT4, 360);
