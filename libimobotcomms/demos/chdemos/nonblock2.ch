/* File: nonblock2.ch
   use the non-blocking functoin moveNB() . */
#include <mobot.h>
CMobot mobot;

/* Connect to the paired Mobot */
mobot.connect();

mobot.moveToZero();

/* Rotate each of the faceplates by 360 degrees */
//mobot.moveJoint(MOBOT_JOINT1, 360); // Blocking version
mobot.moveJointNB(MOBOT_JOINT1, 360); // Non-Blocking version
mobot.moveJoint(MOBOT_JOINT4, 360);
