/* File: nonblock3.ch
   Roll and arch simultaneously. */
#include <mobot.h>

CMobot robot;

/* Connect to the paired MoBot */
robot.connect();

robot.moveToZero();
/* Rotate each of the faceplates by 90 degrees */

robot.motionRollForward(deg2rad(360));
robot.motionArchNB(deg2rad(90));
robot.motionRollForwardNB(deg2rad(360));
robot.motionWait();
