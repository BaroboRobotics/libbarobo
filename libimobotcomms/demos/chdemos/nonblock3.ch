/* File: nonblock3.ch
   Roll and arch simultaneously. */
#include <mobot.h>

CMobot robot;

/* Connect to the paired MoBot */
robot.connect();

robot.moveToZero();

printf("Rolling 360 degrees.\n");
robot.motionRollForward(deg2rad(360));
printf("Rolling 360 degrees while arching.\n");
robot.motionArchNB(deg2rad(90));
robot.motionRollForwardNB(deg2rad(360));
robot.motionWait();
