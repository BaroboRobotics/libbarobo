/* File: nonblock3.ch
   Roll and arch simultaneously. */
#include <mobot.h>
CMobot robot;

/* Connect to the paired MoBot */
robot.connect();

robot.moveToZero();

printf("Rolling 360 degrees.\n");
robot.motionRollForward(360);
printf("Rolling 360 degrees while arching.\n");
robot.motionArchNB(15);
robot.motionRollForwardNB(360);
robot.motionWait();
