/* File: nonblock3.ch
   Roll and arch simultaneously. */
#include <mobot.h>
CMobot robot;

/* Connect to the paired Mobot */
robot.connect();

robot.resetToZero();

printf("Rolling 360 degrees.\n");
robot.moveForward(360);
printf("Rolling 360 degrees while arching.\n");
robot.motionArchNB(15);
robot.moveForwardNB(360);
robot.moveWait();
