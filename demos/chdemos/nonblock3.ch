/* File: nonblock3.ch
   Roll and arch simultaneously. */
#include <mobot.h>
CMobot mobot;

/* Connect to the paired Mobot */
mobot.connect();

mobot.resetToZero();

printf("Rolling 360 degrees.\n");
mobot.moveForward(360);
printf("Rolling 360 degrees while arching.\n");
mobot.motionArchNB(15);
mobot.moveForwardNB(360);
mobot.moveWait();
