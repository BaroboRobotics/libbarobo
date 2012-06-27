/* Filename: tumble.ch 
 * Tumbling mobot */
#include <mobot.h>
CMobot mobot;

/* Connect to the paired Mobot */
mobot.connect();
/* Set the mobot to "home" position, where all joint angles are 0 degrees. */
mobot.moveToZero();

/* Tumble two times */
mobot.motionTumbleLeft(2);
