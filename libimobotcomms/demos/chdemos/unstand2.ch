/* Filename: unstand.ch 
 * Drop the mobot down while it is standing on faceplate 1. */
#include <mobot.h>
CMobot mobot;

/* Connect to the paired Mobot */
mobot.connect();
mobot.moveToZero();

mobot.moveJointToNB(MOBOT_JOINT2, -85);
mobot.moveJointToNB(MOBOT_JOINT3, 45);
mobot.moveWait();
mobot.moveToZero();
