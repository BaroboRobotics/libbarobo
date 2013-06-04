/* Filename: unstand.ch 
 * Drop the mobot down while it is standing on faceplate 1. */
#include <mobot.h>
CMobot mobot;

/* Connect to the paired Mobot */
mobot.connect();
mobot.resetToZero();

mobot.moveJointToNB(ROBOT_JOINT2, -85);
mobot.moveJointToNB(ROBOT_JOINT3, 45);
mobot.moveWait();
mobot.resetToZero();
