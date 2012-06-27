/* Filename: unstand.ch 
 * Drop the mobot down from a standing position. */
#include <mobot.h>
CMobot mobot;

/* Connect to the paired Mobot */
mobot.connect();

mobot.moveJointToNB(MOBOT_JOINT2, -85);
mobot.moveJointToNB(MOBOT_JOINT3, 45);
mobot.moveWait();
mobot.moveToZero();
