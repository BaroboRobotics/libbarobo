/* Filename: stand2.ch 
 * Make a Mobot stand up on a faceplate */
#include <mobot.h>
CMobot mobot;

/* Connect to the paired Mobot */
mobot.connect();

/* Set mobot motors to speed of 90 degrees per second */
mobot.setJointSpeed(MOBOT_JOINT2, 90);
mobot.setJointSpeed(MOBOT_JOINT3, 90);
/* Set the mobot to "home" position, where all joint angles are 0 degrees. */
mobot.resetToZero();

/* Move the mobot into a fetal position */
mobot.moveJointTo(MOBOT_JOINT2, -85);
mobot.moveJointTo(MOBOT_JOINT3, 70);

/* Wait a second for the mobot to settle down */
delay(1);

/* Rotate the bottom faceplate by 45 degrees */
mobot.moveJointTo(MOBOT_JOINT1, 45);

/* Lift the body up */
mobot.moveJointTo(MOBOT_JOINT2, 20);

/* Pan the mobot around for 3 seconds at 45 degrees per second*/
mobot.setJointSpeed(MOBOT_JOINT1, 45);
mobot.setMovementStateTime(MOBOT_FORWARD, MOBOT_HOLD, MOBOT_HOLD, MOBOT_HOLD, 3);

