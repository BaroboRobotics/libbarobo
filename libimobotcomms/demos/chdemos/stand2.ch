/* Filename: stand2.ch 
 * Make a Mobot stand up on a faceplate */
#include <mobot.h>
CMobot mobot;

/* Connect to the paired Mobot */
mobot.connect();

/* Set mobot motors to speed of 90 degrees per second */
mobot.setJointSpeed(ROBOT_JOINT2, 90);
mobot.setJointSpeed(ROBOT_JOINT3, 90);
/* Set the mobot to "home" position, where all joint angles are 0 degrees. */
mobot.resetToZero();

/* Move the mobot into a fetal position */
mobot.moveJointTo(ROBOT_JOINT2, -85);
mobot.moveJointTo(ROBOT_JOINT3, 70);

/* Wait a second for the mobot to settle down */
delay(1);

/* Rotate the bottom faceplate by 45 degrees */
mobot.moveJointTo(ROBOT_JOINT1, 45);

/* Lift the body up */
mobot.moveJointTo(ROBOT_JOINT2, 20);

/* Pan the mobot around for 3 seconds at 45 degrees per second*/
mobot.setJointSpeed(ROBOT_JOINT1, 45);
mobot.setMovementStateTime(ROBOT_FORWARD, ROBOT_HOLD, ROBOT_HOLD, ROBOT_HOLD, 3);

