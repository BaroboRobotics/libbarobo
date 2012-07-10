/* File: inchworm.ch 
 * Perform the "inchworm" motion four times */
#include <mobot.h>
CMobot mobot;

/* Connect to the paired Mobot */
mobot.connect();

/* Set mobot motors to speed of 0.50 */
mobot.setJointSpeedRatio(MOBOT_JOINT2, 0.50);
mobot.setJointSpeedRatio(MOBOT_JOINT3, 0.50);

/* Set the mobot to "home" position, where all joint angles are 0 degrees. */
mobot.resetToZero();

/* Do the inchworm motion four times */
mobot.motionInchwormLeft(4);

