/* File: inchworm2.ch 
 * Perform the "inchworm" motion four times */
#include <mobot.h>
CMobot mobot;

/* Connect to the paired Mobot */
mobot.connect();

/* Set mobot motors to speed of 0.50 */
mobot.setJointSpeedRatio(ROBOT_JOINT2, 0.50);
mobot.setJointSpeedRatio(ROBOT_JOINT3, 0.50);

/* Set the mobot to "home" position, where all joint angles are 0 degrees. */
mobot.resetToZero();

/* Do the inchworm motion four times */
int i, num = 4;
double angle2 = -45;
double angle3 = 45;
for(i = 0; i < num; i++) {
    mobot.moveJointTo(ROBOT_JOINT2, angle2); /* Move joint 2 */
    mobot.moveJointTo(ROBOT_JOINT3, angle3); /* Move joint 3 */
    mobot.moveJointTo(ROBOT_JOINT2, 0);      /* Move joint 2 */
    mobot.moveJointTo(ROBOT_JOINT3, 0);      /* Move joint 3 back to zero position */
}

